"""
비전 노드 (책상 전체 스캔 + 특정 물체 찾기) 통합
카메라 구독/좌표변환 로직은 공용으로 한 번만 두고,
그 위에 세 가지(서비스 2개 + 액션 1개)를 얹음:

  - ScanRequest 서비스 ('yolo_scan_request')
      -> 책상 전체 스캔. 로봇제어가 웨이포인트마다 도착할 때 1번씩 호출.
         1프레임 인식 + DB Bridge 전송 후 바로 응답.

  - GripBoundingBox 서비스 ('grip_bounding_box')
      -> DB에 저장된 좌표로 이미 이동한 상태에서, 그 자리에 물건이 실제로
         있는지 재확인 + 그립에 필요한 정밀 정보(좌표/bbox/raw depth) 응답.
         응답 필드에 found가 없어서, 전부 0이면 "없음"으로 간주하는 규칙 사용
         (로봇제어와 확인 필요).

  - FindOrder 액션 ('find_target_order')
      -> grip_bounding_box에서 못 찾았을 때 호출. 특정 물체를 찾을 때까지
         계속 최신 프레임 확인, state로 진행상황 feedback,
         찾으면 result(found, coordinate, message) 반환.

★ 변경사항: 좌표 변환기를 RgbdPixelToBase -> CameraToBaseTransformer로 교체.
   (서비스/액션 통신은 이전 버전에서 이미 검증됐으므로, 나머지 구조는 그대로 유지)

좌표 변환 (CameraToBaseTransformer)
-----------------------------------
   p_base = T_base_gripper(robot pose) @ T_gripper_camera(NPY) @ p_camera
   - NPY에는 역행렬을 적용하지 않음 (통합 코드 요청사항 반영)
   - robot_pos는 TF에서 읽은 calibration_frame(link_6) 자세를 Doosan 방식
     (Z-Y'-Z'' 오일러각, mm/deg)으로 변환해서 사용
   - link_6 기준 Z에서 TCP 오프셋 250mm를 빼는 보정이 내장돼 있음
     (그리퍼 실측 길이와 다르면 CameraToBaseTransformer 내부의 250.0 값 수정 필요)

DB 연동
-------
Redis Bridge(ROS2 토픽 publish) 방식. pose는 현재 [x, y, z] mm 3개만 전송
(로봇 자세 각도 rx,ry,rz는 그리퍼 계산 로직 추가 시 반영 예정).
"""

from __future__ import annotations

import json
import math
import sys
import time
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import message_filters

import tf2_ros

# TODO 실제 인터페이스 패키지명 다시 한번 확인
from hey_doopal_msg.srv import ScanRequest, GripBoundingBox
from hey_doopal_msg.action import FindOrder


def bbox_center_xyxy(x_min: float, y_min: float, x_max: float, y_max: float) -> Tuple[int, int]:
    """bbox 좌상단/우하단 좌표로 중심 픽셀 계산"""
    return (
        int(round((float(x_min) + float(x_max)) / 2.0)),
        int(round((float(y_min) + float(y_max)) / 2.0)),
    )


def quaternion_to_rotation_matrix(
    x: float, y: float, z: float, w: float,
) -> np.ndarray:
    """TF 쿼터니언 -> 3x3 회전행렬 변환 (CameraToBaseTransformer가 사용)"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("Quaternion norm is zero")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class CameraToBaseTransformer:
    """NPY + TF를 이용해 RGB-D 픽셀을 base_link 좌표로 변환한다.
    (통합 코드에서 가져온 버전 - 역행렬 미적용, Doosan Z-Y'-Z'' 자세, TCP 250mm 보정 포함)
    """

    def __init__(
        self,
        *,
        node: Node,
        tf_buffer: tf2_ros.Buffer,
        transform_path: str,
        base_frame: str = "base_link",
        calibration_frame: str = "link_6",
        transform_direction: str = "gripper_to_camera",
        transform_translation_unit: str = "auto",
        tf_timeout_sec: float = 0.20,
        depth_scale_16u_m: float = 0.001,
        depth_roi_radius: int = 5,
        min_valid_depth_m: float = 0.15,
        max_valid_depth_m: float = 2.0,
        min_valid_depth_pixels: int = 8,
    ) -> None:
        self.node = node
        self.tf_buffer = tf_buffer
        self.base_frame = str(base_frame).strip().lstrip("/")
        self.calibration_frame = str(calibration_frame).strip().lstrip("/")
        self.tf_timeout = Duration(seconds=float(tf_timeout_sec))

        self.depth_scale_16u_m = float(depth_scale_16u_m)
        self.depth_roi_radius = max(1, int(depth_roi_radius))
        self.min_valid_depth_m = float(min_valid_depth_m)
        self.max_valid_depth_m = float(max_valid_depth_m)
        self.min_valid_depth_pixels = max(1, int(min_valid_depth_pixels))

        if self.calibration_frame == "gripper_tcp":
            self.node.get_logger().warning(
                "calibration_frame=gripper_tcp이면 TCP 250 mm가 중복될 수 있습니다. "
                "현재 NPY에는 link_6를 사용해야 합니다."
            )

        self.gripper2cam_path = str(Path(transform_path).expanduser().resolve())
        self.transform_translation_unit = str(transform_translation_unit).strip().lower()
        self._validate_gripper2cam_file()

    def _validate_gripper2cam_file(self) -> None:
        path = Path(self.gripper2cam_path)
        if not path.is_file():
            raise FileNotFoundError(f"Transform file not found: {path}")
        matrix = np.asarray(np.load(str(path)), dtype=np.float64)
        if matrix.shape not in {(3, 4), (4, 4)}:
            raise ValueError(f"Transform must be 4x4 or 3x4: {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("Transform contains NaN or Inf")
        self.node.get_logger().info(
            "Coordinate transform: base2gripper @ gripper2cam @ camera_point "
            "(NPY inverse is not used)"
        )

    def _load_gripper2cam_mm(self, gripper2cam_path: str) -> np.ndarray:
        gripper2cam = np.asarray(
            np.load(str(Path(gripper2cam_path).expanduser().resolve())),
            dtype=np.float64,
        )
        if gripper2cam.shape == (3, 4):
            gripper2cam = np.vstack((gripper2cam, np.array([[0.0, 0.0, 0.0, 1.0]])))
        if gripper2cam.shape != (4, 4):
            raise ValueError(f"Transform must be 4x4 or 3x4: {gripper2cam.shape}")
        if not np.all(np.isfinite(gripper2cam)):
            raise ValueError("Transform contains NaN or Inf")
        if abs(float(gripper2cam[3, 3])) < 1.0e-12:
            raise ValueError("Invalid homogeneous transform")

        gripper2cam = gripper2cam / float(gripper2cam[3, 3])
        unit = self.transform_translation_unit
        translation_norm = float(np.linalg.norm(gripper2cam[:3, 3]))
        if unit == "auto":
            unit = "m" if translation_norm < 2.0 else "mm"
        if unit == "m":
            gripper2cam[:3, 3] *= 1000.0
        elif unit != "mm":
            raise ValueError("transform_translation_unit must be auto, m, or mm")
        return gripper2cam

    @staticmethod
    def get_robot_pose_matrix(x, y, z, rx, ry, rz) -> np.ndarray:
        """Doosan 스타일 Z-Y'-Z'' pose [mm, deg] -> T_base_gripper 4x4 행렬"""
        a, b, c = np.deg2rad([rx, ry, rz])
        ca, sa = math.cos(a), math.sin(a)
        cb, sb = math.cos(b), math.sin(b)
        cc, sc = math.cos(c), math.sin(c)

        rot_z_a = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        rot_y_b = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]], dtype=np.float64)
        rot_z_c = np.array([[cc, -sc, 0.0], [sc, cc, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)

        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rot_z_a @ rot_y_b @ rot_z_c
        matrix[:3, 3] = [float(x), float(y), float(z)]
        return matrix

    @staticmethod
    def _rotation_matrix_to_zyz_deg(rotation: np.ndarray) -> np.ndarray:
        r = np.asarray(rotation, dtype=np.float64)
        beta = math.acos(float(np.clip(r[2, 2], -1.0, 1.0)))
        sin_beta = math.sin(beta)
        if abs(sin_beta) > 1.0e-9:
            alpha = math.atan2(float(r[1, 2]), float(r[0, 2]))
            gamma = math.atan2(float(r[2, 1]), float(-r[2, 0]))
        else:
            alpha = math.atan2(float(r[1, 0]), float(r[0, 0]))
            gamma = 0.0
        return np.rad2deg([alpha, beta, gamma]).astype(np.float64)

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos) -> np.ndarray:
        gripper2cam = self._load_gripper2cam_mm(gripper2cam_path)
        coord = np.append(np.asarray(camera_coords, dtype=np.float64), 1.0)

        x, y, z, rx, ry, rz = [float(v) for v in robot_pos]
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        if abs(float(td_coord[3])) < 1.0e-12:
            raise ValueError("Invalid transformed homogeneous coordinate")
        return td_coord[:3] / float(td_coord[3])

    def _median_depth_m(self, *, depth_image, depth_encoding, u, v, color_width, color_height) -> Optional[float]:
        if depth_image is None or depth_image.ndim < 2:
            return None

        depth_height, depth_width = depth_image.shape[:2]
        u_depth = int(round(u * depth_width / float(color_width)))
        v_depth = int(round(v * depth_height / float(color_height)))
        u_depth = int(np.clip(u_depth, 0, depth_width - 1))
        v_depth = int(np.clip(v_depth, 0, depth_height - 1))

        radius = self.depth_roi_radius
        x1 = max(0, u_depth - radius)
        x2 = min(depth_width, u_depth + radius + 1)
        y1 = max(0, v_depth - radius)
        y2 = min(depth_height, v_depth + radius + 1)

        patch = np.asarray(depth_image[y1:y2, x1:x2])
        if patch.size == 0:
            return None

        if depth_encoding in {"16UC1", "mono16"} or patch.dtype == np.uint16:
            patch_m = patch.astype(np.float32) * self.depth_scale_16u_m
        else:
            patch_m = patch.astype(np.float32)

        valid = patch_m[
            np.isfinite(patch_m)
            & (patch_m >= self.min_valid_depth_m)
            & (patch_m <= self.max_valid_depth_m)
        ]

        if valid.size < self.min_valid_depth_pixels:
            return None

        return float(np.median(valid))

    @staticmethod
    def _deproject_camera_m(*, u, v, depth_m, camera_info: CameraInfo) -> Optional[np.ndarray]:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            return None

        return np.array(
            [(float(u) - cx) * depth_m / fx, (float(v) - cy) * depth_m / fy, depth_m],
            dtype=np.float64,
        )

    def _lookup_robot_pos_mm(self, stamp) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.calibration_frame,
                Time.from_msg(stamp), timeout=self.tf_timeout,
            )
        except Exception:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.calibration_frame,
                    Time(), timeout=self.tf_timeout,
                )
            except Exception as error:
                self.node.get_logger().warning(
                    f"TF unavailable: {self.base_frame} <- {self.calibration_frame}: {error}",
                    throttle_duration_sec=2.0,
                )
                return None

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        rotation = quaternion_to_rotation_matrix(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w,
        )
        zyz_deg = self._rotation_matrix_to_zyz_deg(rotation)
        return np.array(
            [
                float(translation.x) * 1000.0,
                float(translation.y) * 1000.0,
                float(translation.z) * 1000.0,
                float(zyz_deg[0]), float(zyz_deg[1]), float(zyz_deg[2]),
            ],
            dtype=np.float64,
        )

    def pixel_to_base(
        self, *, u, v, depth_image, depth_encoding, camera_info,
        color_width, color_height, stamp,
    ) -> Optional[Dict[str, Any]]:
        depth_m = self._median_depth_m(
            depth_image=depth_image, depth_encoding=depth_encoding,
            u=int(u), v=int(v), color_width=int(color_width), color_height=int(color_height),
        )
        if depth_m is None:
            return None

        camera_point_m = self._deproject_camera_m(u=int(u), v=int(v), depth_m=depth_m, camera_info=camera_info)
        if camera_point_m is None:
            return None

        robot_pos = self._lookup_robot_pos_mm(stamp)
        if robot_pos is None:
            return None

        camera_point_mm = np.asarray(camera_point_m, dtype=np.float64) * 1000.0
        try:
            base_point_mm = self.transform_to_base(camera_point_mm, self.gripper2cam_path, robot_pos)
            gripper2cam = self._load_gripper2cam_mm(self.gripper2cam_path)
        except (OSError, ValueError) as error:
            self.node.get_logger().error(f"Coordinate transform failed: {error}")
            return None

        # ★ 복원: link_6(calibration_frame) 기준 좌표도 같이 계산
        point_camera_h = np.append(camera_point_mm, 1.0)
        point_calibration_h = gripper2cam @ point_camera_h
        if abs(float(point_calibration_h[3])) < 1.0e-12:
            return None
        point_calibration_mm = point_calibration_h[:3] / float(point_calibration_h[3])

        if not np.all(np.isfinite(base_point_mm)):
            return None

        # 프로젝트 기준 보정: link_6 기준 Z에서 TCP 오프셋 250mm 제거, 음수는 0으로 클램프
        base_point_mm = np.asarray(base_point_mm, dtype=np.float64).copy()
        base_point_mm[2] = max(0.0, float(base_point_mm[2]) - 250.0)

        return {
            "depth_m": depth_m,
            "camera_point_m": camera_point_mm / 1000.0,
            "calibration_point_m": point_calibration_mm / 1000.0,  # ★ 복원
            "base_point_m": base_point_mm / 1000.0,
            "robot_pose_mm_deg": np.asarray(robot_pos, dtype=np.float64).copy(),
        }


class ObjectDetectionNode(Node):
    def __init__(
        self,
        model,
        color_topic='/camera/camera/color/image_raw',
        depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/camera/color/camera_info',
        hand_eye_transform_path: str = str(
            Path(__file__).resolve().parent / "T_gripper2camera.npy"
        ),
        base_frame='base_link',          # 로봇제어 실제 이름 확인 필요
        calibration_frame='link_6',      # 실제 TF 이름 확인 필요
        full_scan_conf=0.6,
        find_target_conf=0.5,
        find_target_timeout=15.0,        # 초. 이 시간 안에 못 찾으면 is_found=False
        find_target_interval=0.15,       # 재시도 간격(초)
        grip_retry_attempts=3,           # grip_bounding_box: 순간 인식실패 대비 짧은 재시도 횟수
        grip_retry_interval=0.15,        # grip_bounding_box: 재시도 간격(초)
        per_class_conf_threshold=None,   # 예: {'airpods': 0.9} -> airpods만 0.9, 나머진 기존 threshold 그대로
    ):
        super().__init__('object_detection_node')
        self.model = model
        self.classNames = model.names
        self.bridge = CvBridge()
        self.full_scan_conf = full_scan_conf            # 전체 스캔용 
        self.find_target_conf = find_target_conf        # 타겟 스캔용 
        self.find_target_timeout = find_target_timeout
        self.find_target_interval = find_target_interval
        self.grip_retry_attempts = grip_retry_attempts
        self.grip_retry_interval = grip_retry_interval
        self.per_class_conf_threshold = per_class_conf_threshold or {}

        # ---------- ★ TF2 + 좌표 변환기 (CameraToBaseTransformer로 교체) ----------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.coordinate_transformer = CameraToBaseTransformer(
            node=self,
            tf_buffer=self.tf_buffer,
            transform_path=hand_eye_transform_path,
            base_frame=base_frame,
            calibration_frame=calibration_frame,
            transform_direction='gripper_to_camera',
            transform_translation_unit='auto',
        )

        # ---------- 카메라 관련 (공용) ----------
        self.latest_camera_info = None
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)  # ★ 변경: 기본 QoS -> sensor_data (BEST_EFFORT)

        self.latest_color = None
        self.latest_depth = None
        self.latest_depth_encoding = None
        self.latest_stamp = None  # ★ 신규: pixel_to_base가 정확한 시점의 TF 조회에 사용
        self.frame_lock = threading.Lock()

        # 서비스(전체 스캔)와 액션(타겟 찾기)이 동시에 self.model(...)을 호출할 수 있어서
        # (MultiThreadedExecutor라 실제로 동시 실행 가능) 모델 추론만은 순서 강제
        self.inference_lock = threading.Lock()

        color_sub = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data)  # ★ 변경
        depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)  # ★ 변경
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.frame_callback)

        # ---------- Redis Bridge 전송용 Publisher (검증됨, 그대로 유지) ----------
        self.object_detection_pub = self.create_publisher(
            String,
            '/assistive/object_detection',
            10,
        )

        # ---------- 서비스 서버: 전체 스캔 ----------
        service_cb_group = ReentrantCallbackGroup()
        self.scan_srv = self.create_service(
            ScanRequest, 'yolo_scan_request',
            self.handle_scan_request, callback_group=service_cb_group)

        # ---------- 서비스 서버: 그립 직전 정밀 확인 ----------
        self.grip_srv = self.create_service(
            GripBoundingBox, 'grip_bounding_box',
            self.handle_grip_bounding_box, callback_group=service_cb_group)

        # ---------- 액션 서버: 타겟 찾기 ----------
        action_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FindOrder,
            'find_target_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_cb_group,
        )

        self.get_logger().info(
            'ObjectDetectionNode 준비 완료. yolo_scan_request(서비스), '
            'grip_bounding_box(서비스), find_target_order(액션) 대기 중...')

    # ================= 카메라 수신 (공용) =================
    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg  # raw 메시지 그대로 저장 (변환기가 직접 씀)

    def frame_callback(self, color_msg: Image, depth_msg: Image):
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            # depth는 encoding 그대로 유지해야 변환기가 mm/m 판단 가능 -> passthrough
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as error:
            self.get_logger().error(f'cv_bridge error: {error}')
            return

        with self.frame_lock:
            self.latest_color = color_img
            self.latest_depth = depth_img
            self.latest_depth_encoding = depth_msg.encoding
            self.latest_stamp = color_msg.header.stamp  # ★ 신규

    def _class_name(self, class_id: int) -> str:
        """class_names가 dict든 list든 안전하게 라벨 문자열을 꺼냄"""
        if isinstance(self.classNames, dict):
            return str(self.classNames.get(class_id, f'class_{class_id}'))
        try:
            return str(self.classNames[class_id])
        except (IndexError, TypeError):
            return f'class_{class_id}'
        
    # ================= 인식 + 좌표 변환 (공용) =================
    def run_inference_on_latest_frame(self, conf_threshold, target_label=None):
        """target_label=None -> 전체 스캔, 지정하면 그 라벨만 필터링 (타겟 찾기)"""
        with self.frame_lock:
            if self.latest_color is None or self.latest_depth is None:
                return []
            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()
            depth_encoding = self.latest_depth_encoding
            stamp = self.latest_stamp

        camera_info = self.latest_camera_info
        if camera_info is None or stamp is None:
            return []

        with self.inference_lock:
            results = self.model(color_img, verbose=False)
        detections = []
        color_height, color_width = color_img.shape[:2]

        for r in results:
            if r.boxes is None:
                continue

            for box_idx, box in enumerate(r.boxes):
                confidence = float(box.conf[0].item())
                if confidence < float(conf_threshold):
                    continue

                class_id = int(box.cls[0].item())
                label = self._class_name(class_id)

                # 클래스별로 threshold를 다르게 주고 싶으면 여기서 오버라이드
                # (없으면 기존처럼 conf_threshold 그대로 사용)
                effective_threshold = float(self.per_class_conf_threshold.get(label, conf_threshold))
                if confidence < effective_threshold:
                    continue

                if target_label and label != target_label:
                    continue

                x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].detach().cpu().tolist()]
                center_u, center_v = bbox_center_xyxy(x1, y1, x2, y2)

                coordinate = self.coordinate_transformer.pixel_to_base(
                    u=center_u, v=center_v,
                    depth_image=depth_img,
                    depth_encoding=depth_encoding,
                    camera_info=camera_info,
                    color_width=color_width,
                    color_height=color_height,
                    stamp=stamp,
                )
                if coordinate is None:
                    # depth 유효값 부족 또는 TF(base_link<->calibration_frame) 조회 실패
                    continue

                # ★ 여기서 한 번에 mm로 변환 (이후 응답/DB 전송에서 다시 *1000 안 해도 됨)
                base_point_mm = np.asarray(coordinate['base_point_m'], dtype=np.float64) * 1000.0
                camera_point_mm = np.asarray(coordinate['camera_point_m'], dtype=np.float64) * 1000.0
                link6_point_mm = np.asarray(coordinate['calibration_point_m'], dtype=np.float64) * 1000.0
                robot_pose_mm_deg = np.asarray(coordinate['robot_pose_mm_deg'], dtype=np.float64)

                # ★ 신규: 세그멘테이션 마스크로 물체의 회전 각도 계산 (cv2.minAreaRect)
                # -seg 모델이라 r.masks가 있고, masks.xy[box_idx]가 box_idx번째 박스와
                # 짝지어진 폴리곤 점들(원본 이미지 픽셀 좌표)임. 로봇을 돌려가며 여러 프레임
                # 비교할 필요 없이, 이 한 프레임의 마스크만으로 바로 각도가 나옴.
                
                grip_angle_deg = 0.0
                if r.masks is not None and box_idx < len(r.masks.xy):
                    mask_polygon = r.masks.xy[box_idx]  # (N, 2) numpy array
                    if mask_polygon is not None and len(mask_polygon) >= 3:
                        rect = cv2.minAreaRect(mask_polygon.astype(np.float32))
                        # rect = ((center_x, center_y), (width, height), angle)
                        # OpenCV 버전에 따라 각도 범위(-90~0 또는 0~90)와 기준축이 달라질 수 있어
                        # 실제 로봇 좌표계 기준으로 부호/오프셋을 맞춰야 함 (아래 결과 확인 후 조정)
                        grip_angle_deg = float(rect[2])

                        # 디버그: 회전 방향 확인용 - 방향 확정되면 지워도 됨
                        # 물체를 손으로 천천히 돌리면서 이 로그 값이 커지는지/작아지는지 확인
                        self.get_logger().info(
                            f'[각도 디버그] {label} | rect=(cx={rect[0][0]:.1f}, cy={rect[0][1]:.1f}, '
                            f'w={rect[1][0]:.1f}, h={rect[1][1]:.1f}) | angle={grip_angle_deg:.1f}deg'
                        )

                detections.append({
                    'class_name': label,
                    'class_id': class_id,
                    'confidence': round(confidence, 4),
                    'coordinate_unit': 'mm',
                    'frame_id': self.coordinate_transformer.base_frame,
                    'x': round(float(base_point_mm[0]), 2),   # 로봇 베이스 좌표계 (mm)
                    'y': round(float(base_point_mm[1]), 2),
                    'z': round(float(base_point_mm[2]), 2),
                    # 인식 시점 link_6 자세 (deg)
                    'rx': round(float(robot_pose_mm_deg[3]), 2),
                    'ry': round(float(robot_pose_mm_deg[4]), 2),
                    'rz': round(float(robot_pose_mm_deg[5]), 2),
                    'grip_angle_deg': round(grip_angle_deg, 2),  # ★ 신규: minAreaRect 기반 그립 각도
                    'camera_x_mm': round(float(camera_point_mm[0]), 2),
                    'camera_y_mm': round(float(camera_point_mm[1]), 2),
                    'camera_z_mm': round(float(camera_point_mm[2]), 2),
                    'link6_x_mm': round(float(link6_point_mm[0]), 2),
                    'link6_y_mm': round(float(link6_point_mm[1]), 2),
                    'link6_z_mm': round(float(link6_point_mm[2]), 2),
                    'camera_depth_z': round(float(coordinate['depth_m']), 4),  # m 단위 그대로
                    'bbox_width': int(x2 - x1),
                    'bbox_height': int(y2 - y1),
                    'bbox_center_u': int(center_u),
                    'bbox_center_v': int(center_v),
                })

        return detections

    # ================= DB Bridge 전송 (검증됨, 변경 없음) =================
    def publish_to_db_bridge(self, detections):
        """
        UI ZIP의 ros_object_bridge.py가 구독하는
        /assistive/object_detection 토픽으로 객체 정보를 전송한다.
        pose는 현재 [x, y, z] mm 3개만 전송 (rx,ry,rz는 추후 추가 예정).
        """
        for detection in detections:
            class_name = detection.get('class_name')

            if not class_name:
                self.get_logger().error(
                    '[DB Bridge] class_name이 없어 전송하지 않습니다.')
                continue

            pose = [
                round(float(detection['x']), 2),
                round(float(detection['y']), 2),
                round(float(detection['z']), 2),
                round(float(detection['rx']), 2),
                round(float(detection['ry']), 2),
                round(float(detection['rz']), 2),
            ]

            payload = {
                'record_name': class_name,
                'data': {'pose': pose},
                'replace': True,
            }

            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            self.object_detection_pub.publish(message)

            self.get_logger().info(
                f'[DB Bridge] 객체 정보 발행: {class_name} -> {pose}')

    # ================= 서비스: 전체 스캔 =================
    def handle_scan_request(self, request, response):
        self.get_logger().info(f'[전체 스캔] 요청 수신 (waypoint_id="{request.waypoint_id}")')

        detections = self.run_inference_on_latest_frame(self.full_scan_conf)

        if detections:
            self.publish_to_db_bridge(detections)
            response.success = True
            response.message = f'{len(detections)}개 객체 스캔 및 DB Bridge 전송 완료'
            response.detected_count = len(detections)
        else:
            response.success = True
            response.message = '탐지된 객체 없음 (또는 depth/TF 문제로 계산 실패)'
            response.detected_count = 0

        self.get_logger().info(f'[전체 스캔] 응답: {response.message}')
        return response

    # ================= 서비스: 그립 직전 정밀 확인 =================
    def handle_grip_bounding_box(self, request, response):
        target_label = request.target
        self.get_logger().info(f'[그립 확인] 요청 수신: target="{target_label}"')

        best = None
        for attempt in range(1, self.grip_retry_attempts + 1):
            detections = self.run_inference_on_latest_frame(
                self.find_target_conf, target_label=target_label)

            if detections:
                best = max(detections, key=lambda d: d['confidence'])
                break

            self.get_logger().warn(
                f'[그립 확인] "{target_label}" {attempt}/{self.grip_retry_attempts}번째 '
                f'시도 실패, 재시도...')
            if attempt < self.grip_retry_attempts:
                time.sleep(self.grip_retry_interval)

        if best is not None:
            self.publish_to_db_bridge([best])

            response.coordinate = [float(best['x']), float(best['y']), float(best['z'])]  # 이미 mm
            response.bbox_width = float(best['bbox_width'])
            response.bbox_height = float(best['bbox_height'])
            response.camera_depth_z = float(best['camera_depth_z'])
            response.grip_angle_deg = float(best['grip_angle_deg'])  # ★ 신규: minAreaRect 기반 각도
            response.is_find = True
            self.get_logger().info(
                f'[그립 확인] "{target_label}" 확인됨, angle={best["grip_angle_deg"]}deg')
        else:
            # found 필드가 없는 인터페이스라 "전부 0"으로 없음을 표현
            # (로봇제어와 이 규칙 확인 필요) - grip_retry_attempts번 다 실패한 경우에만 여기 도달
            response.coordinate = [0.0, 0.0, 0.0]
            response.bbox_width = 0.0
            response.bbox_height = 0.0
            response.camera_depth_z = 0.0
            response.grip_angle_deg = 0.0
            response.is_find = False
            self.get_logger().warn(
                f'[그립 확인] "{target_label}" {self.grip_retry_attempts}번 재시도했지만 '
                f'끝내 못 찾음')

        return response

    # ================= 액션: 타겟 찾기 =================
    def goal_callback(self, goal_request):
        target_name = str(goal_request.target_name).strip()
        self.get_logger().info(f'[타겟 찾기] 목표 수신: target_name="{target_name}"')
        if not target_name:
            self.get_logger().warn('[타겟 찾기] target_name이 비어있는 goal -> 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(goal_handle):
        self.get_logger().info('[타겟 찾기] 취소 요청 수신')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_label = str(goal_handle.request.target_name).strip()
        feedback_msg = FindOrder.Feedback()
        result = FindOrder.Result()

        start = time.monotonic()  # ★ 시스템 시계 조정에 영향 안 받는 시계로 변경
        attempts = 0
        found_detection = None

        while rclpy.ok() and time.monotonic() - start < self.find_target_timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.found = False
                result.coordinate = [0.0, 0.0, 0.0]
                result.message = 'canceled'
                self.get_logger().info(f'[타겟 찾기] "{target_label}" 취소됨 ({attempts}번 시도)')
                return result

            attempts += 1
            feedback_msg.state = 'searching'
            goal_handle.publish_feedback(feedback_msg)

            detections = self.run_inference_on_latest_frame(
                self.find_target_conf, target_label=target_label)

            if detections:
                found_detection = max(detections, key=lambda d: d['confidence'])
                break

            time.sleep(self.find_target_interval)

        if found_detection is not None:
            feedback_msg.state = 'calculating'
            goal_handle.publish_feedback(feedback_msg)

            self.publish_to_db_bridge([found_detection])
            goal_handle.succeed()

            result.found = True
            result.coordinate = [
                float(found_detection['x']),  # 이미 mm
                float(found_detection['y']),
                float(found_detection['z']),
            ]
            result.message = found_detection['class_name']
            self.get_logger().info(f'[타겟 찾기] "{target_label}" {attempts}번 시도 만에 발견')
        else:
            goal_handle.abort()
            result.found = False
            result.coordinate = [0.0, 0.0, 0.0]
            result.message = ''
            self.get_logger().warn(
                f'[타겟 찾기] "{target_label}" {self.find_target_timeout}초 동안 못 찾음 '
                f'({attempts}번 시도)')

        return result

    def destroy_node(self):
        self._action_server.destroy()  # ★ 신규: 종료 시 액션 서버 명시적 정리
        return super().destroy_node()


def main() -> None:
    model_path = Path(__file__).resolve().parent / 'my_seg_best.pt'

    if not model_path.is_file():
        print(f'File not found: {model_path}', file=sys.stderr)
        sys.exit(1)

    suffix = model_path.suffix.lower()
    if suffix == '.pt':
        model = YOLO(str(model_path))
    elif suffix in {'.onnx', '.engine'}:
        model = YOLO(str(model_path), task='detect')
    else:
        print(f'Unsupported model format: {suffix}', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = ObjectDetectionNode(
        model,
        per_class_conf_threshold={'airpods' : 0.9, 'cable' : 0.5, 'drink' : 0.6, 'mouse' : 0.6}
    )
    # ★ num_threads는 명시하지 않음 - 스레드 고갈 이슈 때문에 기본값(보통 CPU 코어 수)이 더 안전.
    #   통합본은 num_threads=4로 명시했었는데, 그게 오히려 부족했을 수 있어 일단 자동값 유지.
    #   필요하면 여기에 num_threads=8 등으로 여유 있게 지정 가능.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()