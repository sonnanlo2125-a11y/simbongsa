#!/usr/bin/env python3
"""MediaPipe + RealSense 손바닥 3D 좌표 Action Server.

ROS 인터페이스
--------------
Action: /find_hand_order (hey_doopal_msg/action/FindOrder)
  Goal: target_name = "hand"
  Result: found, coordinate[x, y, z] mm, message
  Feedback: state

Safety topics:
  /find_hand_order/active     std_msgs/msg/Bool
  /find_hand_order/succeeded  std_msgs/msg/Bool

좌표 변환:
  p_base = T_base_calibration(TF) @ T_calibration_camera(NPY) @ p_camera

프로젝트의 기존 캘리브레이션 규칙을 유지하여 NPY 행렬은 역행렬 없이
직접 사용한다. 최종 Action Result와 진단 좌표는 소수점 둘째 자리까지
반올림한다.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Optional, Sequence, Tuple

import cv2
import mediapipe as mp
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from hey_doopal_msg.action import FindOrder
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class MediaPipePalm3DActionServer(Node):
    """손바닥을 검출하고 base 기준 3D 좌표를 반환한다."""

    PALM_LANDMARK_IDS = (0, 5, 9, 13, 17)
    VALID_TARGET_NAMES = {"hand", "palm", "손", "손바닥"}

    def __init__(self) -> None:
        super().__init__("mediapipe_palm_3d_action_server")

        script_dir = Path(__file__).resolve().parent
        self._declare_parameters(script_dir)
        self._read_parameters()
        self._validate_parameters()

        # NPY를 프레임마다 다시 읽지 않고 시작 시 한 번만 검증/로드한다.
        self.calibration_from_camera_mm = self._load_transform_mm(
            self.transform_path
        )

        self.callback_group = ReentrantCallbackGroup()
        self._goal_lock = threading.Lock()
        self._goal_reserved = False
        self.find_hand_active = False

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.filtered_camera_xyz_mm: Optional[np.ndarray] = None
        self.filtered_base_xyz_mm: Optional[np.ndarray] = None
        self.latest_base_xyz_mm: Optional[np.ndarray] = None
        self.last_stable_point_mm: Optional[np.ndarray] = None
        self.latest_valid_time = 0.0
        self.stable_frame_count = 0
        self.current_feedback_state = "idle"
        self.last_process_time = 0.0
        self.last_tf_warning_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=int(self.model_complexity),
            min_detection_confidence=float(self.min_detection_confidence),
            min_tracking_confidence=float(self.min_tracking_confidence),
        )

        # 화면 표시를 사용할 때만 drawing 객체를 만든다.
        self.mp_draw = mp.solutions.drawing_utils if self.show_window else None
        self.mp_styles = mp.solutions.drawing_styles if self.show_window else None

        self.camera_point_pub = self.create_publisher(
            PointStamped, "/mediapipe_palm_3d/camera_point_mm", 10
        )
        self.base_point_pub = self.create_publisher(
            PointStamped, "/mediapipe_palm_3d/base_point_mm", 10
        )
        self.detected_pub = self.create_publisher(
            Bool, "/mediapipe_palm_3d/detected", 10
        )
        self.info_pub = self.create_publisher(
            String, "/mediapipe_palm_3d/info", 10
        )

        scan_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.scan_active_pub = self.create_publisher(
            Bool, "/find_hand_order/active", scan_state_qos
        )
        self.scan_succeeded_pub = self.create_publisher(
            Bool, "/find_hand_order/succeeded", scan_state_qos
        )
        self._publish_scan_active(False)
        self._publish_scan_succeeded(False)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            self.color_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=int(self.sync_queue_size),
            slop=float(self.sync_slop_sec),
        )
        self.sync.registerCallback(self._synced_callback)

        self.action_server = ActionServer(
            self,
            FindOrder,
            "/find_hand_order",
            execute_callback=self.execute_find_hand,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "Hand detection Action Server ready: /find_hand_order"
        )
        self.get_logger().info(
            f"Coordinate TF: {self.base_frame} <- {self.calibration_frame}; "
            "NPY direct; unit=mm"
        )
        self.get_logger().info(
            f"Preview: {'enabled' if self.show_window else 'disabled'}"
        )
        if self.base_z_offset_mm != 0.0:
            self.get_logger().warning(
                f"Base Z correction: z = z - {self.base_z_offset_mm:.1f} mm; "
                f"nonnegative clamp={self.clamp_base_z_nonnegative}"
            )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self, script_dir: Path) -> None:
        defaults = {
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "base_frame": "base_link",
            "calibration_frame": "link_6",
            "transform_path": str(script_dir / "T_gripper2camera.npy"),
            "transform_translation_unit": "mm",
            "base_z_offset_mm": 250.0,
            "clamp_base_z_nonnegative": True,
            "tf_timeout_sec": 0.20,
            "model_complexity": 1,
            "min_detection_confidence": 0.60,
            "min_tracking_confidence": 0.60,
            "depth_window": 17,
            "min_depth_mm": 150.0,
            "max_depth_mm": 1500.0,
            "min_valid_depth_pixels": 10,
            "smoothing_alpha": 0.35,
            "publish_rate_hz": 15.0,
            "sync_queue_size": 10,
            "sync_slop_sec": 0.08,
            "find_hand_timeout_sec": 15.0,
            "stable_frames": 5,
            "stable_max_jump_mm": 35.0,
            "coordinate_max_age_sec": 0.35,
            "show_window": False,
            "mirror_view": False,
            "window_name": "MediaPipe Palm 3D Action Server",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        names = (
            "color_topic",
            "depth_topic",
            "camera_info_topic",
            "base_frame",
            "calibration_frame",
            "transform_path",
            "transform_translation_unit",
            "base_z_offset_mm",
            "clamp_base_z_nonnegative",
            "tf_timeout_sec",
            "model_complexity",
            "min_detection_confidence",
            "min_tracking_confidence",
            "depth_window",
            "min_depth_mm",
            "max_depth_mm",
            "min_valid_depth_pixels",
            "smoothing_alpha",
            "publish_rate_hz",
            "sync_queue_size",
            "sync_slop_sec",
            "find_hand_timeout_sec",
            "stable_frames",
            "stable_max_jump_mm",
            "coordinate_max_age_sec",
            "show_window",
            "mirror_view",
            "window_name",
        )
        for name in names:
            setattr(self, name, self.get_parameter(name).value)

        self.base_frame = str(self.base_frame).strip().lstrip("/")
        self.calibration_frame = str(self.calibration_frame).strip().lstrip("/")
        self.transform_path = str(Path(str(self.transform_path)).expanduser().resolve())
        self.transform_translation_unit = str(
            self.transform_translation_unit
        ).strip().lower()
        self.base_z_offset_mm = float(self.base_z_offset_mm)
        self.clamp_base_z_nonnegative = bool(self.clamp_base_z_nonnegative)
        self.tf_timeout = Duration(seconds=float(self.tf_timeout_sec))
        self.min_process_period = 1.0 / max(0.1, float(self.publish_rate_hz))

        self.depth_window = max(3, int(self.depth_window))
        if self.depth_window % 2 == 0:
            self.depth_window += 1

    def _validate_parameters(self) -> None:
        if not self.base_frame or not self.calibration_frame:
            raise ValueError("base_frame and calibration_frame must not be empty")
        if self.transform_translation_unit not in {"auto", "m", "mm"}:
            raise ValueError(
                "transform_translation_unit must be one of: auto, m, mm"
            )
        if not 0.0 < float(self.smoothing_alpha) <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if float(self.min_depth_mm) >= float(self.max_depth_mm):
            raise ValueError("min_depth_mm must be less than max_depth_mm")
        if int(self.min_valid_depth_pixels) < 1:
            raise ValueError("min_valid_depth_pixels must be >= 1")
        if int(self.stable_frames) < 1:
            raise ValueError("stable_frames must be >= 1")
        if float(self.stable_max_jump_mm) <= 0.0:
            raise ValueError("stable_max_jump_mm must be > 0")
        if float(self.find_hand_timeout_sec) <= 0.0:
            raise ValueError("find_hand_timeout_sec must be > 0")
        if float(self.coordinate_max_age_sec) <= 0.0:
            raise ValueError("coordinate_max_age_sec must be > 0")

    # ------------------------------------------------------------------
    # Transform
    # ------------------------------------------------------------------
    def _load_transform_mm(self, transform_path: str) -> np.ndarray:
        path = Path(transform_path)
        if not path.is_file():
            raise FileNotFoundError(f"Transform file not found: {path}")

        matrix = np.asarray(np.load(str(path)), dtype=np.float64)
        if matrix.shape == (3, 4):
            matrix = np.vstack(
                (matrix, np.array([[0.0, 0.0, 0.0, 1.0]], dtype=np.float64))
            )
        if matrix.shape != (4, 4):
            raise ValueError(f"Transform must be 3x4 or 4x4: {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("Transform contains NaN or Inf")
        if abs(float(matrix[3, 3])) < 1.0e-12:
            raise ValueError("Invalid homogeneous transform")

        matrix = matrix / float(matrix[3, 3])
        unit = self.transform_translation_unit
        translation_norm = float(np.linalg.norm(matrix[:3, 3]))
        if unit == "auto":
            unit = "m" if translation_norm < 2.0 else "mm"
        if unit == "m":
            matrix[:3, 3] *= 1000.0

        self.get_logger().info(
            f"Loaded transform once: {path}; translation unit={unit}; "
            "inverse=False"
        )
        return matrix

    @staticmethod
    def _quaternion_to_rotation_matrix(
        x: float, y: float, z: float, w: float
    ) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-12:
            raise ValueError("Invalid zero-length quaternion")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

    def _lookup_base_from_calibration_mm(self, stamp: Any) -> Optional[np.ndarray]:
        try:
            image_time = rclpy.time.Time.from_msg(stamp)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.calibration_frame,
                image_time,
                timeout=self.tf_timeout,
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.calibration_frame,
                    rclpy.time.Time(),
                    timeout=self.tf_timeout,
                )
            except TransformException as error:
                now = time.monotonic()
                if now - self.last_tf_warning_time >= 2.0:
                    self.get_logger().warning(
                        f"TF unavailable: {self.base_frame} <- "
                        f"{self.calibration_frame}: {error}"
                    )
                    self.last_tf_warning_time = now
                return None

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self._quaternion_to_rotation_matrix(
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
        )
        matrix[:3, 3] = [
            float(translation.x) * 1000.0,
            float(translation.y) * 1000.0,
            float(translation.z) * 1000.0,
        ]
        return matrix

    def _camera_to_base_mm(
        self, camera_xyz_mm: Sequence[float], stamp: Any
    ) -> Optional[np.ndarray]:
        base_from_calibration = self._lookup_base_from_calibration_mm(stamp)
        if base_from_calibration is None:
            return None

        camera_point = np.append(
            np.asarray(camera_xyz_mm, dtype=np.float64), 1.0
        )
        base_point_h = (
            base_from_calibration
            @ self.calibration_from_camera_mm
            @ camera_point
        )
        if abs(float(base_point_h[3])) < 1.0e-12:
            self.get_logger().error("Invalid transformed homogeneous coordinate")
            return None

        base_point_mm = base_point_h[:3] / float(base_point_h[3])
        if not np.all(np.isfinite(base_point_mm)):
            self.get_logger().error("Coordinate transform returned NaN or Inf")
            return None

        base_point_mm = base_point_mm.astype(np.float64, copy=True)
        corrected_z = float(base_point_mm[2]) - self.base_z_offset_mm
        if self.clamp_base_z_nonnegative:
            corrected_z = max(0.0, corrected_z)
        base_point_mm[2] = corrected_z
        return base_point_mm

    # ------------------------------------------------------------------
    # Action and safety state
    # ------------------------------------------------------------------
    @staticmethod
    def _bool_message(value: bool) -> Bool:
        message = Bool()
        message.data = bool(value)
        return message

    def _publish_scan_active(self, active: bool) -> None:
        self.scan_active_pub.publish(self._bool_message(active))

    def _publish_scan_succeeded(self, succeeded: bool) -> None:
        self.scan_succeeded_pub.publish(self._bool_message(succeeded))

    @staticmethod
    def _round_xyz_mm(values: Sequence[float]) -> list[float]:
        if len(values) < 3:
            raise ValueError("hand coordinate requires three values")
        rounded = [round(float(values[index]), 2) for index in range(3)]
        if not all(math.isfinite(value) for value in rounded):
            raise ValueError("hand coordinate contains NaN or Inf")
        return rounded

    def _reset_measurements(self) -> None:
        self.filtered_camera_xyz_mm = None
        self.filtered_base_xyz_mm = None
        self.latest_base_xyz_mm = None
        self.last_stable_point_mm = None
        self.latest_valid_time = 0.0
        self.stable_frame_count = 0

    def goal_callback(self, goal_request: FindOrder.Goal) -> GoalResponse:
        target = str(goal_request.target_name).strip().lower()
        if target not in self.VALID_TARGET_NAMES:
            self.get_logger().warning(
                f"Rejected target_name: {goal_request.target_name!r}"
            )
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_reserved:
                self.get_logger().warning("Another hand search goal is active")
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def _failed_result(message: str) -> FindOrder.Result:
        result = FindOrder.Result()
        result.found = False
        result.coordinate = [0.0, 0.0, 0.0]
        result.message = message
        return result

    def execute_find_hand(self, goal_handle) -> FindOrder.Result:
        self.find_hand_active = True
        self.current_feedback_state = "searching"
        self._reset_measurements()
        self._publish_scan_succeeded(False)
        self._publish_scan_active(True)
        self._publish_detected(False)
        self.get_logger().warning(
            "Hand scan active: SpeedL tracking must remain disabled"
        )

        started_at = time.monotonic()
        last_feedback = ""

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._failed_result("hand detection canceled")

                if (
                    time.monotonic() - started_at
                    > float(self.find_hand_timeout_sec)
                ):
                    goal_handle.abort()
                    return self._failed_result("hand not detected")

                if self.current_feedback_state != last_feedback:
                    feedback = FindOrder.Feedback()
                    feedback.state = self.current_feedback_state
                    goal_handle.publish_feedback(feedback)
                    last_feedback = self.current_feedback_state

                coordinate_is_fresh = (
                    self.latest_base_xyz_mm is not None
                    and time.monotonic() - self.latest_valid_time
                    <= float(self.coordinate_max_age_sec)
                )
                if (
                    coordinate_is_fresh
                    and self.stable_frame_count >= int(self.stable_frames)
                ):
                    coordinate = self._round_xyz_mm(self.latest_base_xyz_mm)
                    result = FindOrder.Result()
                    result.found = True
                    result.coordinate = coordinate
                    result.message = "hand detected"
                    goal_handle.succeed()

                    self._publish_scan_succeeded(True)
                    self.get_logger().info(
                        "Hand detected [mm]: "
                        f"[{coordinate[0]:.2f}, {coordinate[1]:.2f}, "
                        f"{coordinate[2]:.2f}]"
                    )
                    return result

                # 다른 executor thread가 Camera/TF callback을 처리한다.
                time.sleep(0.05)
        finally:
            self.find_hand_active = False
            self.current_feedback_state = "idle"
            self._publish_scan_active(False)
            self._publish_detected(False)
            with self._goal_lock:
                self._goal_reserved = False

    # ------------------------------------------------------------------
    # Camera and detection
    # ------------------------------------------------------------------
    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _get_intrinsics(self) -> Optional[Tuple[float, float, float, float]]:
        if self.camera_info is None:
            return None
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        return fx, fy, cx, cy

    def _depth_to_mm(self, depth_msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"
        )
        if depth_msg.encoding.lower() in {"16uc1", "mono16"} or depth.dtype == np.uint16:
            return depth.astype(np.float32)
        return depth.astype(np.float32) * 1000.0

    def _median_depth_mm(
        self,
        depth_mm: np.ndarray,
        u_color: int,
        v_color: int,
        color_width: int,
        color_height: int,
    ) -> Optional[float]:
        depth_height, depth_width = depth_mm.shape[:2]
        u_depth = int(round(u_color * depth_width / float(color_width)))
        v_depth = int(round(v_color * depth_height / float(color_height)))
        u_depth = int(np.clip(u_depth, 0, depth_width - 1))
        v_depth = int(np.clip(v_depth, 0, depth_height - 1))

        radius = self.depth_window // 2
        patch = depth_mm[
            max(0, v_depth - radius) : min(
                depth_height, v_depth + radius + 1
            ),
            max(0, u_depth - radius) : min(
                depth_width, u_depth + radius + 1
            ),
        ]
        valid = patch[
            np.isfinite(patch)
            & (patch >= float(self.min_depth_mm))
            & (patch <= float(self.max_depth_mm))
        ]
        if valid.size < int(self.min_valid_depth_pixels):
            return None
        return float(np.median(valid))

    @staticmethod
    def _deproject_mm(
        u: int,
        v: int,
        depth_mm: float,
        intrinsics: Tuple[float, float, float, float],
    ) -> np.ndarray:
        fx, fy, cx, cy = intrinsics
        return np.array(
            [
                (float(u) - cx) * depth_mm / fx,
                (float(v) - cy) * depth_mm / fy,
                depth_mm,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _smooth(
        previous: Optional[np.ndarray], current: np.ndarray, alpha: float
    ) -> np.ndarray:
        if previous is None:
            return current.astype(np.float64, copy=True)
        return alpha * current + (1.0 - alpha) * previous

    def _update_stability(self, base_xyz_mm: np.ndarray) -> None:
        if self.last_stable_point_mm is None:
            self.stable_frame_count = 1
        else:
            jump = float(np.linalg.norm(base_xyz_mm - self.last_stable_point_mm))
            self.stable_frame_count = (
                self.stable_frame_count + 1
                if jump <= float(self.stable_max_jump_mm)
                else 1
            )

        self.last_stable_point_mm = base_xyz_mm.copy()
        self.latest_base_xyz_mm = base_xyz_mm.copy()
        self.latest_valid_time = time.monotonic()
        self.current_feedback_state = (
            "hand detected"
            if self.stable_frame_count == 1
            else "stabilizing coordinate"
        )

    def _invalidate_measurement(self, feedback_state: str) -> None:
        self._reset_measurements()
        self.current_feedback_state = feedback_state
        self._publish_detected(False)

    @staticmethod
    def _point_message_mm(
        xyz_mm: Sequence[float], frame_id: str, stamp: Any
    ) -> PointStamped:
        rounded = MediaPipePalm3DActionServer._round_xyz_mm(xyz_mm)
        message = PointStamped()
        message.header.frame_id = frame_id
        message.header.stamp = stamp
        message.point.x = rounded[0]
        message.point.y = rounded[1]
        message.point.z = rounded[2]
        return message

    def _publish_detected(self, detected: bool) -> None:
        self.detected_pub.publish(self._bool_message(detected))

    def _publish_info(
        self,
        *,
        detected: bool,
        status: str,
        center_uv: Optional[Tuple[int, int]] = None,
        depth_mm: Optional[float] = None,
        camera_xyz_mm: Optional[Sequence[float]] = None,
        base_xyz_mm: Optional[Sequence[float]] = None,
    ) -> None:
        payload = {
            "detected": bool(detected),
            "status": status,
            "coordinate_unit": "mm",
            "center_uv_px": list(center_uv) if center_uv else None,
            "depth_mm": round(float(depth_mm), 2) if depth_mm is not None else None,
            "camera_xyz_mm": (
                self._round_xyz_mm(camera_xyz_mm)
                if camera_xyz_mm is not None
                else None
            ),
            "base_frame": self.base_frame,
            "base_xyz_mm": (
                self._round_xyz_mm(base_xyz_mm)
                if base_xyz_mm is not None
                else None
            ),
            "stable_frames": int(self.stable_frame_count),
            "action_active": bool(self.find_hand_active),
        }
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )
        self.info_pub.publish(message)

    def _synced_callback(self, color_msg: Image, depth_msg: Image) -> None:
        # 대기 중에는 이미지 변환과 MediaPipe 추론을 수행하지 않는다.
        if not self.find_hand_active:
            if self.show_window:
                try:
                    waiting = self.bridge.imgmsg_to_cv2(
                        color_msg, desired_encoding="bgr8"
                    )
                    self._show_waiting(waiting)
                except CvBridgeError:
                    pass
            return

        now = time.monotonic()
        if now - self.last_process_time < self.min_process_period:
            return
        self.last_process_time = now

        intrinsics = self._get_intrinsics()
        if intrinsics is None:
            self._invalidate_measurement("waiting camera info")
            self._publish_info(detected=False, status="WAITING_CAMERA_INFO")
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding="bgr8"
            )
            depth_mm = self._depth_to_mm(depth_msg)
        except CvBridgeError as error:
            self._invalidate_measurement("image conversion failed")
            self.get_logger().error(f"Image conversion failed: {error}")
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        detection = self.hands.process(rgb)
        rgb.flags.writeable = True

        annotated = bgr.copy() if self.show_window else None
        if not detection.multi_hand_landmarks:
            self._invalidate_measurement("searching")
            self._publish_info(detected=False, status="HAND_NOT_DETECTED")
            if annotated is not None:
                self._show_waiting(annotated, "HAND NOT DETECTED")
            return

        hand_landmarks = detection.multi_hand_landmarks[0]
        image_height, image_width = bgr.shape[:2]
        pixels = []
        for landmark_id in self.PALM_LANDMARK_IDS:
            landmark = hand_landmarks.landmark[landmark_id]
            pixels.append(
                (
                    int(
                        np.clip(
                            round(landmark.x * image_width),
                            0,
                            image_width - 1,
                        )
                    ),
                    int(
                        np.clip(
                            round(landmark.y * image_height),
                            0,
                            image_height - 1,
                        )
                    ),
                )
            )
        center_u = int(round(np.mean([point[0] for point in pixels])))
        center_v = int(round(np.mean([point[1] for point in pixels])))

        depth_value_mm = self._median_depth_mm(
            depth_mm,
            center_u,
            center_v,
            image_width,
            image_height,
        )
        if depth_value_mm is None:
            self._invalidate_measurement("invalid depth")
            self._publish_info(
                detected=False,
                status="INVALID_DEPTH",
                center_uv=(center_u, center_v),
            )
            if annotated is not None:
                self._show_waiting(annotated, "INVALID DEPTH")
            return

        camera_raw_mm = self._deproject_mm(
            center_u, center_v, depth_value_mm, intrinsics
        )
        base_raw_mm = self._camera_to_base_mm(
            camera_raw_mm, color_msg.header.stamp
        )
        if base_raw_mm is None:
            self._invalidate_measurement("waiting tf")
            self._publish_info(
                detected=False,
                status="TF_UNAVAILABLE",
                center_uv=(center_u, center_v),
                depth_mm=depth_value_mm,
                camera_xyz_mm=camera_raw_mm,
            )
            if annotated is not None:
                self._show_waiting(annotated, "BASE TF UNAVAILABLE")
            return

        alpha = float(self.smoothing_alpha)
        self.filtered_camera_xyz_mm = self._smooth(
            self.filtered_camera_xyz_mm, camera_raw_mm, alpha
        )
        self.filtered_base_xyz_mm = self._smooth(
            self.filtered_base_xyz_mm, base_raw_mm, alpha
        )
        self._update_stability(self.filtered_base_xyz_mm)

        camera_frame = color_msg.header.frame_id or "camera_color_optical_frame"
        self.camera_point_pub.publish(
            self._point_message_mm(
                self.filtered_camera_xyz_mm,
                camera_frame,
                color_msg.header.stamp,
            )
        )
        self.base_point_pub.publish(
            self._point_message_mm(
                self.filtered_base_xyz_mm,
                self.base_frame,
                color_msg.header.stamp,
            )
        )
        self._publish_detected(True)
        self._publish_info(
            detected=True,
            status="PALM_3D_OK",
            center_uv=(center_u, center_v),
            depth_mm=depth_value_mm,
            camera_xyz_mm=self.filtered_camera_xyz_mm,
            base_xyz_mm=self.filtered_base_xyz_mm,
        )

        if annotated is not None:
            assert self.mp_draw is not None and self.mp_styles is not None
            self.mp_draw.draw_landmarks(
                annotated,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_styles.get_default_hand_landmarks_style(),
                self.mp_styles.get_default_hand_connections_style(),
            )
            cv2.circle(annotated, (center_u, center_v), 9, (0, 0, 255), -1)
            x, y, z = self._round_xyz_mm(self.filtered_base_xyz_mm)
            cv2.putText(
                annotated,
                f"BASE [{x:.2f}, {y:.2f}, {z:.2f}] mm",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                f"Stable {self.stable_frame_count}/{self.stable_frames}",
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self._show(annotated)

    def _show_waiting(
        self, image: np.ndarray, text: str = "WAITING /find_hand_order"
    ) -> None:
        cv2.putText(
            image,
            text,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        self._show(image)

    def _show(self, image: np.ndarray) -> None:
        if not self.show_window:
            return
        display = cv2.flip(image, 1) if self.mirror_view else image
        cv2.imshow(str(self.window_name), display)
        key = cv2.waitKey(1) & 0xFF
        if key in {ord("q"), 27}:
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        # 종료 시 TRANSIENT_LOCAL 상태를 false로 남겨 재시작 노드를 disarm한다.
        try:
            self._publish_scan_active(False)
            self._publish_scan_succeeded(False)
            self._publish_detected(False)
        except Exception:
            pass
        self.action_server.destroy()
        self.hands.close()
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[MediaPipePalm3DActionServer] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = MediaPipePalm3DActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
