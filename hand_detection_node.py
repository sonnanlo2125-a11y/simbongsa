#!/usr/bin/env python3
"""MediaPipe + RealSense 손바닥 3D Action Server.

Action
------
Name: /find_hand_order
Type: hey_doopal_msg/action/FindOrder

Goal:
  target_name = "hand"

Result:
  found = true
  coordinate = [x_mm, y_mm, z_mm]
  message = "hand detected"

Feedback:
  state = searching / hand detected / stabilizing coordinate

SpeedL 안전 연동:
  /find_hand_order/active     std_msgs/msg/Bool
  /find_hand_order/succeeded  std_msgs/msg/Bool

손 좌표 Action Result는 소수점 둘째 자리까지 반환한다.

좌표 및 NPY 기준
----------------
- 모든 좌표는 base_link 기준 mm 단위다.
- 요청된 직접 행렬식을 사용한다.
- p_base = T_base_gripper(robot pose) @ T_gripper_camera(NPY) @ p_camera
- NPY에는 역행렬을 적용하지 않는다.
- T_gripper2camera.npy는 link_6 기준으로 사용한다.
- 기본 calibration_frame은 link_6이다.
- 최종 base Z는 max(0, z - 250 mm)로 보정한다.
"""

from __future__ import annotations

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
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
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
    PALM_LANDMARK_IDS = (0, 5, 9, 13, 17)

    def __init__(self) -> None:
        super().__init__("mediapipe_palm_3d_action_server")

        script_dir = Path(__file__).resolve().parent
        self.callback_group = ReentrantCallbackGroup()
        self._goal_lock = threading.Lock()
        self._goal_reserved = False
        self.find_hand_active = False

        self._declare_parameters(script_dir)
        self._read_parameters()
        self._validate_parameters()

        self.gripper2cam_path = str(Path(self.transform_path).expanduser().resolve())
        self._validate_gripper2cam_file()

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.filtered_camera_xyz_mm: Optional[np.ndarray] = None
        self.filtered_base_xyz_mm: Optional[np.ndarray] = None
        self.latest_base_xyz_mm: Optional[np.ndarray] = None
        self.latest_valid_time = 0.0
        self.stable_frame_count = 0
        self.last_stable_point_mm: Optional[np.ndarray] = None
        self.last_process_time = 0.0
        self.last_tf_warning_time = 0.0
        self.current_feedback_state = "idle"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=self.model_complexity,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )

        self.camera_point_pub = self.create_publisher(
            PointStamped,
            "/mediapipe_palm_3d/camera_point_mm",
            10,
        )
        self.base_point_pub = self.create_publisher(
            PointStamped,
            "/mediapipe_palm_3d/base_point_mm",
            10,
        )
        self.detected_pub = self.create_publisher(
            Bool,
            "/mediapipe_palm_3d/detected",
            10,
        )
        self.info_pub = self.create_publisher(
            String,
            "/mediapipe_palm_3d/info",
            10,
        )

        # 손 스캔과 SpeedL 추적을 명확히 분리하기 위한 상태 신호.
        # TRANSIENT_LOCAL을 사용해 SpeedL 노드가 늦게 시작해도
        # 최신 상태를 즉시 받을 수 있다.
        scan_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.scan_active_pub = self.create_publisher(
            Bool,
            "/find_hand_order/active",
            scan_state_qos,
        )
        self.scan_succeeded_pub = self.create_publisher(
            Bool,
            "/find_hand_order/succeeded",
            scan_state_qos,
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
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
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
            "Hand detection Action Server started: /find_hand_order"
        )
        self.get_logger().info(
            "Hand scan state topics: "
            "/find_hand_order/active, /find_hand_order/succeeded"
        )
        self.get_logger().info(
            "Camera preview disabled (headless mode)"
        )
        self.get_logger().info(
            f"TF calculation: {self.base_frame} <- {self.calibration_frame}, mm"
        )
        self.get_logger().warning(
            "Base Z correction enabled: "
            f"z=max(0, z-{self.base_z_offset_mm:.1f} mm)"
        )

    # ------------------------------------------------------------------
    # Hand scan state signals
    # ------------------------------------------------------------------
    def _publish_scan_active(self, active: bool) -> None:
        message = Bool()
        message.data = bool(active)
        self.scan_active_pub.publish(message)

    def _publish_scan_succeeded(self, succeeded: bool) -> None:
        message = Bool()
        message.data = bool(succeeded)
        self.scan_succeeded_pub.publish(message)

    @staticmethod
    def _round_xyz_mm(values: Sequence[float]) -> list[float]:
        if len(values) < 3:
            raise ValueError("hand coordinate requires three values")
        rounded = [
            round(float(values[index]), 2)
            for index in range(3)
        ]
        if not all(math.isfinite(value) for value in rounded):
            raise ValueError("hand coordinate contains NaN or Inf")
        return rounded

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
            "transform_direction": "gripper_to_camera",
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
        names = [
            "color_topic",
            "depth_topic",
            "camera_info_topic",
            "base_frame",
            "calibration_frame",
            "transform_path",
            "transform_direction",
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
        ]
        for name in names:
            setattr(self, name, self.get_parameter(name).value)

        self.base_frame = str(self.base_frame).strip().lstrip("/")
        self.calibration_frame = str(self.calibration_frame).strip().lstrip("/")
        self.transform_direction = str(self.transform_direction).strip().lower()
        self.transform_translation_unit = (
            str(self.transform_translation_unit).strip().lower()
        )
        self.base_z_offset_mm = float(self.base_z_offset_mm)
        self.clamp_base_z_nonnegative = bool(self.clamp_base_z_nonnegative)
        self.tf_timeout = Duration(seconds=float(self.tf_timeout_sec))
        self.min_process_period = 1.0 / max(0.1, float(self.publish_rate_hz))

        self.depth_window = int(self.depth_window)
        if self.depth_window < 3:
            self.depth_window = 3
        if self.depth_window % 2 == 0:
            self.depth_window += 1

    def _validate_parameters(self) -> None:
        if not 0.0 < float(self.smoothing_alpha) <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if int(self.stable_frames) < 1:
            raise ValueError("stable_frames must be >= 1")
        if float(self.find_hand_timeout_sec) <= 0:
            raise ValueError("find_hand_timeout_sec must be > 0")

    # ------------------------------------------------------------------
    # Requested direct coordinate transform
    # ------------------------------------------------------------------
    def _validate_gripper2cam_file(self) -> None:
        path = Path(self.gripper2cam_path)
        if not path.is_file():
            raise FileNotFoundError(f"Transform file not found: {path}")

        matrix = np.asarray(np.load(str(path)), dtype=np.float64)
        if matrix.shape not in {(3, 4), (4, 4)}:
            raise ValueError(f"Transform must be 4x4 or 3x4: {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("Transform contains NaN or Inf")

        self.get_logger().info(
            "Coordinate transform: base2gripper @ gripper2cam @ camera_point "
            "(NPY inverse is not used)"
        )

    def _load_gripper2cam_mm(self, gripper2cam_path: str) -> np.ndarray:
        gripper2cam = np.asarray(
            np.load(str(Path(gripper2cam_path).expanduser().resolve())),
            dtype=np.float64,
        )
        if gripper2cam.shape == (3, 4):
            gripper2cam = np.vstack(
                (gripper2cam, np.array([[0.0, 0.0, 0.0, 1.0]]))
            )
        if gripper2cam.shape != (4, 4):
            raise ValueError(
                f"Transform must be 4x4 or 3x4: {gripper2cam.shape}"
            )
        if not np.all(np.isfinite(gripper2cam)):
            raise ValueError("Transform contains NaN or Inf")
        if abs(float(gripper2cam[3, 3])) < 1.0e-12:
            raise ValueError("Invalid homogeneous transform")

        gripper2cam = gripper2cam / float(gripper2cam[3, 3])
        unit = str(self.transform_translation_unit).strip().lower()
        translation_norm = float(np.linalg.norm(gripper2cam[:3, 3]))
        if unit == "auto":
            unit = "m" if translation_norm < 2.0 else "mm"
        if unit == "m":
            gripper2cam[:3, 3] *= 1000.0
        elif unit != "mm":
            raise ValueError(
                "transform_translation_unit must be auto, m, or mm"
            )
        return gripper2cam

    @staticmethod
    def get_robot_pose_matrix(
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
    ) -> np.ndarray:
        """Create T_base_gripper from a Doosan-style Z-Y'-Z'' pose [mm, deg]."""
        a, b, c = np.deg2rad([rx, ry, rz])
        ca, sa = math.cos(a), math.sin(a)
        cb, sb = math.cos(b), math.sin(b)
        cc, sc = math.cos(c), math.sin(c)

        rot_z_a = np.array(
            [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        rot_y_b = np.array(
            [[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]],
            dtype=np.float64,
        )
        rot_z_c = np.array(
            [[cc, -sc, 0.0], [sc, cc, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rot_z_a @ rot_y_b @ rot_z_c
        matrix[:3, 3] = [float(x), float(y), float(z)]
        return matrix

    @staticmethod
    def _rotation_matrix_to_zyz_deg(rotation: np.ndarray) -> np.ndarray:
        """Convert a rotation matrix to intrinsic Z-Y'-Z'' Euler angles."""
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

    def _lookup_robot_pos_mm(
        self,
        stamp: Any,
    ) -> Optional[np.ndarray]:
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
                if now - self.last_tf_warning_time > 2.0:
                    self.get_logger().warning(
                        f"TF unavailable: {self.base_frame} <- "
                        f"{self.calibration_frame}: {error}"
                    )
                    self.last_tf_warning_time = now
                return None

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        rotation = self._quaternion_to_rotation_matrix(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        zyz_deg = self._rotation_matrix_to_zyz_deg(rotation)
        return np.array(
            [
                float(translation.x) * 1000.0,
                float(translation.y) * 1000.0,
                float(translation.z) * 1000.0,
                float(zyz_deg[0]),
                float(zyz_deg[1]),
                float(zyz_deg[2]),
            ],
            dtype=np.float64,
        )

    def transform_to_base(
        self,
        camera_coords: Sequence[float],
        gripper2cam_path: str,
        robot_pos: Sequence[float],
    ) -> np.ndarray:
        """Convert camera coordinates to the robot base frame using the requested formula."""
        gripper2cam = self._load_gripper2cam_mm(gripper2cam_path)
        coord = np.append(np.asarray(camera_coords, dtype=np.float64), 1.0)

        x, y, z, rx, ry, rz = [float(value) for value in robot_pos]
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        if abs(float(td_coord[3])) < 1.0e-12:
            raise ValueError("Invalid transformed homogeneous coordinate")
        return td_coord[:3] / float(td_coord[3])

    # ------------------------------------------------------------------
    # Action
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request: FindOrder.Goal) -> GoalResponse:
        target = str(goal_request.target_name).strip().lower()
        if target not in {"hand", "palm", "손", "손바닥"}:
            self.get_logger().warning(
                f"Rejected /find_hand_order target: {goal_request.target_name}"
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

    def execute_find_hand(self, goal_handle) -> FindOrder.Result:
        self.find_hand_active = True
        self._publish_scan_succeeded(False)
        self._publish_scan_active(True)
        self.get_logger().warning(
            "Hand scan active: SpeedL tracking must remain disabled"
        )

        self.filtered_camera_xyz_mm = None
        self.filtered_base_xyz_mm = None
        self.latest_base_xyz_mm = None
        self.last_stable_point_mm = None
        self.latest_valid_time = 0.0
        self.stable_frame_count = 0
        self.current_feedback_state = "searching"

        start_time = time.monotonic()
        last_feedback = ""

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = FindOrder.Result()
                    result.found = False
                    result.coordinate = [0.0, 0.0, 0.0]
                    result.message = "hand detection canceled"
                    return result

                elapsed = time.monotonic() - start_time
                if elapsed > float(self.find_hand_timeout_sec):
                    goal_handle.abort()
                    result = FindOrder.Result()
                    result.found = False
                    result.coordinate = [0.0, 0.0, 0.0]
                    result.message = "hand not detected"
                    return result

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
                    coordinate = self._round_xyz_mm(
                        self.latest_base_xyz_mm
                    )
                    goal_handle.succeed()
                    result = FindOrder.Result()
                    result.found = True
                    result.coordinate = coordinate
                    result.message = "hand detected"

                    # 성공 좌표가 확정된 경우에만 SpeedL 추적을 arm한다.
                    self._publish_scan_succeeded(True)
                    self.get_logger().info(
                        "Hand detected [mm]: "
                        f"[{coordinate[0]:.2f}, {coordinate[1]:.2f}, "
                        f"{coordinate[2]:.2f}]"
                    )
                    self.get_logger().info(
                        "Published /find_hand_order/succeeded=True"
                    )
                    return result

                # MultiThreadedExecutor의 다른 스레드가 카메라/TF 콜백을 처리한다.
                # asyncio 이벤트 루프를 요구하지 않도록 동기 대기를 사용한다.
                time.sleep(0.05)
        finally:
            self.find_hand_active = False
            self.current_feedback_state = "idle"
            self._publish_scan_active(False)
            self.get_logger().info(
                "Published /find_hand_order/active=False"
            )
            with self._goal_lock:
                self._goal_reserved = False

    # ------------------------------------------------------------------
    # Camera and 3D
    # ------------------------------------------------------------------
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _depth_to_mm(self, depth_msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth_msg.encoding in {"16UC1", "mono16"} or depth.dtype == np.uint16:
            return depth.astype(np.float32)
        return depth.astype(np.float32) * 1000.0

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
            max(0, v_depth - radius):min(depth_height, v_depth + radius + 1),
            max(0, u_depth - radius):min(depth_width, u_depth + radius + 1),
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
    def _quaternion_to_rotation_matrix(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-12:
            raise ValueError("Invalid zero-length quaternion")
        x /= norm
        y /= norm
        z /= norm
        w /= norm
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

    def _camera_to_base_mm(
        self,
        camera_xyz_mm: Sequence[float],
        stamp: Any,
    ) -> Optional[np.ndarray]:
        robot_pos = self._lookup_robot_pos_mm(stamp)
        if robot_pos is None:
            return None

        try:
            base_point_mm = self.transform_to_base(
                camera_xyz_mm,
                self.gripper2cam_path,
                robot_pos,
            )
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Coordinate transform failed: {error}")
            return None

        if not np.all(np.isfinite(base_point_mm)):
            self.get_logger().error("Coordinate transform returned NaN/Inf")
            return None

        base_point_mm = np.asarray(base_point_mm, dtype=np.float64).copy()
        corrected_z_mm = float(base_point_mm[2]) - self.base_z_offset_mm
        if self.clamp_base_z_nonnegative:
            corrected_z_mm = max(0.0, corrected_z_mm)
        base_point_mm[2] = corrected_z_mm
        return base_point_mm

    def _smooth(
        self,
        previous: Optional[np.ndarray],
        current: np.ndarray,
    ) -> np.ndarray:
        if previous is None:
            return current.copy()
        alpha = float(self.smoothing_alpha)
        return alpha * current + (1.0 - alpha) * previous

    def _update_stability(self, base_xyz_mm: np.ndarray) -> None:
        if self.last_stable_point_mm is None:
            self.stable_frame_count = 1
        else:
            jump = float(np.linalg.norm(base_xyz_mm - self.last_stable_point_mm))
            if jump <= float(self.stable_max_jump_mm):
                self.stable_frame_count += 1
            else:
                self.stable_frame_count = 1
        self.last_stable_point_mm = base_xyz_mm.copy()
        self.latest_base_xyz_mm = base_xyz_mm.copy()
        self.latest_valid_time = time.monotonic()
        self.current_feedback_state = (
            "hand detected"
            if self.stable_frame_count == 1
            else "stabilizing coordinate"
        )

    @staticmethod
    def _point_message_mm(
        xyz_mm: Sequence[float],
        frame_id: str,
        stamp: Any,
    ) -> PointStamped:
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.point.x = float(xyz_mm[0])
        msg.point.y = float(xyz_mm[1])
        msg.point.z = float(xyz_mm[2])
        return msg

    def _publish_detected(self, detected: bool) -> None:
        msg = Bool()
        msg.data = detected
        self.detected_pub.publish(msg)

    def _publish_info(
        self,
        color_msg: Image,
        *,
        detected: bool,
        status: str,
        center_uv: Optional[Tuple[int, int]] = None,
        depth_mm: Optional[float] = None,
        camera_xyz_mm: Optional[Sequence[float]] = None,
        base_xyz_mm: Optional[Sequence[float]] = None,
    ) -> None:
        payload = {
            "detected": detected,
            "status": status,
            "coordinate_unit": "mm",
            "center_uv_px": list(center_uv) if center_uv else None,
            "depth_mm": depth_mm,
            "camera_xyz_mm": (
                [float(value) for value in camera_xyz_mm]
                if camera_xyz_mm is not None
                else None
            ),
            "base_frame": self.base_frame,
            "base_xyz_mm": (
                [float(value) for value in base_xyz_mm]
                if base_xyz_mm is not None
                else None
            ),
            "stable_frames": int(self.stable_frame_count),
            "action_active": bool(self.find_hand_active),
        }
        msg = String()
        import json

        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.info_pub.publish(msg)

    def _synced_callback(self, color_msg: Image, depth_msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_process_time < self.min_process_period:
            return
        self.last_process_time = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().error(f"Color conversion failed: {error}")
            return

        if not self.find_hand_active:
            self._publish_detected(False)
            self._publish_info(
                color_msg,
                detected=False,
                status="WAITING_FIND_HAND_ORDER",
            )
            self._show_waiting(bgr)
            return

        intrinsics = self._get_intrinsics()
        if intrinsics is None:
            self.current_feedback_state = "waiting camera info"
            self._publish_detected(False)
            self._publish_info(
                color_msg,
                detected=False,
                status="WAITING_CAMERA_INFO",
            )
            self._show_waiting(bgr, "WAITING CAMERA INFO")
            return

        try:
            depth_mm = self._depth_to_mm(depth_msg)
        except CvBridgeError as error:
            self.get_logger().error(f"Depth conversion failed: {error}")
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        result = self.hands.process(rgb)
        rgb.flags.writeable = True
        annotated = bgr.copy()

        if not result.multi_hand_landmarks:
            self.stable_frame_count = 0
            self.last_stable_point_mm = None
            self.current_feedback_state = "searching"
            self._publish_detected(False)
            self._publish_info(
                color_msg,
                detected=False,
                status="HAND_NOT_DETECTED",
            )
            self._show_waiting(annotated, "HAND NOT DETECTED")
            return

        hand_landmarks = result.multi_hand_landmarks[0]
        self.mp_draw.draw_landmarks(
            annotated,
            hand_landmarks,
            self.mp_hands.HAND_CONNECTIONS,
            self.mp_styles.get_default_hand_landmarks_style(),
            self.mp_styles.get_default_hand_connections_style(),
        )

        image_height, image_width = annotated.shape[:2]
        pixels = []
        for landmark_id in self.PALM_LANDMARK_IDS:
            landmark = hand_landmarks.landmark[landmark_id]
            pixels.append(
                (
                    int(np.clip(round(landmark.x * image_width), 0, image_width - 1)),
                    int(np.clip(round(landmark.y * image_height), 0, image_height - 1)),
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
            self.stable_frame_count = 0
            self.last_stable_point_mm = None
            self.current_feedback_state = "invalid depth"
            self._publish_detected(False)
            self._publish_info(
                color_msg,
                detected=False,
                status="INVALID_DEPTH",
                center_uv=(center_u, center_v),
            )
            self._show_waiting(annotated, "INVALID DEPTH")
            return

        camera_raw_mm = self._deproject_mm(
            center_u,
            center_v,
            depth_value_mm,
            intrinsics,
        )
        base_raw_mm = self._camera_to_base_mm(
            camera_raw_mm,
            color_msg.header.stamp,
        )
        if base_raw_mm is None:
            self.stable_frame_count = 0
            self.last_stable_point_mm = None
            self.current_feedback_state = "waiting tf"
            self._publish_detected(False)
            self._publish_info(
                color_msg,
                detected=False,
                status="TF_UNAVAILABLE",
                center_uv=(center_u, center_v),
                depth_mm=depth_value_mm,
                camera_xyz_mm=camera_raw_mm,
            )
            self._show_waiting(annotated, "BASE TF UNAVAILABLE")
            return

        self.filtered_camera_xyz_mm = self._smooth(
            self.filtered_camera_xyz_mm,
            camera_raw_mm,
        )
        self.filtered_base_xyz_mm = self._smooth(
            self.filtered_base_xyz_mm,
            base_raw_mm,
        )
        self._update_stability(self.filtered_base_xyz_mm)

        self.camera_point_pub.publish(
            self._point_message_mm(
                self.filtered_camera_xyz_mm,
                "camera_color_optical_frame",
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
            color_msg,
            detected=True,
            status="PALM_3D_OK",
            center_uv=(center_u, center_v),
            depth_mm=depth_value_mm,
            camera_xyz_mm=self.filtered_camera_xyz_mm,
            base_xyz_mm=self.filtered_base_xyz_mm,
        )

        cv2.circle(annotated, (center_u, center_v), 9, (0, 0, 255), -1)
        x, y, z = self.filtered_base_xyz_mm
        cv2.putText(
            annotated,
            f"BASE [{x:.1f}, {y:.1f}, {z:.1f}] mm",
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

    def _show_waiting(self, image: np.ndarray, text: str = "WAITING /find_hand_order") -> None:
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
        self.action_server.destroy()
        self.hands.close()
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MediaPipePalm3DActionServer()
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


if __name__ == "__main__":
    main()
