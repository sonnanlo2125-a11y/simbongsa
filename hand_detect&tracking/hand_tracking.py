#!/usr/bin/env python3
"""RealSense + MediaPipe 손바닥 이미지 서보 SpeedL 서비스 노드.

제어 흐름
---------
1. /find_hand_order Action이 수행되는 동안 SpeedL을 강제로 차단한다.
2. Action 성공 신호를 받은 뒤 /arrived_goal(Trigger)을 한 번 허용한다.
3. 손바닥 중심 픽셀을 목표 픽셀로 정렬한다.
4. 중심 오차가 허용 범위 안일 때만 목표 Depth까지 전진한다.
5. 목표 조건을 연속으로 만족하면 정지하고 /hand_arrived=True를 발행한다.

절대 손 좌표와 TCP 좌표를 서로 빼는 방식은 사용하지 않는다. NPY에서는
회전 성분만 사용하며, 속도 변환은 다음과 같다.

  v_base = R_base_link6(TF) @ R_link6_camera(NPY) @ v_camera
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional, Sequence, Tuple

import cv2
import mediapipe as mp
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from dsr_msgs2.msg import SpeedlStream
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformException


PALM_LANDMARK_INDICES = (0, 5, 9, 13, 17)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def quaternion_to_rotation_matrix(
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


class PalmImageServoFollowService(Node):
    """손바닥 영상 오차를 SpeedL 속도 명령으로 변환한다."""

    def __init__(self) -> None:
        super().__init__("realsense_hand_image_servo_follow_service")

        script_dir = Path(__file__).resolve().parent
        self._declare_parameters(script_dir)
        self._read_parameters()
        self._validate_parameters()

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None

        self.filtered_u: Optional[float] = None
        self.filtered_v: Optional[float] = None
        self.filtered_depth_mm: Optional[float] = None
        self.latest_valid_time = 0.0
        self.valid_frame_count = 0
        self.arrival_stable_count = 0
        self.tracking_enabled = False

        # Action scan과 SpeedL 추적을 분리하는 one-shot gate.
        self.hand_scan_active = False
        self.hand_scan_succeeded = False

        self.last_speed_nonzero = False
        self.last_inference_time = 0.0
        self.last_log_time = 0.0
        self.last_tf_warning_time = 0.0

        self.link6_from_camera_rotation = self._load_camera_rotation()

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=int(self.model_complexity),
            min_detection_confidence=float(
                self.min_hand_detection_confidence
            ),
            min_tracking_confidence=float(self.min_tracking_confidence),
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.static_tf_broadcaster = None
        if self.publish_gripper_tcp_static_tf:
            self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            self._publish_gripper_tcp_static_tf()

        self.speedl_publisher = self.create_publisher(
            SpeedlStream, self.speedl_topic, 10
        )

        signal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.tracking_started_publisher = self.create_publisher(
            Bool, "/hand_tracking_request", signal_qos
        )
        self.hand_arrived_publisher = self.create_publisher(
            Bool, "/hand_arrived", signal_qos
        )
        self.start_tracking_service = self.create_service(
            Trigger, "/arrived_goal", self.start_tracking_callback
        )

        scan_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.scan_active_sub = self.create_subscription(
            Bool,
            "/find_hand_order/active",
            self.hand_scan_active_callback,
            scan_state_qos,
        )
        self.scan_succeeded_sub = self.create_subscription(
            Bool,
            "/find_hand_order/succeeded",
            self.hand_scan_succeeded_callback,
            scan_state_qos,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.color_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.color_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber],
            queue_size=int(self.sync_queue_size),
            slop=float(self.sync_slop_sec),
        )
        self.sync.registerCallback(self.synced_image_callback)

        self.control_timer = self.create_timer(
            1.0 / float(self.control_rate_hz), self.control_callback
        )

        self.get_logger().info("Image-based palm servo node ready")
        self.get_logger().info(
            "Start condition: successful /find_hand_order then /arrived_goal"
        )
        self.get_logger().info(
            f"Target: pixel ratios=({self.target_u_ratio:.4f}, "
            f"{self.target_v_ratio:.4f}), depth={self.target_depth_mm:.1f} mm"
        )
        self.get_logger().info(
            f"Speed limits [mm/s]: lateral={self.max_lateral_speed_mm_s:.1f}, "
            f"forward={self.max_forward_speed_mm_s:.1f}, "
            f"total={self.max_total_speed_mm_s:.1f}"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self, script_dir: Path) -> None:
        defaults = {
            "transform_path": str(script_dir / "T_gripper2camera.npy"),
            "npy_rotation_direction": "camera_to_link6",
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "speedl_topic": "/dsr01/speedl_stream",
            "base_frame": "base_link",
            "calibration_frame": "link_6",
            # 다른 노드에서 gripper_tcp TF를 사용한다면 True로 유지한다.
            "publish_gripper_tcp_static_tf": True,
            "static_tf_parent_frame": "link_6",
            "static_tf_child_frame": "gripper_tcp",
            "static_tf_x_m": 0.0,
            "static_tf_y_m": 0.0,
            "static_tf_z_m": 0.250,
            "static_tf_roll_rad": 0.0,
            "static_tf_pitch_rad": 0.0,
            "static_tf_yaw_rad": 0.0,
            "model_complexity": 1,
            "min_hand_detection_confidence": 0.60,
            "min_tracking_confidence": 0.60,
            "max_inference_hz": 15.0,
            "sync_queue_size": 10,
            "sync_slop_sec": 0.04,
            "depth_scale_16u_mm": 1.0,
            "depth_roi_radius": 5,
            "min_valid_depth_mm": 120.0,
            "max_valid_depth_mm": 2000.0,
            "position_filter_alpha": 0.55,
            "max_pixel_jump": 250.0,
            "max_depth_jump_mm": 350.0,
            "hand_timeout_sec": 0.35,
            "stable_frames_before_motion": 3,
            "control_rate_hz": 20.0,
            "kp_lateral": 1.2,
            "center_deadband_px": 3.0,
            "center_tolerance_px": 10.0,
            "approach_center_gate_px": 60.0,
            "target_u_ratio": 0.5,
            "target_v_ratio": 2.0 / 3.0,
            "target_depth_mm": 230.0,
            "kp_depth": 0.8,
            "depth_tolerance_mm": 5.0,
            "max_lateral_speed_mm_s": 250.0,
            "max_forward_speed_mm_s": 500.0,
            "max_total_speed_mm_s": 500.0,
            "linear_acc_mm_s2": 60.0,
            "angular_acc_deg_s2": 100.0,
            "command_time_sec": 0.0,
            "arrival_stable_cycles": 3,
            "require_hand_scan_success_before_tracking": True,
            "camera_x_sign": 1.0,
            "camera_y_sign": 1.0,
            "camera_z_sign": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        names = (
            "transform_path",
            "npy_rotation_direction",
            "color_topic",
            "depth_topic",
            "camera_info_topic",
            "speedl_topic",
            "base_frame",
            "calibration_frame",
            "publish_gripper_tcp_static_tf",
            "static_tf_parent_frame",
            "static_tf_child_frame",
            "static_tf_x_m",
            "static_tf_y_m",
            "static_tf_z_m",
            "static_tf_roll_rad",
            "static_tf_pitch_rad",
            "static_tf_yaw_rad",
            "model_complexity",
            "min_hand_detection_confidence",
            "min_tracking_confidence",
            "max_inference_hz",
            "sync_queue_size",
            "sync_slop_sec",
            "depth_scale_16u_mm",
            "depth_roi_radius",
            "min_valid_depth_mm",
            "max_valid_depth_mm",
            "position_filter_alpha",
            "max_pixel_jump",
            "max_depth_jump_mm",
            "hand_timeout_sec",
            "stable_frames_before_motion",
            "control_rate_hz",
            "kp_lateral",
            "center_deadband_px",
            "center_tolerance_px",
            "approach_center_gate_px",
            "target_u_ratio",
            "target_v_ratio",
            "target_depth_mm",
            "kp_depth",
            "depth_tolerance_mm",
            "max_lateral_speed_mm_s",
            "max_forward_speed_mm_s",
            "max_total_speed_mm_s",
            "linear_acc_mm_s2",
            "angular_acc_deg_s2",
            "command_time_sec",
            "arrival_stable_cycles",
            "require_hand_scan_success_before_tracking",
            "camera_x_sign",
            "camera_y_sign",
            "camera_z_sign",
        )
        for name in names:
            setattr(self, name, self.get_parameter(name).value)

        self.transform_path = str(Path(str(self.transform_path)).expanduser().resolve())
        self.npy_rotation_direction = str(
            self.npy_rotation_direction
        ).strip().lower()
        self.base_frame = str(self.base_frame).strip().lstrip("/")
        self.calibration_frame = str(self.calibration_frame).strip().lstrip("/")
        self.publish_gripper_tcp_static_tf = bool(
            self.publish_gripper_tcp_static_tf
        )

    def _validate_parameters(self) -> None:
        positive_names = (
            "max_inference_hz",
            "sync_queue_size",
            "sync_slop_sec",
            "depth_roi_radius",
            "hand_timeout_sec",
            "stable_frames_before_motion",
            "control_rate_hz",
            "kp_lateral",
            "approach_center_gate_px",
            "target_depth_mm",
            "kp_depth",
            "max_lateral_speed_mm_s",
            "max_forward_speed_mm_s",
            "max_total_speed_mm_s",
            "linear_acc_mm_s2",
            "arrival_stable_cycles",
        )
        for name in positive_names:
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be > 0")

        nonnegative_names = (
            "center_deadband_px",
            "center_tolerance_px",
            "depth_tolerance_mm",
            "angular_acc_deg_s2",
            "command_time_sec",
        )
        for name in nonnegative_names:
            if float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} must be >= 0")

        if not self.base_frame or not self.calibration_frame:
            raise ValueError("base_frame and calibration_frame must not be empty")
        if not 0.0 < float(self.position_filter_alpha) <= 1.0:
            raise ValueError("position_filter_alpha must be in (0, 1]")
        if float(self.min_valid_depth_mm) >= float(self.max_valid_depth_mm):
            raise ValueError("min_valid_depth_mm must be less than max_valid_depth_mm")
        if not (
            float(self.min_valid_depth_mm)
            <= float(self.target_depth_mm)
            <= float(self.max_valid_depth_mm)
        ):
            raise ValueError("target_depth_mm must be inside the valid depth range")
        if not 0.0 <= float(self.target_u_ratio) <= 1.0:
            raise ValueError("target_u_ratio must be in [0, 1]")
        if not 0.0 <= float(self.target_v_ratio) <= 1.0:
            raise ValueError("target_v_ratio must be in [0, 1]")
        if self.npy_rotation_direction not in {
            "camera_to_link6",
            "link6_to_camera",
        }:
            raise ValueError(
                "npy_rotation_direction must be camera_to_link6 or "
                "link6_to_camera"
            )
        for name in ("camera_x_sign", "camera_y_sign", "camera_z_sign"):
            if float(getattr(self, name)) not in {-1.0, 1.0}:
                raise ValueError(f"{name} must be -1.0 or 1.0")

    # ------------------------------------------------------------------
    # Safety gate and service
    # ------------------------------------------------------------------
    @staticmethod
    def _bool_message(value: bool) -> Bool:
        message = Bool()
        message.data = bool(value)
        return message

    def _clear_tracking_measurements(self) -> None:
        self.filtered_u = None
        self.filtered_v = None
        self.filtered_depth_mm = None
        self.latest_valid_time = 0.0
        self.valid_frame_count = 0
        self.arrival_stable_count = 0
        self.last_inference_time = 0.0
        self.last_log_time = 0.0

    def _force_disable_tracking(self) -> None:
        self.tracking_enabled = False
        self._clear_tracking_measurements()
        self._send_zero_speed(force=True)

    def hand_scan_active_callback(self, message: Bool) -> None:
        was_active = self.hand_scan_active
        self.hand_scan_active = bool(message.data)
        if self.hand_scan_active:
            self.hand_scan_succeeded = False
            self._force_disable_tracking()
            if not was_active:
                self.get_logger().warning(
                    "Hand scan started: SpeedL stopped and tracking disarmed"
                )
        elif was_active:
            self.get_logger().info("Hand scan finished")

    def hand_scan_succeeded_callback(self, message: Bool) -> None:
        previous = self.hand_scan_succeeded
        self.hand_scan_succeeded = bool(message.data)
        if self.hand_scan_succeeded and not previous:
            self.get_logger().info(
                "Hand scan succeeded: /arrived_goal armed once"
            )
        elif previous and not self.hand_scan_succeeded:
            self.get_logger().info("Hand scan success gate cleared")

    def _reset_tracking_session(self) -> None:
        self._clear_tracking_measurements()
        self.tracking_enabled = True

    def start_tracking_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.hand_scan_active:
            self._force_disable_tracking()
            response.success = False
            response.message = "hand scan is active; SpeedL tracking is blocked"
            self.get_logger().error(response.message)
            return response

        if (
            bool(self.require_hand_scan_success_before_tracking)
            and not self.hand_scan_succeeded
        ):
            self._force_disable_tracking()
            response.success = False
            response.message = "no successful hand scan; /arrived_goal rejected"
            self.get_logger().error(response.message)
            return response

        if self.tracking_enabled:
            response.success = False
            response.message = "hand tracking is already active"
            return response

        # 성공 gate는 추적 시작 한 번에만 소비한다.
        self.hand_scan_succeeded = False
        self._send_zero_speed(force=True)
        self._reset_tracking_session()
        self.tracking_started_publisher.publish(self._bool_message(True))

        response.success = True
        response.message = "image-based hand tracking started"
        self.get_logger().warning("Hand tracking started by /arrived_goal")
        return response

    # ------------------------------------------------------------------
    # Static TF and calibration rotation
    # ------------------------------------------------------------------
    @staticmethod
    def _rpy_to_quaternion(
        roll: float, pitch: float, yaw: float
    ) -> Tuple[float, float, float, float]:
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _publish_gripper_tcp_static_tf(self) -> None:
        if self.static_tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(
            self.static_tf_parent_frame
        ).strip().lstrip("/")
        transform.child_frame_id = str(
            self.static_tf_child_frame
        ).strip().lstrip("/")
        transform.transform.translation.x = float(self.static_tf_x_m)
        transform.transform.translation.y = float(self.static_tf_y_m)
        transform.transform.translation.z = float(self.static_tf_z_m)

        qx, qy, qz, qw = self._rpy_to_quaternion(
            float(self.static_tf_roll_rad),
            float(self.static_tf_pitch_rad),
            float(self.static_tf_yaw_rad),
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(transform)

    def _load_camera_rotation(self) -> np.ndarray:
        path = Path(self.transform_path)
        if not path.is_file():
            raise FileNotFoundError(f"Transform file not found: {path}")

        matrix = np.asarray(np.load(str(path)), dtype=np.float64)
        if matrix.shape not in {(3, 4), (4, 4)}:
            raise ValueError(f"Transform must be 3x4 or 4x4: {matrix.shape}")
        rotation = matrix[:3, :3]
        if not np.all(np.isfinite(rotation)):
            raise ValueError("Transform rotation contains NaN or Inf")

        # 캘리브레이션 수치 오차를 가장 가까운 정규직교 행렬로 보정한다.
        u, _, vt = np.linalg.svd(rotation)
        rotation = u @ vt
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vt

        if self.npy_rotation_direction == "link6_to_camera":
            rotation = rotation.T

        self.get_logger().info(
            f"Loaded NPY rotation once: {path}; "
            f"direction={self.npy_rotation_direction}"
        )
        return rotation

    def _lookup_base_from_link6_rotation(self) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.calibration_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            now = time.monotonic()
            if now - self.last_tf_warning_time >= 2.0:
                self.get_logger().warning(
                    f"TF missing: {self.base_frame} <- "
                    f"{self.calibration_frame}: {error}"
                )
                self.last_tf_warning_time = now
            return None

        quaternion = transform.transform.rotation
        return quaternion_to_rotation_matrix(
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
        )

    # ------------------------------------------------------------------
    # Camera processing
    # ------------------------------------------------------------------
    def camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info = message

    @staticmethod
    def _palm_pixel(
        landmarks: Sequence, width: int, height: int
    ) -> Tuple[float, float]:
        u = float(
            np.mean(
                [float(landmarks[index].x) for index in PALM_LANDMARK_INDICES]
            )
        ) * float(width)
        v = float(
            np.mean(
                [float(landmarks[index].y) for index in PALM_LANDMARK_INDICES]
            )
        ) * float(height)
        return (
            clamp(u, 0.0, float(width - 1)),
            clamp(v, 0.0, float(height - 1)),
        )

    def _median_depth_mm(
        self,
        depth: np.ndarray,
        encoding: str,
        u: float,
        v: float,
    ) -> Optional[float]:
        height, width = depth.shape[:2]
        center_u = int(round(u))
        center_v = int(round(v))
        radius = int(self.depth_roi_radius)
        roi = depth[
            max(0, center_v - radius) : min(
                height, center_v + radius + 1
            ),
            max(0, center_u - radius) : min(
                width, center_u + radius + 1
            ),
        ]
        if roi.size == 0:
            return None

        values = roi.astype(np.float64)
        if roi.dtype == np.uint16 or str(encoding).lower() in {
            "16uc1",
            "mono16",
        }:
            values *= float(self.depth_scale_16u_mm)
        else:
            values *= 1000.0

        valid = values[
            np.isfinite(values)
            & (values >= float(self.min_valid_depth_mm))
            & (values <= float(self.max_valid_depth_mm))
        ]
        if valid.size < 5:
            return None
        return float(np.median(valid))

    def synced_image_callback(
        self, color_msg: Image, depth_msg: Image
    ) -> None:
        if not self.tracking_enabled or self.hand_scan_active:
            return

        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / float(self.max_inference_hz):
            return
        self.last_inference_time = now

        try:
            color = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding="bgr8"
            )
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except CvBridgeError as error:
            self.valid_frame_count = 0
            self.get_logger().error(f"Image conversion failed: {error}")
            return

        if color is None or depth is None or color.size == 0 or depth.size == 0:
            self.valid_frame_count = 0
            return

        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        detection = self.hands.process(rgb)
        rgb.flags.writeable = True
        if not detection.multi_hand_landmarks:
            self.valid_frame_count = 0
            return

        height, width = color.shape[:2]
        landmarks = detection.multi_hand_landmarks[0].landmark
        palm_u, palm_v = self._palm_pixel(landmarks, width, height)
        depth_mm = self._median_depth_mm(
            np.asarray(depth), depth_msg.encoding, palm_u, palm_v
        )
        if depth_mm is None:
            self.valid_frame_count = 0
            return

        if not self._filter_measurement(palm_u, palm_v, depth_mm, now):
            self.valid_frame_count = 0
            return

        self.latest_valid_time = now
        self.valid_frame_count = min(
            self.valid_frame_count + 1,
            int(self.stable_frames_before_motion),
        )

    def _filter_measurement(
        self,
        new_u: float,
        new_v: float,
        new_depth_mm: float,
        now: float,
    ) -> bool:
        stale = (
            self.filtered_u is None
            or self.filtered_v is None
            or self.filtered_depth_mm is None
            or now - self.latest_valid_time > float(self.hand_timeout_sec)
        )
        if stale:
            self.filtered_u = float(new_u)
            self.filtered_v = float(new_v)
            self.filtered_depth_mm = float(new_depth_mm)
            return True

        pixel_jump = math.hypot(
            float(new_u) - float(self.filtered_u),
            float(new_v) - float(self.filtered_v),
        )
        depth_jump = abs(float(new_depth_mm) - float(self.filtered_depth_mm))
        if pixel_jump > float(self.max_pixel_jump):
            return False
        if depth_jump > float(self.max_depth_jump_mm):
            return False

        alpha = float(self.position_filter_alpha)
        self.filtered_u = alpha * float(new_u) + (1.0 - alpha) * float(
            self.filtered_u
        )
        self.filtered_v = alpha * float(new_v) + (1.0 - alpha) * float(
            self.filtered_v
        )
        self.filtered_depth_mm = alpha * float(new_depth_mm) + (
            1.0 - alpha
        ) * float(self.filtered_depth_mm)
        return True

    # ------------------------------------------------------------------
    # SpeedL control
    # ------------------------------------------------------------------
    def control_callback(self) -> None:
        # callback 순서와 관계없이 손 스캔 중 SpeedL을 최종 차단한다.
        if self.hand_scan_active:
            self._send_zero_speed()
            return
        if not self.tracking_enabled:
            self._send_zero_speed()
            return

        now = time.monotonic()
        if (
            self.filtered_u is None
            or self.filtered_v is None
            or self.filtered_depth_mm is None
            or self.camera_info is None
            or now - self.latest_valid_time > float(self.hand_timeout_sec)
            or self.valid_frame_count < int(self.stable_frames_before_motion)
        ):
            self.arrival_stable_count = 0
            self._send_zero_speed()
            return

        info = self.camera_info
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self._send_zero_speed()
            return

        image_width = int(info.width) or max(1, int(round(cx * 2.0)))
        image_height = int(info.height) or max(1, int(round(cy * 2.0)))
        target_u_px = float(image_width - 1) * float(self.target_u_ratio)
        target_v_px = float(image_height - 1) * float(self.target_v_ratio)

        u = float(self.filtered_u)
        v = float(self.filtered_v)
        depth_mm = float(self.filtered_depth_mm)
        error_u_px = u - target_u_px
        error_v_px = v - target_v_px
        center_error_px = math.hypot(error_u_px, error_v_px)

        error_x_camera_mm = error_u_px * depth_mm / fx
        error_y_camera_mm = error_v_px * depth_mm / fy
        depth_error_mm = depth_mm - float(self.target_depth_mm)

        center_ok = (
            abs(error_u_px) <= float(self.center_tolerance_px)
            and abs(error_v_px) <= float(self.center_tolerance_px)
        )
        # 목표보다 가까워져도 후퇴시키지 않고 도착으로 처리한다.
        depth_ok = depth_error_mm <= float(self.depth_tolerance_mm)
        if center_ok and depth_ok:
            self._send_zero_speed()
            self.arrival_stable_count += 1
            if self.arrival_stable_count >= int(self.arrival_stable_cycles):
                self._finish_tracking()
            return

        self.arrival_stable_count = 0
        vx_camera = 0.0
        vy_camera = 0.0
        if abs(error_u_px) > float(self.center_deadband_px):
            vx_camera = clamp(
                float(self.kp_lateral) * error_x_camera_mm,
                -float(self.max_lateral_speed_mm_s),
                float(self.max_lateral_speed_mm_s),
            )
        if abs(error_v_px) > float(self.center_deadband_px):
            vy_camera = clamp(
                float(self.kp_lateral) * error_y_camera_mm,
                -float(self.max_lateral_speed_mm_s),
                float(self.max_lateral_speed_mm_s),
            )

        # 손이 목표 픽셀 근처에 있고 아직 멀 때만 전진한다.
        vz_camera = 0.0
        if (
            center_error_px <= float(self.approach_center_gate_px)
            and depth_error_mm > 0.0
        ):
            vz_camera = clamp(
                float(self.kp_depth) * depth_error_mm,
                0.0,
                float(self.max_forward_speed_mm_s),
            )

        camera_velocity = np.array(
            [
                float(self.camera_x_sign) * vx_camera,
                float(self.camera_y_sign) * vy_camera,
                float(self.camera_z_sign) * vz_camera,
            ],
            dtype=np.float64,
        )
        base_from_link6 = self._lookup_base_from_link6_rotation()
        if base_from_link6 is None:
            self._send_zero_speed()
            return

        base_velocity = (
            base_from_link6
            @ self.link6_from_camera_rotation
            @ camera_velocity
        )
        speed_norm = float(np.linalg.norm(base_velocity))
        if speed_norm > float(self.max_total_speed_mm_s):
            base_velocity *= float(self.max_total_speed_mm_s) / speed_norm

        self._publish_speedl(
            [
                float(base_velocity[0]),
                float(base_velocity[1]),
                float(base_velocity[2]),
                0.0,
                0.0,
                0.0,
            ]
        )
        self.last_speed_nonzero = speed_norm > 1.0e-6

        if now - self.last_log_time >= 0.5:
            self.get_logger().info(
                f"pixel_error=({error_u_px:.1f}, {error_v_px:.1f}) px, "
                f"depth={depth_mm:.1f} mm, "
                f"v_camera={np.array2string(camera_velocity, precision=1)}, "
                f"v_base={np.array2string(base_velocity, precision=1)}"
            )
            self.last_log_time = now

    def _finish_tracking(self) -> None:
        self.tracking_enabled = False
        self.valid_frame_count = 0
        self.arrival_stable_count = 0
        self._send_zero_speed(force=True)
        self.hand_arrived_publisher.publish(self._bool_message(True))
        self.get_logger().warning(
            "Palm handover position reached; SpeedL stopped; "
            "/hand_arrived=True"
        )

    def _publish_speedl(self, velocity: Sequence[float]) -> None:
        if len(velocity) != 6:
            raise ValueError("SpeedL velocity must contain six values")
        values = [float(value) for value in velocity]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Blocked SpeedL command containing NaN/Inf")
            values = [0.0] * 6

        message = SpeedlStream()
        message.vel = values
        message.acc = [
            float(self.linear_acc_mm_s2),
            float(self.angular_acc_deg_s2),
        ]
        message.time = float(self.command_time_sec)
        self.speedl_publisher.publish(message)

    def _send_zero_speed(self, force: bool = False) -> None:
        if not force and not self.last_speed_nonzero:
            return
        self._publish_speedl([0.0] * 6)
        self.last_speed_nonzero = False

    def destroy_node(self) -> bool:
        self.tracking_enabled = False
        self.hand_scan_active = False
        self.hand_scan_succeeded = False
        try:
            for _ in range(3):
                self._send_zero_speed(force=True)
                time.sleep(0.02)
        except Exception:
            pass
        self.hands.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[PalmImageServoFollowService] = None
    try:
        node = PalmImageServoFollowService()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"[ERROR] {error}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
