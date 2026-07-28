#!/usr/bin/env python3
from __future__ import annotations

import ast
import asyncio
import re
import json
import math
import os
from typing import Any, Mapping, Sequence

import rclpy
from hey_doopal_msg.srv import (
    GetDbData,
    GetFixedPose,
    GetObjectPose,
    GetScanCase,
)
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from redis_store import RedisStore, utc_now
from runtime_log import append_runtime_log


class RedisObjectBridge(Node):
    """ROS 2와 Redis/Flask UI 사이 Bridge 노드.

    기존 기능:
    - 객체 인식 결과를 Redis에 저장
    - 일반 DB 조회: /assistive/get_db_data
    - VLA 상태/대화/런타임 로그 처리

    추가 기능:
    - 로봇 컨트롤 노드가 객체명을 보내면 Redis 객체 레코드를 조회
    - JSON/딕셔너리의 pose[6]를 검증해 그대로 반환
    - test_robot_control2.py 호환 서비스 /get_fixed_pose, /get_scan_case 제공
    - /get_fixed_pose는 고정 pose와 객체 pose[6]를 모두 조회

    pose 계약:
    - 객체 인식 입력: [x, y, z, rx, ry, rz]
    - Redis 저장/서비스 응답: [x, y, z, rx, ry, rz]
    - pose[0:3]: 객체의 base_link 좌표, mm
    - pose[3:6]: 객체 인식 시점의 M0609 link_6 회전, deg
    """

    USER_UI_STATE_KEY = "assistive_robot:user_ui_state"

    # 객체 좌표는 Bridge 내부와 Robot Control 응답에서 항상
    # [x, y, z, rx, ry, rz] 순서를 사용한다.
    OBJECT_SCHEMA_VERSION_KEY = (
        "assistive_robot:object_schema_version"
    )
    OBJECT_SCHEMA_VERSION_PREFIX = "xyz_rpy_fields_v4_round2"
    POSE_DECIMAL_PLACES = 2

    def __init__(self) -> None:
        super().__init__("assistive_robot_redis_bridge")

        self.store = RedisStore()
        self.store.ping()
        fixed_result = self.store.initialize_fixed_data()

        # 현재 객체 인식 노드가 발행하는 pose 배열:
        # [x, y, z, rx, ry, rz]
        #
        # Bridge 내부/Redis/Robot Control 응답도 동일한 순서를 사용한다.
        # 배열을 임의로 재배치하지 않는다.
        #
        # 다른 객체 인식 노드를 연결할 때만 ROS 파라미터로 변경한다.
        self.declare_parameter(
            "object_detection_pose_order",
            "xyz_rpy",
        )
        self.object_detection_pose_order = self._normalize_pose_order(
            self.get_parameter(
                "object_detection_pose_order"
            ).value
        )

        # 직전 Bridge 버전은 실제 입력 [x,y,z,rx,ry,rz]를
        # [rx,ry,rz,x,y,z]로 잘못 가정하고 재배치했다.
        # 그 결과 모든 필드가 뒤섞였고 원본 y 값은 ry=-180 보정으로
        # 복구할 수 없으므로, 새 스키마 최초 실행 시 객체만 삭제한다.
        self._reset_objects_for_pose_schema_change()

        # 정상 스키마의 기존 레코드는 x/y/z/rx/ry/rz 6개 필드로 정리한다.
        self._compact_existing_object_records()

        # Robot Control은 객체 이동 완료 정보를 DB Bridge에 직접 보내지 않는다.
        # 따라서 변경 불가능한 Robot Control의 기존 /rosout 로그와
        # /task_completed 신호를 이용해 현재 이동 작업을 추적한다.
        self.pending_move_targets: list[str] = []
        self.pending_move_goal: str | None = None
        self.pending_move_goal_pose: list[float] | None = None
        self.pending_move_started_at: str | None = None

        # Robot Control의 call_service() 기본 timeout은 10초이며,
        # 서비스/인터페이스/Robot Control을 수정할 수 없으므로
        # 미존재 객체에 대해서만 응답을 timeout 이후로 지연한다.
        # 그러면 Robot Control의 기존 timeout 분기가 Python None을 반환한다.
        self.declare_parameter(
            "missing_object_response_delay_sec",
            10.5,
        )
        self.missing_object_response_delay_sec = float(
            self.get_parameter(
                "missing_object_response_delay_sec"
            ).value
        )
        if self.missing_object_response_delay_sec <= 10.0:
            self.get_logger().warning(
                "missing_object_response_delay_sec should be > 10.0 "
                "to trigger Robot Control's existing timeout"
            )

        self.chat_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.create_subscription(
            String,
            "/assistive/object_detection",
            self.object_detection_callback,
            10,
        )
        self.create_subscription(
            String,
            "/assistive/object_moved",
            self.object_moved_callback,
            10,
        )
        self.create_subscription(
            String,
            "/assistive/vla_state",
            self.vla_state_callback,
            10,
        )
        self.create_subscription(
            String,
            "/ui_chat_log",
            self.ui_chat_log_callback,
            self.chat_qos,
        )
        self.create_subscription(
            Bool,
            "/hand_tracking_request",
            self.hand_tracking_started_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/hand_arrived",
            self.hand_arrived_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/task_completed",
            self.task_completed_callback,
            10,
        )
        self.create_subscription(
            String,
            "/assistive/system_log",
            self.system_log_callback,
            50,
        )

        filter_text = os.getenv(
            "ROSOUT_LOG_FILTER",
            "m0609,dsr,robot_control,hand,object_detection,redis_bridge,vla",
        )
        self.rosout_filters = {
            item.strip().lower()
            for item in filter_text.split(",")
            if item.strip()
        }
        self.rosout_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1000,
        )
        self.create_subscription(
            Log,
            "/rosout",
            self.rosout_callback,
            self.rosout_qos,
        )

        # 기존 범용 DB 조회 서비스
        self.get_db_data_service = self.create_service(
            GetDbData,
            "/assistive/get_db_data",
            self.get_db_data_callback,
        )

        # 객체 pose 전용 서비스:
        # [object_x, object_y, object_z, robot_rx, robot_ry, robot_rz]
        self.get_object_pose_service = self.create_service(
            GetObjectPose,
            "/assistive/get_object_pose",
            self.get_object_pose_callback,
        )

        # test_robot_control2.py가 사용하는 DB 서비스 이름.
        # /get_fixed_pose는 고정 waypoint와 객체 pose[6]를 모두 반환한다.
        self.get_fixed_pose_service = self.create_service(
            GetFixedPose,
            "/get_fixed_pose",
            self.get_fixed_pose_callback,
        )
        self.get_scan_case_service = self.create_service(
            GetScanCase,
            "/get_scan_case",
            self.get_scan_case_callback,
        )

        # 기존 UI/다른 노드와의 하위 호환용 별칭.
        self.get_fixed_pose_compat_service = self.create_service(
            GetFixedPose,
            "/assistive/get_fixed_pose",
            self.get_fixed_pose_callback,
        )
        self.get_scan_case_compat_service = self.create_service(
            GetScanCase,
            "/assistive/get_scan_case",
            self.get_scan_case_callback,
        )

        self.get_logger().info(f"fixed data: {fixed_result}")
        self.get_logger().info(
            "Object detection input pose order: "
            f"{self.object_detection_pose_order} "
            "([x,y,z,rx,ry,rz] expected)"
        )
        self.get_logger().info(
            "Object Redis and service pose order: "
            "x, y, z, rx, ry, rz"
        )
        self.get_logger().info(
            "Pose precision: Redis storage and ROS2 service responses "
            f"rounded to {self.POSE_DECIMAL_PLACES} decimal places"
        )
        self.get_logger().info(
            "Object move DB sync enabled: "
            "robot command/place logs + /task_completed"
        )
        self.get_logger().info(
            "Missing object behavior: delay /get_fixed_pose response "
            f"for {self.missing_object_response_delay_sec:.1f}s so the "
            "unchanged Robot Control returns None via its timeout branch"
        )
        self.get_logger().info(
            "Redis bridge started: "
            "query=/assistive/get_db_data, "
            "object_pose=/assistive/get_object_pose, "
            "fixed_pose=/get_fixed_pose, "
            "scan_case=/get_scan_case"
        )
        self._runtime_log(
            source="redis_bridge",
            level="INFO",
            category="startup",
            message="Redis Bridge started",
            details={
                "fixed_data": fixed_result,
                "query_service": "/assistive/get_db_data",
                "object_pose_service": "/assistive/get_object_pose",
                "fixed_pose_service": "/get_fixed_pose",
                "scan_case_service": "/get_scan_case",
                "fixed_pose_compat_service": "/assistive/get_fixed_pose",
                "scan_case_compat_service": "/assistive/get_scan_case",
            },
        )

    # ------------------------------------------------------------------
    # Common helpers
    # ------------------------------------------------------------------
    def _runtime_log(
        self,
        *,
        source: str,
        level: str,
        message: str,
        category: str = "system",
        details: Mapping[str, Any] | None = None,
    ) -> None:
        try:
            append_runtime_log(
                source=source,
                level=level,
                message=message,
                category=category,
                details=details,
            )
        except OSError as error:
            self.get_logger().warning(
                f"Runtime log file write failed: {error}",
                throttle_duration_sec=5.0,
            )

    def _parse(
        self,
        message: String,
        *,
        source: str,
    ) -> dict[str, Any] | None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"Invalid JSON from {source}: {error}")
            self._runtime_log(
                source=source,
                level="ERROR",
                category="communication",
                message="Invalid JSON received",
                details={"error": str(error)},
            )
            return None

        if not isinstance(payload, dict):
            self.get_logger().error(
                f"JSON payload from {source} must be an object"
            )
            return None
        return payload

    @staticmethod
    def _record_name(payload: Mapping[str, Any]) -> str:
        value = (
            payload.get("record_name")
            or payload.get("object_name")
            or payload.get("class_name")
        )
        if value is None:
            raise ValueError(
                "record_name, object_name, or class_name is required"
            )
        return str(value)

    @staticmethod
    def _record_data(payload: Mapping[str, Any]) -> Mapping[str, Any]:
        if "data" in payload:
            data = payload.get("data")
            if not isinstance(data, Mapping):
                raise ValueError("data must be a JSON object")
            return data

        return {
            key: value
            for key, value in payload.items()
            if key not in {"record_name", "object_name", "replace"}
        }

    @staticmethod
    def _as_mapping(value: Any) -> Mapping[str, Any]:
        """Redis 조회 결과가 JSON 문자열이어도 dict로 변환한다."""
        if isinstance(value, Mapping):
            return value

        if isinstance(value, str):
            try:
                decoded = json.loads(value)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"DB JSON 해석 실패: {error}"
                ) from error

            if not isinstance(decoded, Mapping):
                raise ValueError("DB 객체 레코드는 JSON object여야 합니다")
            return decoded

        raise ValueError(
            f"지원하지 않는 DB 레코드 타입: {type(value).__name__}"
        )

    @classmethod
    def _extract_object_pose(
        cls,
        record_value: Any,
    ) -> tuple[list[float], str]:
        """객체 레코드를 항상 [x,y,z,rx,ry,rz] 순서로 반환한다.

        지원 형식:
        - {"pose": [x,y,z,rx,ry,rz]}
        - {"x":..., "y":..., "z":...,
           "rx":..., "ry":..., "rz":...}

        Redis Hash의 필드 표시 순서는 의미가 없으며, 각 필드 이름을
        기준으로 읽은 뒤 Robot Control에는 고정 순서로 반환한다.
        """
        return cls._extract_pose6(record_value)


    @staticmethod
    def _is_sequence(value: Any) -> bool:
        return (
            isinstance(value, Sequence)
            and not isinstance(
                value,
                (str, bytes, bytearray),
            )
        )

    @staticmethod
    def _finite_float_list(
        values: Sequence[Any],
        *,
        expected_length: int,
        field_name: str,
    ) -> list[float]:
        if len(values) < expected_length:
            raise ValueError(
                f"{field_name}에는 {expected_length}개 값이 필요합니다: "
                f"{values}"
            )

        try:
            result = [
                float(values[index])
                for index in range(expected_length)
            ]
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{field_name} 값을 float로 변환할 수 없습니다: {values}"
            ) from error

        if not all(math.isfinite(value) for value in result):
            raise ValueError(
                f"{field_name}에 NaN 또는 Inf가 포함되어 있습니다"
            )

        return result

    @staticmethod
    def _mapping_xyz(
        mapping: Mapping[str, Any],
    ) -> list[Any] | None:
        key_sets = (
            ("x", "y", "z"),
            ("x_mm", "y_mm", "z_mm"),
        )
        for keys in key_sets:
            if all(key in mapping for key in keys):
                return [mapping[key] for key in keys]
        return None

    @staticmethod
    def _mapping_orientation(
        mapping: Mapping[str, Any],
    ) -> list[Any] | None:
        key_sets = (
            ("rx", "ry", "rz"),
            ("a", "b", "c"),
            ("roll", "pitch", "yaw"),
        )
        for keys in key_sets:
            if all(key in mapping for key in keys):
                return [mapping[key] for key in keys]
        return None

    @classmethod
    def _extract_pose6(
        cls,
        record_value: Any,
    ) -> tuple[list[float], str]:
        """고정 웨이포인트를 Doosan posx 6D 리스트로 변환한다.

        반환:
            [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg], frame_id

        허용 형식:
        - {"pose": [x,y,z,rx,ry,rz]}
        - {"position": [x,y,z,rx,ry,rz]}
        - {"x":..., "y":..., "z":..., "rx":..., "ry":..., "rz":...}
        - {
            "position": {"x":..., "y":..., "z":...},
            "orientation": {"rx":..., "ry":..., "rz":...}
          }

        웨이포인트는 두산 posx 데이터이므로 단위 필드가 없으면
        위치 mm, 각도 deg로 간주한다.
        """
        record = cls._as_mapping(record_value)
        raw_data = record.get("data", record)
        data = cls._as_mapping(raw_data)

        linear_unit = str(
            data.get("coordinate_unit")
            or data.get("position_unit")
            or data.get("linear_unit")
            or data.get("unit")
            or record.get("coordinate_unit")
            or record.get("position_unit")
            or record.get("linear_unit")
            or record.get("unit")
            or "mm"
        ).strip().lower()

        angle_unit = str(
            data.get("angle_unit")
            or data.get("orientation_unit")
            or record.get("angle_unit")
            or record.get("orientation_unit")
            or "deg"
        ).strip().lower()

        pose_values: Sequence[Any] | None = None

        # 1) 한 배열에 6개 값이 저장된 형식
        for key in (
            "pose",
            "posx",
            "coordinates",
            "coordinate",
            "values",
        ):
            candidate = data.get(key)
            if cls._is_sequence(candidate) and len(candidate) >= 6:
                pose_values = candidate
                break

        # position 자체가 6개 배열인 형식
        if pose_values is None:
            candidate = data.get("position")
            if cls._is_sequence(candidate) and len(candidate) >= 6:
                pose_values = candidate

        # 2) x/y/z/rx/ry/rz가 직접 저장된 형식
        if pose_values is None:
            xyz = cls._mapping_xyz(data)
            orientation = cls._mapping_orientation(data)
            if xyz is not None and orientation is not None:
                pose_values = xyz + orientation

        # 3) position/orientation이 각각 dict인 형식
        if pose_values is None:
            position = data.get("position")
            orientation_data = (
                data.get("orientation")
                or data.get("rotation")
                or data.get("rpy")
            )

            if isinstance(position, Mapping):
                xyz = cls._mapping_xyz(position)
            else:
                xyz = None

            if isinstance(orientation_data, Mapping):
                orientation = cls._mapping_orientation(
                    orientation_data
                )
            elif (
                cls._is_sequence(orientation_data)
                and len(orientation_data) >= 3
            ):
                orientation = list(orientation_data[:3])
            else:
                orientation = None

            if xyz is not None and orientation is not None:
                pose_values = xyz + orientation

        if pose_values is None:
            raise ValueError(
                "웨이포인트에 6D pose가 없습니다. "
                "pose=[x,y,z,rx,ry,rz] 또는 "
                "x/y/z/rx/ry/rz 형식으로 저장하세요"
            )

        pose = cls._finite_float_list(
            pose_values,
            expected_length=6,
            field_name="pose",
        )

        if linear_unit in {
            "m",
            "meter",
            "meters",
        }:
            pose[0:3] = [
                value * 1000.0
                for value in pose[0:3]
            ]
        elif linear_unit not in {
            "mm",
            "millimeter",
            "millimeters",
        }:
            raise ValueError(
                f"지원하지 않는 위치 단위입니다: {linear_unit}"
            )

        if angle_unit in {
            "rad",
            "radian",
            "radians",
        }:
            pose[3:6] = [
                math.degrees(value)
                for value in pose[3:6]
            ]
        elif angle_unit not in {
            "deg",
            "degree",
            "degrees",
        }:
            raise ValueError(
                f"지원하지 않는 각도 단위입니다: {angle_unit}"
            )

        frame_id = str(
            data.get("frame_id")
            or record.get("frame_id")
            or "base_link"
        ).strip()

        pose = cls._round_pose6(pose)
        return pose, frame_id or "base_link"

    @classmethod
    def _waypoint_reference(
        cls,
        value: Any,
        *,
        default_name: str,
    ) -> tuple[str, Any | None]:
        """CASE waypoint에서 이름과 inline pose 여부를 추출한다.

        허용 예:
        - "SCAN_WAYPOINT1"
        - {"pose": "SCAN_WAYPOINT1"}
        - {"waypoint_name": "SCAN_WAYPOINT1", "pose": [x,y,z,rx,ry,rz]}
        - {"pose_name": "SCAN_WAYPOINT1", "x": ..., "rz": ...}
        """
        if isinstance(value, str):
            name = value.strip()
            if not name:
                raise ValueError("CASE waypoint 이름이 비어 있습니다")
            return name, None

        if not isinstance(value, Mapping):
            raise ValueError(
                "CASE waypoint는 pose 이름 문자열 또는 JSON object여야 합니다"
            )

        resolved_name = default_name
        for key in (
            "pose_name",
            "waypoint_name",
            "fixed_point",
            "name",
        ):
            candidate = value.get(key)
            if isinstance(candidate, str) and candidate.strip():
                resolved_name = candidate.strip()
                break

        pose_field = value.get("pose")
        if isinstance(pose_field, str) and pose_field.strip():
            return pose_field.strip(), None

        # 이름과 실제 pose[6]가 같이 있으면 이름을 보존하고 inline pose를 쓴다.
        has_inline_pose = (
            cls._is_sequence(pose_field)
            or cls._is_sequence(value.get("position"))
            or all(key in value for key in ("x", "y", "z", "rx", "ry", "rz"))
        )
        return resolved_name, value if has_inline_pose else None

    def _extract_case_poses(
        self,
        case_record: Any,
    ) -> tuple[list[str], list[list[float]], str]:
        """CASE의 1~3개 waypoint를 DB 순서대로 이름+6D pose로 변환한다."""
        record = self._as_mapping(case_record)
        raw_data = record.get("data", record)
        data = self._as_mapping(raw_data)

        waypoints = (
            data.get("waypoints")
            or data.get("poses")
            or data.get("sequence")
            or data.get("route")
        )

        # pose1/pose2/pose3 또는 first/second/third 형식도 허용한다.
        if not self._is_sequence(waypoints):
            ordered_candidates = [
                data.get("first_pose")
                or data.get("pose1")
                or data.get("start_pose"),
                data.get("second_pose")
                or data.get("pose2"),
                data.get("third_pose")
                or data.get("pose3")
                or data.get("end_pose"),
            ]
            waypoints = [
                candidate
                for candidate in ordered_candidates
                if candidate is not None
            ]

        if not self._is_sequence(waypoints):
            raise ValueError("CASE에 waypoints 배열이 없습니다")

        waypoint_count = len(waypoints)
        if not 1 <= waypoint_count <= 3:
            raise ValueError(
                "CASE waypoint는 1~3개여야 합니다. "
                f"DB CASE에는 {waypoint_count}개가 들어 있습니다"
            )

        resolved_names: list[str] = []
        resolved_poses: list[list[float]] = []
        frame_ids: list[str] = []

        for index, waypoint in enumerate(waypoints, start=1):
            pose_name, inline_record = self._waypoint_reference(
                waypoint,
                default_name=f"waypoint_{index}",
            )

            if inline_record is None:
                fixed_record = self.store.get_fixed_point(pose_name)
                if fixed_record is None:
                    raise ValueError(
                        "CASE가 참조한 웨이포인트를 찾을 수 없습니다: "
                        f"{pose_name}"
                    )
                pose, frame_id = self._extract_pose6(fixed_record)
            else:
                pose, frame_id = self._extract_pose6(inline_record)

            resolved_names.append(pose_name)
            resolved_poses.append(pose)
            frame_ids.append(frame_id)

        if len(set(frame_ids)) != 1:
            raise ValueError(
                "CASE의 waypoint frame_id가 서로 다릅니다: "
                + ", ".join(frame_ids)
            )

        return resolved_names, resolved_poses, frame_ids[0]

    # ------------------------------------------------------------------
    # 고정 웨이포인트 6D pose 조회 서비스
    # ------------------------------------------------------------------
    async def get_fixed_pose_callback(
        self,
        request: GetFixedPose.Request,
        response: GetFixedPose.Response,
    ) -> GetFixedPose.Response:
        """고정 pose 또는 객체 pose[6]를 GetFixedPose 형식으로 반환한다.

        test_robot_control2.py는 HAND_SCAN/pos1 등의 고정 pose와
        airpods/drink 등의 객체를 모두 /get_fixed_pose로 요청한다.
        """
        pose_name = request.pose_name.strip()

        response.success = False
        response.pose = [0.0] * 6
        response.coordinate_unit = "mm"
        response.angle_unit = "deg"
        response.frame_id = "base_link"
        response.json_data = ""

        if not pose_name:
            response.message = "pose_name이 비어 있습니다"
            return response

        try:
            record = self.store.get_fixed_point(pose_name)
            record_kind = "fixed_pose"

            if record is not None:
                pose, frame_id = self._extract_pose6(record)
                success_message = "웨이포인트 조회 성공"
            else:
                record = self.store.get_object_record(pose_name)
                record_kind = "object_pose"
                if record is None:
                    response.success = False
                    response.message = (
                        f"고정 pose 또는 객체를 찾을 수 없습니다: {pose_name}"
                    )

                    self.get_logger().warning(
                        f"GetFixedPose not found: {pose_name}. "
                        "No usable response will be delivered before "
                        "Robot Control timeout; Robot Control will return None."
                    )

                    # GetFixedPose.srv의 pose는 float64[6] 고정 배열이므로
                    # ROS2 응답 필드에 Python None을 직접 넣을 수 없다.
                    # Robot Control은 서비스 응답이 10초 안에 없으면
                    # 기존 코드에서 None을 반환하므로, 그 동작을 사용한다.
                    await asyncio.sleep(
                        self.missing_object_response_delay_sec
                    )
                    return response
                pose, frame_id = self._extract_object_pose(record)
                success_message = "객체 pose 조회 성공"

            response.success = True
            response.pose = self._round_pose6(pose)
            response.frame_id = frame_id
            response.json_data = json.dumps(
                record,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = success_message

            # 명령 로그에서 기억한 destination의 실제 응답 pose를 보관한다.
            # Redis 고정 pose 이름의 대소문자/구성 차이가 있더라도
            # Robot Control에 전달된 정확한 좌표를 객체 새 위치로 사용한다.
            normalized_pose_name = self._normalize_move_goal(pose_name)
            if (
                record_kind == "fixed_pose"
                and self.pending_move_goal in {"pos1", "pos2", "pos3"}
                and normalized_pose_name == self.pending_move_goal
            ):
                self.pending_move_goal_pose = self._round_pose6(
                    response.pose
                )

            self.get_logger().info(
                f"GetFixedPose success: {pose_name} ({record_kind}) -> "
                f"{list(response.pose)}"
            )
        except (KeyError, TypeError, ValueError) as error:
            response.message = str(error)
            self.get_logger().warning(
                f"GetFixedPose failed ({pose_name}): {error}"
            )
        except Exception as error:
            response.message = f"pose 조회 중 오류가 발생했습니다: {error}"
            self.get_logger().error(response.message)

        return response

    # ------------------------------------------------------------------
    # Table scan object reset
    # ------------------------------------------------------------------
    def _clear_object_records_for_scan(self, case_name: str) -> int:
        """새 테이블 스캔 시작 시 이전 객체 레코드만 삭제한다.

        삭제 대상:
        - assistive_robot:object:*
        - assistive_robot:objects:index

        유지 대상:
        - 고정 waypoint 및 HAND_SCAN
        - CASE 정의
        - VLA 대화 기록
        - UI 상태와 런타임 로그
        """
        object_keys = sorted(
            set(
                self.store.redis.scan_iter(
                    match="assistive_robot:object:*",
                )
            )
        )
        object_index_key = "assistive_robot:objects:index"

        with self.store.redis.pipeline(transaction=True) as pipe:
            if object_keys:
                pipe.delete(*object_keys)
            pipe.delete(object_index_key)
            pipe.execute()

        deleted_count = len(object_keys)
        log_message = (
            f"New table scan detected by GetScanCase({case_name}): "
            f"cleared {deleted_count} previous object records"
        )
        self.get_logger().info(log_message)
        self._runtime_log(
            source="redis_bridge",
            level="INFO",
            category="database",
            message=log_message,
            details={
                "case_name": case_name,
                "deleted_count": deleted_count,
                "deleted_keys": object_keys,
                "trigger": "/get_scan_case",
            },
        )
        return deleted_count

    # ------------------------------------------------------------------
    # CASE 순서형 1~3개 6D pose 조회 서비스
    # ------------------------------------------------------------------
    def get_scan_case_callback(
        self,
        request: GetScanCase.Request,
        response: GetScanCase.Response,
    ) -> GetScanCase.Response:
        """CASE의 1~3개 waypoint를 pose_name_1..3, pose_1..3으로 반환한다."""
        case_name = request.case_name.strip()

        response.success = False
        response.pose_name_1 = ""
        response.pose_1 = [0.0] * 6
        response.pose_name_2 = ""
        response.pose_2 = [0.0] * 6
        response.pose_name_3 = ""
        response.pose_3 = [0.0] * 6
        response.coordinate_unit = "mm"
        response.angle_unit = "deg"
        response.frame_id = "base_link"
        response.json_data = ""

        if not case_name:
            response.message = "case_name이 비어 있습니다"
            return response

        try:
            case_record = self.store.get_scan_case(case_name)
            if case_record is None:
                response.message = f"스캔 CASE를 찾을 수 없습니다: {case_name}"
                return response

            names, poses, frame_id = self._extract_case_poses(case_record)

            # Robot Control은 테이블 스캔 시작 직전에 /get_scan_case를 호출한다.
            # 다른 노드를 수정하지 않고 이 기존 요청을 스캔 시작 신호로 사용한다.
            # 유효한 CASE가 확인된 뒤 이전 객체 데이터만 한 번 초기화한다.
            deleted_count = self._clear_object_records_for_scan(case_name)

            # 서비스 인터페이스는 고정 3칸이므로 실제 CASE 개수만 채운다.
            for index, (pose_name, pose) in enumerate(
                zip(names, poses),
                start=1,
            ):
                setattr(response, f"pose_name_{index}", pose_name)
                setattr(
                    response,
                    f"pose_{index}",
                    self._round_pose6(pose),
                )

            response.success = True
            response.frame_id = frame_id

            resolved_case = {
                "case_name": case_name,
                "waypoint_count": len(names),
                "waypoints": [
                    {
                        "order": index,
                        "waypoint_name": pose_name,
                        "pose": self._round_pose6(pose),
                    }
                    for index, (pose_name, pose) in enumerate(
                        zip(names, poses),
                        start=1,
                    )
                ],
                "coordinate_unit": "mm",
                "angle_unit": "deg",
                "frame_id": frame_id,
            }
            response.json_data = json.dumps(
                resolved_case,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = (
                f"CASE 조회 성공: {len(names)}개 waypoint, "
                f"기존 객체 {deleted_count}개 초기화"
            )

            route_text = " -> ".join(
                f"{pose_name} {pose}"
                for pose_name, pose in zip(names, poses)
            )
            self.get_logger().info(
                f"Scan case query success: {case_name} "
                f"({len(names)} waypoints, "
                f"cleared_objects={deleted_count}) -> {route_text}"
            )
        except (KeyError, TypeError, ValueError) as error:
            response.message = str(error)
            self.get_logger().warning(
                f"Scan case query failed ({case_name}): {error}"
            )
        except Exception as error:
            response.message = f"CASE 조회 중 오류가 발생했습니다: {error}"
            self.get_logger().error(response.message)

        return response

    # ------------------------------------------------------------------
    # 객체 6D pose 전용 조회 서비스
    # ------------------------------------------------------------------
    def get_object_pose_callback(
        self,
        request: GetObjectPose.Request,
        response: GetObjectPose.Response,
    ) -> GetObjectPose.Response:
        object_name = request.object_name.strip()

        response.success = False
        response.has_pose = False
        response.pose = [0.0] * 6
        response.coordinate_unit = "mm"
        response.angle_unit = "deg"
        response.frame_id = "base_link"
        response.json_data = ""

        if not object_name:
            response.message = "object_name이 비어 있습니다"
            return response

        try:
            record = self.store.get_object_record(object_name)
            if record is None:
                response.message = f"객체를 찾을 수 없습니다: {object_name}"
                return response

            pose, frame_id = self._extract_object_pose(record)

            response.success = True
            response.has_pose = True
            response.pose = self._round_pose6(pose)
            response.coordinate_unit = "mm"
            response.angle_unit = "deg"
            response.frame_id = frame_id
            response.json_data = json.dumps(
                record,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = "객체 pose 조회 성공"

            self.get_logger().info(
                f"Object pose query success: {object_name} -> "
                f"{list(response.pose)}, frame={frame_id}"
            )
            self._runtime_log(
                source="redis_bridge",
                level="INFO",
                category="communication",
                message=f"Object pose query success: {object_name}",
                details={
                    "pose": list(response.pose),
                    "coordinate_unit": "mm",
                    "angle_unit": "deg",
                    "frame_id": frame_id,
                },
            )
        except (KeyError, TypeError, ValueError) as error:
            response.message = str(error)
            self.get_logger().warning(
                f"Object pose query failed ({object_name}): {error}"
            )
            self._runtime_log(
                source="redis_bridge",
                level="WARN",
                category="communication",
                message=f"Object pose query failed: {object_name}",
                details={"error": str(error)},
            )
        except Exception as error:
            response.message = f"객체 pose 조회 중 오류가 발생했습니다: {error}"
            self.get_logger().error(response.message)
            self._runtime_log(
                source="redis_bridge",
                level="ERROR",
                category="communication",
                message=response.message,
            )

        return response

    # ------------------------------------------------------------------
    # 기존 Redis 범용 조회 서비스
    # ------------------------------------------------------------------
    def get_db_data_callback(
        self,
        request: GetDbData.Request,
        response: GetDbData.Response,
    ) -> GetDbData.Response:
        data_type = request.data_type.strip().lower()
        name = request.name.strip()

        aliases = {
            "object_list": "objects",
            "list_objects": "objects",
            "fixed": "fixed_point",
            "waypoint": "fixed_point",
            "waypoints": "fixed_points",
            "list_fixed_points": "fixed_points",
            "case": "scan_case",
            "cases": "scan_cases",
            "list_scan_cases": "scan_cases",
            "conversation": "conversations",
        }
        data_type = aliases.get(data_type, data_type)

        try:
            if data_type == "object":
                if not name:
                    raise ValueError("object 조회에는 name이 필요합니다")
                result = self.store.get_object_record(name)
                if result is None:
                    return self._not_found(
                        response,
                        f"객체를 찾을 수 없습니다: {name}",
                    )
            elif data_type == "objects":
                result = self.store.list_objects()
            elif data_type == "fixed_point":
                if not name:
                    raise ValueError(
                        "fixed_point 조회에는 name이 필요합니다"
                    )
                result = self.store.get_fixed_point(name)
                if result is None:
                    return self._not_found(
                        response,
                        f"고정 좌표를 찾을 수 없습니다: {name}",
                    )
            elif data_type == "fixed_points":
                result = self.store.list_fixed_points()
            elif data_type == "scan_case":
                if not name:
                    raise ValueError(
                        "scan_case 조회에는 name이 필요합니다"
                    )
                result = self.store.get_scan_case(name)
                if result is None:
                    return self._not_found(
                        response,
                        f"스캔 CASE를 찾을 수 없습니다: {name}",
                    )
            elif data_type == "scan_cases":
                result = self.store.list_scan_cases()
            elif data_type == "conversations":
                limit = 100
                if name:
                    try:
                        limit = int(name)
                    except ValueError as error:
                        raise ValueError(
                            "conversations의 name에는 조회 개수만 "
                            "입력할 수 있습니다"
                        ) from error
                result = self.store.list_conversations(limit=limit)
            else:
                raise ValueError(
                    "지원하지 않는 data_type입니다. 사용 가능: "
                    "object, objects, fixed_point, fixed_points, "
                    "scan_case, scan_cases, conversations"
                )

            response.success = True
            response.json_data = json.dumps(
                result,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = "조회 성공"

            label = f"{data_type}:{name}" if name else data_type
            self.get_logger().info(f"DB query success: {label}")
            self._runtime_log(
                source="redis_bridge",
                level="INFO",
                category="communication",
                message=f"DB query success: {label}",
            )
        except (KeyError, TypeError, ValueError) as error:
            response.success = False
            response.json_data = ""
            response.message = str(error)
            self.get_logger().warning(f"DB query failed: {error}")
            self._runtime_log(
                source="redis_bridge",
                level="WARN",
                category="communication",
                message=f"DB query failed: {error}",
            )
        except Exception as error:
            response.success = False
            response.json_data = ""
            response.message = (
                f"DB 조회 중 오류가 발생했습니다: {error}"
            )
            self.get_logger().error(response.message)
            self._runtime_log(
                source="redis_bridge",
                level="ERROR",
                category="communication",
                message=response.message,
            )

        return response

    @staticmethod
    def _not_found(
        response: GetDbData.Response,
        message: str,
    ) -> GetDbData.Response:
        response.success = False
        response.json_data = ""
        response.message = message
        return response

    # ------------------------------------------------------------------
    # Robot Control move-state inference
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_move_goal(value: Any) -> str:
        goal = str(value or "").strip()
        lowered = goal.lower()

        if lowered in {"", "hand", "user_hand", "hand_scan"}:
            return "hand"
        if lowered in {"pos1", "pos2", "pos3"}:
            return lowered
        return goal

    def _clear_pending_move(self) -> None:
        self.pending_move_targets = []
        self.pending_move_goal = None
        self.pending_move_goal_pose = None
        self.pending_move_started_at = None

    def _set_pending_move(
        self,
        *,
        targets: Sequence[Any],
        goal: Any,
    ) -> None:
        normalized_targets = [
            str(target).strip()
            for target in targets
            if str(target).strip()
        ]
        normalized_goal = self._normalize_move_goal(goal)

        if normalized_goal == "table_scan":
            self._clear_pending_move()
            return

        self.pending_move_targets = normalized_targets
        self.pending_move_goal = normalized_goal
        self.pending_move_goal_pose = None
        self.pending_move_started_at = utc_now()

        self.get_logger().info(
            "Pending object move captured: "
            f"targets={self.pending_move_targets}, "
            f"goal={self.pending_move_goal}"
        )
        self._runtime_log(
            source="redis_bridge",
            level="INFO",
            category="object_move",
            message="Pending object move captured",
            details={
                "targets": list(self.pending_move_targets),
                "goal": self.pending_move_goal,
                "started_at": self.pending_move_started_at,
            },
        )

    def _current_pending_target(self) -> str | None:
        if not self.pending_move_targets:
            return None
        return self.pending_move_targets[0]

    def _pop_pending_target(self) -> str | None:
        if not self.pending_move_targets:
            return None

        target = self.pending_move_targets.pop(0)
        if not self.pending_move_targets:
            self._clear_pending_move()
        return target

    def _resolve_destination_pose(
        self,
        destination: str,
    ) -> list[float]:
        normalized = self._normalize_move_goal(destination)
        if normalized not in {"pos1", "pos2", "pos3"}:
            raise ValueError(
                f"좌표로 갱신할 수 없는 destination입니다: {destination}"
            )

        if (
            self.pending_move_goal == normalized
            and self.pending_move_goal_pose is not None
        ):
            return self._round_pose6(self.pending_move_goal_pose)

        fixed_record = self.store.get_fixed_point(normalized)
        if fixed_record is None:
            raise KeyError(
                f"destination 고정 pose를 찾을 수 없습니다: {normalized}"
            )

        pose, _frame_id = self._extract_pose6(fixed_record)
        return self._round_pose6(pose)

    @staticmethod
    def _normalize_pose_order(value: Any) -> str:
        normalized = str(value or "").strip().lower()
        aliases = {
            "rpy_xyz": "rpy_xyz",
            "rx_ry_rz_x_y_z": "rpy_xyz",
            "rotation_position": "rpy_xyz",
            "xyz_rpy": "xyz_rpy",
            "x_y_z_rx_ry_rz": "xyz_rpy",
            "position_rotation": "xyz_rpy",
        }
        try:
            return aliases[normalized]
        except KeyError as error:
            raise ValueError(
                "object_detection_pose_order는 "
                "rpy_xyz 또는 xyz_rpy여야 합니다: "
                f"{value!r}"
            ) from error

    def _canonical_detection_pose(
        self,
        pose_value: Sequence[Any],
    ) -> list[float]:
        """객체 인식 pose를 [x,y,z,rx,ry,rz]로 정규화한다."""
        raw_pose = self._finite_float_list(
            pose_value,
            expected_length=6,
            field_name="object_detection pose",
        )

        if self.object_detection_pose_order == "rpy_xyz":
            rx, ry, rz, x, y, z = raw_pose
            return [x, y, z, rx, ry, rz]

        # 이미 [x,y,z,rx,ry,rz] 순서인 경우
        return raw_pose

    @classmethod
    def _canonical_detection_pose_from_data(
        cls,
        data: Mapping[str, Any],
        *,
        pose_order: str,
    ) -> tuple[list[float], list[float], str]:
        """객체 인식 payload를 [x,y,z,rx,ry,rz]로 변환한다.

        우선순위:
        1. x/y/z/rx/ry/rz 개별 필드
        2. pose[6] 배열

        반환:
            canonical_pose, received_pose, detected_format
        """
        required_fields = ("x", "y", "z", "rx", "ry", "rz")
        if all(field in data for field in required_fields):
            received = cls._finite_float_list(
                [data[field] for field in required_fields],
                expected_length=6,
                field_name="object_detection named pose",
            )
            return list(received), list(received), "named_xyz_rpy"

        pose_value = data.get("pose")
        if not cls._is_sequence(pose_value):
            raise ValueError(
                "object_detection에는 pose[6] 또는 "
                "x/y/z/rx/ry/rz 필드가 필요합니다"
            )
        if len(pose_value) != 6:
            raise ValueError(
                "object_detection pose는 정확히 6개 값이어야 합니다: "
                f"{pose_value}"
            )

        received = cls._finite_float_list(
            pose_value,
            expected_length=6,
            field_name="object_detection pose",
        )

        if pose_order == "rpy_xyz":
            rx, ry, rz, x, y, z = received
            canonical = [x, y, z, rx, ry, rz]
        else:
            canonical = list(received)

        return canonical, received, f"pose_array_{pose_order}"

    @staticmethod
    def _validate_detected_object_pose(
        pose: Sequence[float],
    ) -> None:
        """필드 뒤섞임과 비정상 좌표를 저장 전에 차단한다."""
        x, y, z, rx, ry, rz = [float(value) for value in pose]

        # M0609/Doosan ZYZ 각도는 일반적으로 -180~180 deg 범위다.
        # 위치값이 회전 필드에 잘못 배정된 경우를 즉시 검출한다.
        for name, value in (
            ("rx", rx),
            ("ry", ry),
            ("rz", rz),
        ):
            if not -180.001 <= value <= 180.001:
                raise ValueError(
                    f"{name}={value:.2f} deg는 허용 범위를 벗어났습니다. "
                    "객체 pose 배열 순서가 뒤섞였는지 확인하세요"
                )

        # 단위 오류(m/mm 혼용)나 인덱스 오류를 조기에 검출하는
        # 넓은 안전 범위다. 실제 작업반경보다 충분히 크게 설정했다.
        for name, value in (
            ("x", x),
            ("y", y),
            ("z", z),
        ):
            if not -2500.0 <= value <= 2500.0:
                raise ValueError(
                    f"{name}={value:.2f} mm는 정상적인 로봇 작업 좌표 "
                    "범위를 크게 벗어났습니다"
                )

    @classmethod
    def _round_pose6(
        cls,
        pose: Sequence[Any],
    ) -> list[float]:
        """6D pose를 검증하고 모든 값을 소수점 둘째 자리로 반올림한다."""
        values = cls._finite_float_list(
            pose,
            expected_length=6,
            field_name="pose",
        )
        return [
            round(value, cls.POSE_DECIMAL_PLACES)
            for value in values
        ]

    @classmethod
    def _pose6_fields(
        cls,
        pose: Sequence[Any],
    ) -> dict[str, float]:
        values = cls._round_pose6(pose)
        return {
            "x": values[0],
            "y": values[1],
            "z": values[2],
            "rx": values[3],
            "ry": values[4],
            "rz": values[5],
        }

    def _reset_objects_for_pose_schema_change(self) -> None:
        """잘못된 배열 순서로 저장된 기존 객체를 최초 1회 삭제한다.

        직전 코드에서는 실제 입력 [x,y,z,rx,ry,rz]를
        [rx,ry,rz,x,y,z]로 다시 배치했다.

        잘못 저장된 대응:
        - stored x  = source rx
        - stored y  = source ry
        - stored z  = source rz
        - stored rx = source x
        - stored ry = -180
        - stored rz = source z

        source y는 덮어쓰기 과정에서 손실되어 완전 복구할 수 없다.
        """
        desired_version = (
            f"{self.OBJECT_SCHEMA_VERSION_PREFIX}:"
            f"input={self.object_detection_pose_order}"
        )

        current_version = self.store.redis.get(
            self.OBJECT_SCHEMA_VERSION_KEY
        )
        if isinstance(current_version, bytes):
            current_version = current_version.decode(
                "utf-8",
                errors="replace",
            )
        current_version = (
            str(current_version)
            if current_version is not None
            else ""
        )

        if current_version == desired_version:
            return

        object_keys = sorted(
            set(
                self.store.redis.scan_iter(
                    match="assistive_robot:object:*",
                )
            )
        )
        object_index_key = "assistive_robot:objects:index"

        with self.store.redis.pipeline(transaction=True) as pipe:
            if object_keys:
                pipe.delete(*object_keys)
            pipe.delete(object_index_key)
            pipe.set(
                self.OBJECT_SCHEMA_VERSION_KEY,
                desired_version,
            )
            pipe.execute()

        self.get_logger().warning(
            "Object pose schema changed: "
            f"{current_version or '<none>'} -> {desired_version}. "
            f"Deleted {len(object_keys)} legacy object records; "
            "run a new object scan."
        )
        self._runtime_log(
            source="redis_bridge",
            level="WARN",
            category="database",
            message="Legacy object records cleared after pose-order fix",
            details={
                "old_schema": current_version or None,
                "new_schema": desired_version,
                "deleted_count": len(object_keys),
                "reason": (
                    "legacy rpy_xyz input was interpreted as xyz_rpy"
                ),
            },
        )

    def _compact_existing_object_records(self) -> None:
        """기존 객체 Hash도 x/y/z/rx/ry/rz 6개 필드로 정리한다."""
        compacted = 0
        skipped = 0

        for item in self.store.list_objects():
            record_name = str(item.get("record_name", "")).strip()
            if not record_name:
                skipped += 1
                continue

            try:
                pose, _frame_id = self._extract_pose6(item)
                self.store.save_object_record(
                    record_name=record_name,
                    data=self._pose6_fields(pose),
                    replace=True,
                )
                compacted += 1
            except (KeyError, TypeError, ValueError) as error:
                skipped += 1
                self.get_logger().warning(
                    "Existing object compact skipped: "
                    f"{record_name}: {error}"
                )

        self.get_logger().info(
            "Object record compact migration complete: "
            f"compacted={compacted}, skipped={skipped}"
        )

    def _update_object_to_destination(
        self,
        *,
        record_name: str,
        destination: str,
        pose: Sequence[Any] | None = None,
    ) -> None:
        normalized = self._normalize_move_goal(destination)
        if normalized not in {"pos1", "pos2", "pos3"}:
            raise ValueError(
                f"지원하지 않는 위치 destination입니다: {destination}"
            )

        if pose is None:
            resolved_pose = self._resolve_destination_pose(normalized)
        else:
            resolved_pose = self._finite_float_list(
                pose,
                expected_length=6,
                field_name="moved pose",
            )

        resolved_pose = self._round_pose6(resolved_pose)
        moved_at = utc_now()

        # 객체 Hash에는 위치 6개 필드만 남긴다.
        # replace=True이므로 기존 pose/frame_id/visible/last_moved_*도 삭제된다.
        self.store.save_object_record(
            record_name=record_name,
            data=self._pose6_fields(resolved_pose),
            replace=True,
        )

        log_message = (
            f"Object location updated: {record_name} -> "
            f"{normalized}, pose={resolved_pose}"
        )
        self.get_logger().info(log_message)
        self._runtime_log(
            source="redis_bridge",
            level="INFO",
            category="object_move",
            message=log_message,
            details={
                "record_name": record_name,
                "destination": normalized,
                "pose": resolved_pose,
                "moved_at": moved_at,
            },
        )

    def _delete_handed_object(self, record_name: str) -> None:
        deleted = self.store.delete_object(record_name)
        if not deleted:
            self.get_logger().warning(
                f"Handed object record was already absent: {record_name}"
            )
            return

        log_message = (
            f"Object handed to user; Redis record deleted: {record_name}"
        )
        self.get_logger().info(log_message)
        self._runtime_log(
            source="redis_bridge",
            level="INFO",
            category="object_move",
            message=log_message,
            details={
                "record_name": record_name,
                "destination": "hand",
                "deleted_at": utc_now(),
            },
        )

    def _track_robot_control_rosout(
        self,
        *,
        node_name: str,
        message_text: str,
    ) -> None:
        if "robot_control" not in node_name.lower():
            return

        # Robot Control의 기존 로그:
        # 명령 수신: target=['airpods'], goal=pos1
        if "명령 수신:" in message_text:
            match = re.search(
                r"명령\s*수신:\s*target=(\[.*\]),\s*goal=(.*)$",
                message_text,
            )
            if match is None:
                self.get_logger().warning(
                    f"Robot command log parse failed: {message_text}"
                )
                return

            try:
                parsed_targets = ast.literal_eval(match.group(1))
            except (SyntaxError, ValueError) as error:
                self.get_logger().warning(
                    f"Robot target list parse failed: {error}"
                )
                return

            if not isinstance(parsed_targets, (list, tuple)):
                self.get_logger().warning(
                    "Robot command target is not a list/tuple"
                )
                return

            self._set_pending_move(
                targets=parsed_targets,
                goal=match.group(2),
            )
            return

        # pos1/pos2/pos3 이동은 그리퍼가 열린 뒤 이 로그가 발생한다.
        if (
            message_text.strip() == "물체 내려놓기 완료"
            and self.pending_move_goal in {"pos1", "pos2", "pos3"}
        ):
            target = self._current_pending_target()
            if target is None:
                self.get_logger().warning(
                    "Place completion received without pending target"
                )
                return

            try:
                self._update_object_to_destination(
                    record_name=target,
                    destination=self.pending_move_goal,
                )
            except (KeyError, TypeError, ValueError) as error:
                self.get_logger().error(
                    f"Placed-object DB update failed: {error}"
                )
                return

            self._pop_pending_target()

    def task_completed_callback(self, message: Bool) -> None:
        """손 전달 후 /ungrip이 발행하는 기존 완료 신호를 처리한다."""
        if not message.data:
            return
        if self.pending_move_goal != "hand":
            return

        target = self._current_pending_target()
        if target is None:
            self.get_logger().warning(
                "/task_completed received without pending hand target"
            )
            return

        self._delete_handed_object(target)
        self._pop_pending_target()

    # ------------------------------------------------------------------
    # Redis save callbacks
    # ------------------------------------------------------------------
    def object_detection_callback(self, message: String) -> None:
        payload = self._parse(message, source="object_detection")
        if payload is None:
            return

        try:
            record_name = self._record_name(payload)

            # 객체 인식 노드가 보낸 데이터를 직접 변경하지 않고 복사한다.
            record_data = dict(self._record_data(payload))

            # 현재 객체 인식 노드는 다음 순서로 발행한다.
            # [x, y, z, rx, ry, rz]
            #
            # 개별 필드가 있으면 필드 이름을 우선 사용하고,
            # pose 배열이면 object_detection_pose_order에 따라 해석한다.
            pose, raw_pose, detected_format = (
                self._canonical_detection_pose_from_data(
                    record_data,
                    pose_order=self.object_detection_pose_order,
                )
            )

            # 보정 전에 전체 위치/회전 필드를 검증해 배열 뒤섞임을 차단한다.
            self._validate_detected_object_pose(pose)

            # ry 보정은 정상 순서가 확정된 후 회전 성분에만 적용한다.
            # x/y/z 위치에는 절대 적용하지 않는다.
            measured_ry = pose[4]
            pose[4] = -180.0
            pose = self._round_pose6(pose)

            # 객체 인식 payload에 confidence, frame_id, visible 등이 포함되어도
            # Redis 객체 Hash에는 x/y/z/rx/ry/rz만 저장한다.
            item = self.store.save_object_record(
                record_name=record_name,
                data=self._pose6_fields(pose),
                replace=True,
            )
            field_count = len(item.get("data", {}))
            log_message = (
                f"Object updated: {record_name} ({field_count} fields), "
                f"ry corrected {measured_ry:.2f} -> -180.00"
            )
            self.get_logger().info(log_message)
            self._runtime_log(
                source="object_detection",
                level="INFO",
                category="communication",
                message=log_message,
                details={
                    "record_name": record_name,
                    "input_pose_order": self.object_detection_pose_order,
                    "detected_format": detected_format,
                    "received_pose": raw_pose,
                    "canonical_before_ry_override": [
                        pose[0],
                        pose[1],
                        pose[2],
                        pose[3],
                        measured_ry,
                        pose[5],
                    ],
                    "measured_ry": measured_ry,
                    "corrected_ry": -180.0,
                    "stored_pose_xyz_rpy": pose,
                },
            )
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f"Object payload error: {error}")
            self._runtime_log(
                source="object_detection",
                level="ERROR",
                category="communication",
                message=f"Object payload error: {error}",
            )

    def object_moved_callback(self, message: String) -> None:
        """명시적 /assistive/object_moved 메시지도 동일한 정책으로 처리한다.

        - destination=hand/user_hand: 객체 레코드 삭제
        - destination=pos1/pos2/pos3: 실제 pose 필드 갱신
        - 그 외 data payload: 기존 병합 동작 유지
        """
        payload = self._parse(message, source="object_moved")
        if payload is None:
            return

        try:
            record_name = self._record_name(payload)
            nested_data = payload.get("data")
            data = (
                nested_data
                if isinstance(nested_data, Mapping)
                else payload
            )

            destination = self._normalize_move_goal(
                data.get("destination")
                or data.get("last_moved_to")
                or payload.get("destination")
            )

            if destination == "hand":
                self._delete_handed_object(record_name)
                return

            if destination in {"pos1", "pos2", "pos3"}:
                explicit_pose = data.get("pose")
                if self._is_sequence(explicit_pose) and len(explicit_pose) >= 6:
                    pose = explicit_pose
                else:
                    pose = None

                self._update_object_to_destination(
                    record_name=record_name,
                    destination=destination,
                    pose=pose,
                )
                return

            if isinstance(nested_data, Mapping):
                pose, _frame_id = self._extract_pose6(nested_data)
                self.store.save_object_record(
                    record_name=record_name,
                    data=self._pose6_fields(pose),
                    replace=True,
                )
                log_message = (
                    "Object moved pose replaced with six fields only: "
                    f"{record_name}"
                )
                self.get_logger().info(log_message)
                return

            raise ValueError(
                "object_moved에는 destination=hand/pos1/pos2/pos3 "
                "또는 6D pose가 포함된 data가 필요합니다"
            )

        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f"Moved payload error: {error}")
            self._runtime_log(
                source="robot_control",
                level="ERROR",
                category="communication",
                message=f"Moved payload error: {error}",
            )

    def vla_state_callback(self, message: String) -> None:
        payload = self._parse(message, source="vla_state")
        if payload is None:
            return

        state = (
            str(payload.get("state", "idle")).strip().lower()
            or "idle"
        )
        status_message = str(payload.get("message", ""))

        self.store.redis.hset(
            self.USER_UI_STATE_KEY,
            mapping={
                "state": state,
                "message": status_message,
            },
        )
        self._runtime_log(
            source="vla",
            level="INFO",
            category="state",
            message=f"VLA state changed: {state}",
            details={"message": status_message},
        )

    def ui_chat_log_callback(self, message: String) -> None:
        """USER와 VLA 응답을 Redis 대화 기록에 저장한다.

        허용되는 사용자 발화자:
        - USER, HUMAN, CLIENT, 사용자

        허용되는 VLA 발화자:
        - ASSISTANT, VLA, ROBOT, BOT, AI, DOOPAL, 두팔이

        기존 VLA 노드가 speaker="VLA" 또는 speaker="ROBOT"으로
        발행하더라도 관리자 UI에서는 role="assistant"로 표시한다.
        """
        payload = self._parse(message, source="ui_chat_log")
        if payload is None:
            return

        raw_speaker = (
            payload.get("speaker")
            or payload.get("role")
            or payload.get("sender")
            or ""
        )
        speaker = str(raw_speaker).strip()
        normalized_speaker = speaker.upper()

        raw_text = (
            payload.get("text")
            or payload.get("content")
            or payload.get("response")
            or payload.get("answer")
            or payload.get("assistant_text")
            or payload.get("user_text")
            or payload.get("message")
            or ""
        )
        text = str(raw_text).strip()
        session_id = (
            str(payload.get("session_id", "default")).strip()
            or "default"
        )

        user_speakers = {
            "USER",
            "HUMAN",
            "CLIENT",
            "사용자",
        }
        assistant_speakers = {
            "ASSISTANT",
            "VLA",
            "ROBOT",
            "BOT",
            "AI",
            "DOOPAL",
            "두팔이",
        }

        if normalized_speaker in user_speakers:
            role = "user"
        elif normalized_speaker in assistant_speakers:
            role = "assistant"
        else:
            self.get_logger().warning(
                "ui_chat_log speaker is unsupported: "
                f"{speaker!r}; expected USER or "
                "ASSISTANT/VLA/ROBOT"
            )
            self._runtime_log(
                source="vla",
                level="WARN",
                category="communication",
                message="Unsupported UI chat speaker",
                details={
                    "speaker": speaker,
                    "payload_keys": sorted(payload.keys()),
                },
            )
            return

        if not text:
            self.get_logger().warning(
                f"ui_chat_log text is empty: speaker={speaker!r}"
            )
            return

        self.store.append_conversation(
            role=role,
            text=text,
            session_id=session_id,
            source="ui_chat_log",
            state=str(payload.get("state", "chat")),
            metadata={
                "original_speaker": speaker,
            },
        )

        ui_field = (
            "user_text"
            if role == "user"
            else "assistant_text"
        )
        self.store.redis.hset(
            self.USER_UI_STATE_KEY,
            mapping={ui_field: text},
        )
        self._runtime_log(
            source="vla",
            level="INFO",
            category="communication",
            message=(
                "UI chat received: "
                f"{speaker or normalized_speaker} -> {role}"
            ),
        )

    # ------------------------------------------------------------------
    # Runtime-only log callbacks
    # ------------------------------------------------------------------
    def hand_tracking_started_callback(
        self,
        message: Bool,
    ) -> None:
        if not message.data:
            return
        self._runtime_log(
            source="hand_tracking",
            level="INFO",
            category="communication",
            message="Hand tracking started signal received",
            details={
                "topic": "/hand_tracking_request",
                "data": True,
            },
        )

    def hand_arrived_callback(self, message: Bool) -> None:
        if not message.data:
            return
        self._runtime_log(
            source="hand_tracking",
            level="INFO",
            category="communication",
            message="Hand arrived signal received",
            details={
                "topic": "/hand_arrived",
                "data": True,
            },
        )

    def system_log_callback(self, message: String) -> None:
        payload = self._parse(message, source="system_log")
        if payload is None:
            return

        self._runtime_log(
            source=str(payload.get("source", "external_node")),
            level=str(payload.get("level", "INFO")),
            category=str(
                payload.get("category", "communication")
            ),
            message=str(payload.get("message", "")),
            details=(
                payload.get("details")
                if isinstance(
                    payload.get("details"),
                    Mapping,
                )
                else None
            ),
        )

    def rosout_callback(self, message: Log) -> None:
        node_name = str(message.name)
        message_text = str(message.msg)

        # DB 이동 상태 추적은 ROSOUT_LOG_FILTER와 무관하게 항상 수행한다.
        self._track_robot_control_rosout(
            node_name=node_name,
            message_text=message_text,
        )

        searchable = f"{node_name} {message_text}".lower()

        if self.rosout_filters and not any(
            keyword in searchable
            for keyword in self.rosout_filters
        ):
            return

        level_mapping = {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }
        level = level_mapping.get(
            int(message.level),
            str(message.level),
        )
        timestamp = (
            f"{int(message.stamp.sec)}."
            f"{int(message.stamp.nanosec):09d}"
        )

        try:
            append_runtime_log(
                source=node_name or "rosout",
                level=level,
                category="rosout",
                message=message_text,
                timestamp=timestamp,
                details={
                    "file": str(message.file),
                    "function": str(message.function),
                    "line": int(message.line),
                },
            )
        except OSError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RedisObjectBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
