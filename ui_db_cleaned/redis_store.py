from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from typing import Any, Mapping

import redis
from dotenv import load_dotenv

load_dotenv()

OBJECT_INDEX_KEY = "assistive_robot:objects:index"
FIXED_POINT_INDEX_KEY = "assistive_robot:fixed_points:index"
SCAN_CASE_INDEX_KEY = "assistive_robot:scan_cases:index"
FIXED_CONFIG_VERSION_KEY = "assistive_robot:fixed_config:version"
FIXED_CONFIG_LOCK_KEY = "assistive_robot:fixed_config:migration_lock"
CONVERSATION_KEY = "assistive_robot:conversation_history"

# 이 값이 바뀌면 initialize_fixed_data()가 이전 고정 데이터만 제거하고
# 아래 웨이포인트/케이스 구성으로 다시 생성합니다.
FIXED_CONFIG_VERSION = "2026-07-27-all-fixed-points-v4"

FIXED_POINTS: dict[str, list[float]] = {
    "HAND_SCAN": [
        445.29, -23.52, 533.56,
        90.00, -90.00, -90.00,
    ],
    "SCAN_WAYPOINT1": [
        434.70, 243.51, 198.79,
        61.00, -179.21, -119.46,
    ],
    "SCAN_WAYPOINT2": [
        434.70, -68.30, 198.79,
        61.02, -179.21, -119.44,
    ],
    "SCAN_WAYPOINT3": [
        434.70, -387.01, 198.79,
        61.01, -179.21, -119.45,
    ],
    "pos1": [
        301.84, -559.17, 94.45,
        88.37, -175.10, -85.13,
    ],
    "pos2": [
        298.36, -407.04, 55.73,
        101.46, -177.09, -77.68,
    ],
    "pos3": [
        508.43, -465.94, -6.38,
        135.86, -179.88, -37.05,
    ],
    "pos4": [
        304.54, -594.92, 130.73,
        91.13, -89.90, -91.88,
    ],
}

# 각 값은 좌표 자체를 복사한 배열이 아니라 고정 웨이포인트 이름의 순서입니다.
# 따라서 로봇은 get_scan_case_poses("CASE_1")로 실제 posx 배열 목록을 얻습니다.
SCAN_CASES: dict[str, list[str]] = {
    # CASE는 좌표를 복사하지 않고 waypoint 이름을 참조한다.
    # 따라서 위 SCAN_WAYPOINT 값을 변경하면 CASE 응답 좌표도 함께 변경된다.
    "CASE_1": [
        "SCAN_WAYPOINT1",
        "SCAN_WAYPOINT2",
    ],
    "CASE_2": [
        "SCAN_WAYPOINT1",
        "SCAN_WAYPOINT2",
        "SCAN_WAYPOINT3",
    ],
}

# 이전 코드에서 생성했던 키입니다. 새 버전으로 최초 실행할 때 제거됩니다.
LEGACY_FIXED_POINT_NAMES = {"HAND_SCAN", "TARGET_SCAN"}
_JSON_PREFIX = "__json_v1__:"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def create_redis_client() -> redis.Redis:
    password = os.getenv("REDIS_PASSWORD", "")
    username = os.getenv("REDIS_USERNAME", "default").strip() or None

    return redis.Redis(
        host=os.getenv("REDIS_HOST", "127.0.0.1"),
        port=int(os.getenv("REDIS_PORT", "6379")),
        db=int(os.getenv("REDIS_DB", "0")),
        username=username if password else None,
        password=password or None,
        ssl=_env_bool("REDIS_SSL", False),
        decode_responses=True,
        socket_connect_timeout=3.0,
        socket_timeout=3.0,
        health_check_interval=30,
    )


def _safe_key_name(value: str, field_name: str) -> str:
    cleaned = str(value).strip()
    if not cleaned:
        raise ValueError(f"{field_name} cannot be empty")
    if len(cleaned) > 128:
        raise ValueError(f"{field_name} is too long")
    if "/" in cleaned or "\\" in cleaned or any(ord(ch) < 32 for ch in cleaned):
        raise ValueError(
            f"{field_name} cannot contain slashes or control characters"
        )
    return cleaned


def _safe_hash_field(value: Any) -> str:
    field = str(value).strip()
    if not field:
        raise ValueError("Object field name cannot be empty")
    if "\x00" in field:
        raise ValueError("Object field name cannot contain a null character")
    if len(field) > 256:
        raise ValueError("Object field name is too long")
    return field


def _encode_value(value: Any) -> str:
    """모든 JSON 타입을 손실 없이 Redis Hash 문자열로 저장합니다."""
    try:
        return _JSON_PREFIX + json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            default=str,
        )
    except (TypeError, ValueError) as error:
        raise ValueError(f"Value is not JSON serializable: {value!r}") from error


def _decode_value(value: str) -> Any:
    if value.startswith(_JSON_PREFIX):
        try:
            return json.loads(value[len(_JSON_PREFIX):])
        except json.JSONDecodeError:
            return value

    # 이전 버전에서 저장한 값도 계속 읽을 수 있도록 유지합니다.
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if value.startswith(("{", "[")):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            pass
    try:
        return float(value) if any(ch in value for ch in ".eE") else int(value)
    except ValueError:
        return value


def _decode_hash(data: Mapping[str, str]) -> dict[str, Any]:
    return {key: _decode_value(value) for key, value in data.items()}


class RedisStore:
    def __init__(self, client: redis.Redis | None = None) -> None:
        self.redis = client or create_redis_client()

    @staticmethod
    def object_key(record_name: str) -> str:
        return f"assistive_robot:object:{_safe_key_name(record_name, 'record_name')}"

    @staticmethod
    def fixed_point_key(point_name: str) -> str:
        return (
            "assistive_robot:fixed_point:"
            f"{_safe_key_name(point_name, 'point_name')}"
        )

    @staticmethod
    def scan_case_key(case_name: str) -> str:
        return (
            "assistive_robot:scan_case:"
            f"{_safe_key_name(case_name, 'case_name')}"
        )

    def ping(self) -> bool:
        return bool(self.redis.ping())

    def initialize_fixed_data(self) -> dict[str, Any]:
        """
        고정 설정 버전이 달라졌을 때만 이전 고정 좌표/케이스를 제거하고
        FIXED_POINTS와 SCAN_CASES로 원자적으로 다시 생성합니다.

        객체 데이터와 대화 기록은 삭제하지 않습니다.
        """
        with self.redis.lock(
            FIXED_CONFIG_LOCK_KEY,
            timeout=15,
            blocking_timeout=8,
        ):
            current_version = self.redis.get(FIXED_CONFIG_VERSION_KEY)
            point_index_complete = set(FIXED_POINTS).issubset(
                self.redis.smembers(FIXED_POINT_INDEX_KEY)
            )
            case_index_complete = set(SCAN_CASES).issubset(
                self.redis.smembers(SCAN_CASE_INDEX_KEY)
            )
            points_complete = all(
                self.redis.exists(self.fixed_point_key(name))
                for name in FIXED_POINTS
            )
            cases_complete = all(
                self.redis.lrange(self.scan_case_key(name), 0, -1) == waypoint_names
                for name, waypoint_names in SCAN_CASES.items()
            )
            if (
                current_version == FIXED_CONFIG_VERSION
                and point_index_complete
                and case_index_complete
                and points_complete
                and cases_complete
            ):
                return {
                    "status": "kept",
                    "version": FIXED_CONFIG_VERSION,
                    "fixed_points": list(FIXED_POINTS),
                    "scan_cases": list(SCAN_CASES),
                }

            old_point_names = set(self.redis.smembers(FIXED_POINT_INDEX_KEY))
            old_case_names = set(self.redis.smembers(SCAN_CASE_INDEX_KEY))

            point_names_to_delete = (
                old_point_names | LEGACY_FIXED_POINT_NAMES | set(FIXED_POINTS)
            )
            case_names_to_delete = old_case_names | set(SCAN_CASES)

            delete_keys = [
                *(self.fixed_point_key(name) for name in point_names_to_delete),
                *(self.scan_case_key(name) for name in case_names_to_delete),
                FIXED_POINT_INDEX_KEY,
                SCAN_CASE_INDEX_KEY,
                FIXED_CONFIG_VERSION_KEY,
            ]

            created_at = utc_now()
            with self.redis.pipeline(transaction=True) as pipe:
                if delete_keys:
                    pipe.delete(*delete_keys)

                for name, pose in FIXED_POINTS.items():
                    if len(pose) != 6:
                        raise ValueError(
                            f"{name} must contain x, y, z, rx, ry, rz"
                        )
                    mapping = {
                        "name": name,
                        "x": pose[0],
                        "y": pose[1],
                        "z": pose[2],
                        "rx": pose[3],
                        "ry": pose[4],
                        "rz": pose[5],
                        "coordinate_type": "posx",
                        "translation_unit": "mm",
                        "rotation_unit": "deg",
                        "readonly": True,
                        "config_version": FIXED_CONFIG_VERSION,
                        "created_at": created_at,
                    }
                    pipe.hset(
                        self.fixed_point_key(name),
                        mapping={key: _encode_value(value) for key, value in mapping.items()},
                    )
                    pipe.sadd(FIXED_POINT_INDEX_KEY, name)

                for case_name, waypoint_names in SCAN_CASES.items():
                    unknown = [name for name in waypoint_names if name not in FIXED_POINTS]
                    if unknown:
                        raise ValueError(
                            f"{case_name} references unknown waypoints: {unknown}"
                        )
                    case_key = self.scan_case_key(case_name)
                    if waypoint_names:
                        pipe.rpush(case_key, *waypoint_names)
                    pipe.sadd(SCAN_CASE_INDEX_KEY, case_name)

                pipe.set(FIXED_CONFIG_VERSION_KEY, FIXED_CONFIG_VERSION)
                pipe.execute()

            return {
                "status": "migrated",
                "previous_version": current_version,
                "version": FIXED_CONFIG_VERSION,
                "fixed_points": list(FIXED_POINTS),
                "scan_cases": list(SCAN_CASES),
            }

    # 이전 코드에서 호출하던 이름과의 호환용 별칭입니다.
    def initialize_fixed_points(self) -> dict[str, Any]:
        return self.initialize_fixed_data()

    def get_fixed_point(self, point_name: str) -> dict[str, Any] | None:
        data = self.redis.hgetall(self.fixed_point_key(point_name))
        return _decode_hash(data) if data else None

    def get_fixed_pose(self, point_name: str) -> list[float]:
        point = self.get_fixed_point(point_name)
        if not point:
            raise KeyError(f"Fixed point not found: {point_name}")
        return [
            float(point[field])
            for field in ("x", "y", "z", "rx", "ry", "rz")
        ]

    def list_fixed_points(self) -> list[dict[str, Any]]:
        names = sorted(self.redis.smembers(FIXED_POINT_INDEX_KEY))
        result: list[dict[str, Any]] = []
        stale_names: list[str] = []
        for name in names:
            point = self.get_fixed_point(name)
            if point:
                result.append(point)
            else:
                stale_names.append(name)
        if stale_names:
            self.redis.srem(FIXED_POINT_INDEX_KEY, *stale_names)
        return result

    def get_scan_case(self, case_name: str) -> dict[str, Any] | None:
        case_name = _safe_key_name(case_name, "case_name")
        key = self.scan_case_key(case_name)
        if not self.redis.exists(key):
            return None

        waypoint_names = self.redis.lrange(key, 0, -1)
        waypoints: list[dict[str, Any]] = []
        for index, point_name in enumerate(waypoint_names, start=1):
            point = self.get_fixed_point(point_name)
            if not point:
                raise KeyError(
                    f"{case_name} references missing fixed point: {point_name}"
                )
            waypoints.append({
                "order": index,
                "name": point_name,
                "pose": [
                    float(point[field])
                    for field in ("x", "y", "z", "rx", "ry", "rz")
                ],
            })

        return {
            "name": case_name,
            "redis_key": key,
            "waypoint_names": waypoint_names,
            "waypoints": waypoints,
            "readonly": True,
        }

    def list_scan_cases(self) -> list[dict[str, Any]]:
        names = sorted(self.redis.smembers(SCAN_CASE_INDEX_KEY))
        result: list[dict[str, Any]] = []
        stale_names: list[str] = []
        for name in names:
            case = self.get_scan_case(name)
            if case:
                result.append(case)
            else:
                stale_names.append(name)
        if stale_names:
            self.redis.srem(SCAN_CASE_INDEX_KEY, *stale_names)
        return result

    def get_scan_case_poses(self, case_name: str) -> list[list[float]]:
        case = self.get_scan_case(case_name)
        if not case:
            raise KeyError(f"Scan case not found: {case_name}")
        return [waypoint["pose"] for waypoint in case["waypoints"]]

    def save_object_record(
        self,
        *,
        record_name: str,
        data: Mapping[str, Any],
        replace: bool = False,
    ) -> dict[str, Any]:
        """
        객체 식별자(record_name) 외에는 어떤 공통 필드도 강제하지 않습니다.
        data의 최상위 JSON 필드가 그대로 Redis Hash 필드가 됩니다.

        replace=False: 기존 필드에 병합
        replace=True: 기존 Hash를 지우고 전달된 데이터로 전체 교체
        """
        record_name = _safe_key_name(record_name, "record_name")
        if not isinstance(data, Mapping):
            raise ValueError("data must be a JSON object")
        if not data:
            raise ValueError("data must contain at least one field")

        encoded: dict[str, str] = {}
        for raw_field, value in data.items():
            field = _safe_hash_field(raw_field)
            encoded[field] = _encode_value(value)

        key = self.object_key(record_name)
        with self.redis.pipeline(transaction=True) as pipe:
            if replace:
                pipe.delete(key)
            pipe.hset(key, mapping=encoded)
            pipe.sadd(OBJECT_INDEX_KEY, record_name)
            pipe.execute()

        return self.get_object_record(record_name) or {
            "record_name": record_name,
            "redis_key": key,
            "data": dict(data),
        }

    def update_object_fields(
        self,
        *,
        record_name: str,
        fields: Mapping[str, Any],
    ) -> dict[str, Any]:
        if not self.redis.exists(self.object_key(record_name)):
            raise KeyError(f"Object record not found: {record_name}")
        return self.save_object_record(
            record_name=record_name,
            data=fields,
            replace=False,
        )

    # 기존 구조를 호출하는 코드가 남아 있어도 동작하도록 유지합니다.
    def save_object(
        self,
        *,
        class_name: str,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        frame_id: str = "base_link",
        confidence: float = 0.0,
        visible: bool = True,
        attributes: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        data: dict[str, Any] = {
            "class_name": class_name,
            "position": {
                "x_mm": float(x_mm),
                "y_mm": float(y_mm),
                "z_mm": float(z_mm),
            },
            "frame_id": frame_id,
            "confidence": float(confidence),
            "visible": bool(visible),
        }
        if attributes:
            data.update(dict(attributes))
        return self.save_object_record(
            record_name=class_name,
            data=data,
            replace=False,
        )

    def update_object_moved(
        self,
        *,
        class_name: str,
        destination: str,
        x_mm: float,
        y_mm: float,
        z_mm: float,
    ) -> dict[str, Any]:
        return self.update_object_fields(
            record_name=class_name,
            fields={
                "last_moved": {
                    "destination": destination,
                    "position": {
                        "x_mm": float(x_mm),
                        "y_mm": float(y_mm),
                        "z_mm": float(z_mm),
                    },
                    "timestamp": utc_now(),
                }
            },
        )

    def get_object_record(self, record_name: str) -> dict[str, Any] | None:
        record_name = _safe_key_name(record_name, "record_name")
        key = self.object_key(record_name)
        data = self.redis.hgetall(key)
        if not data:
            return None
        return {
            "record_name": record_name,
            "redis_key": key,
            "data": _decode_hash(data),
        }

    # 기존 호출명 호환용
    def get_object(self, class_name: str) -> dict[str, Any] | None:
        record = self.get_object_record(class_name)
        return record["data"] if record else None

    def list_objects(self) -> list[dict[str, Any]]:
        record_names = sorted(self.redis.smembers(OBJECT_INDEX_KEY))
        result: list[dict[str, Any]] = []
        stale_names: list[str] = []
        for record_name in record_names:
            item = self.get_object_record(record_name)
            if item:
                result.append(item)
            else:
                stale_names.append(record_name)
        if stale_names:
            self.redis.srem(OBJECT_INDEX_KEY, *stale_names)
        return result

    def delete_object(self, record_name: str) -> bool:
        record_name = _safe_key_name(record_name, "record_name")
        deleted = bool(self.redis.delete(self.object_key(record_name)))
        self.redis.srem(OBJECT_INDEX_KEY, record_name)
        return deleted

    def append_conversation(
        self,
        *,
        role: str,
        text: str,
        session_id: str = "default",
        source: str = "http",
        state: str = "idle",
        metadata: Mapping[str, Any] | None = None,
        max_messages: int = 5000,
    ) -> dict[str, Any]:
        message = {
            "timestamp": utc_now(),
            "session_id": session_id,
            "role": role,
            "text": text,
            "source": source,
            "state": state,
            "metadata": dict(metadata or {}),
        }
        encoded = json.dumps(message, ensure_ascii=False)
        with self.redis.pipeline(transaction=True) as pipe:
            pipe.rpush(CONVERSATION_KEY, encoded)
            pipe.ltrim(CONVERSATION_KEY, -max_messages, -1)
            pipe.execute()
        return message

    def list_conversations(self, limit: int = 500) -> list[dict[str, Any]]:
        limit = max(1, min(int(limit), 5000))
        raw = self.redis.lrange(CONVERSATION_KEY, -limit, -1)
        result: list[dict[str, Any]] = []
        for item in raw:
            try:
                result.append(json.loads(item))
            except json.JSONDecodeError:
                continue
        return result

    def clear_conversations(self) -> int:
        return int(self.redis.delete(CONVERSATION_KEY))
