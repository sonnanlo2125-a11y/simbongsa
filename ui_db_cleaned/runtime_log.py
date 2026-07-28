from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

_LOG_LOCK = threading.Lock()
_DEFAULT_MAX_BYTES = 2 * 1024 * 1024
_DEFAULT_BACKUP_COUNT = 3


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def runtime_log_path() -> Path:
    configured = os.getenv("RUNTIME_LOG_FILE", "").strip()
    if configured:
        path = Path(configured).expanduser()
        if not path.is_absolute():
            path = Path(__file__).resolve().parent / path
        return path.resolve()
    return Path(__file__).resolve().parent / "runtime_logs.jsonl"


def _rotate_if_needed(path: Path) -> None:
    try:
        max_bytes = int(os.getenv("RUNTIME_LOG_MAX_BYTES", str(_DEFAULT_MAX_BYTES)))
    except ValueError:
        max_bytes = _DEFAULT_MAX_BYTES

    try:
        backup_count = int(
            os.getenv("RUNTIME_LOG_BACKUP_COUNT", str(_DEFAULT_BACKUP_COUNT))
        )
    except ValueError:
        backup_count = _DEFAULT_BACKUP_COUNT

    max_bytes = max(64 * 1024, max_bytes)
    backup_count = max(1, backup_count)

    if not path.exists() or path.stat().st_size < max_bytes:
        return

    oldest = path.with_name(f"{path.name}.{backup_count}")
    if oldest.exists():
        oldest.unlink()

    for index in range(backup_count - 1, 0, -1):
        source = path.with_name(f"{path.name}.{index}")
        target = path.with_name(f"{path.name}.{index + 1}")
        if source.exists():
            source.replace(target)

    path.replace(path.with_name(f"{path.name}.1"))


def append_runtime_log(
    *,
    source: str,
    level: str,
    message: str,
    category: str = "system",
    details: Mapping[str, Any] | None = None,
    timestamp: str | None = None,
) -> dict[str, Any]:
    entry: dict[str, Any] = {
        "timestamp": timestamp or utc_now(),
        "source": str(source).strip() or "unknown",
        "level": str(level).strip().upper() or "INFO",
        "category": str(category).strip().lower() or "system",
        "message": str(message),
    }
    if details:
        entry["details"] = dict(details)

    path = runtime_log_path()
    path.parent.mkdir(parents=True, exist_ok=True)

    with _LOG_LOCK:
        _rotate_if_needed(path)
        with path.open("a", encoding="utf-8") as stream:
            stream.write(
                json.dumps(
                    entry,
                    ensure_ascii=False,
                    separators=(",", ":"),
                    default=str,
                )
            )
            stream.write("\n")

    return entry


def read_runtime_logs(limit: int = 500) -> list[dict[str, Any]]:
    limit = max(1, min(int(limit), 5000))
    path = runtime_log_path()
    if not path.exists():
        return []

    with _LOG_LOCK:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError:
            return []

    result: list[dict[str, Any]] = []
    for line in lines[-limit:]:
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            result.append(item)
    return result


def clear_runtime_logs() -> int:
    """현재 로그와 회전된 백업 로그를 모두 삭제합니다."""
    path = runtime_log_path()
    backup_paths = sorted(path.parent.glob(f"{path.name}.*"))
    log_paths = [path, *backup_paths]
    count = 0

    with _LOG_LOCK:
        for log_path in log_paths:
            if not log_path.is_file():
                continue
            try:
                count += len(log_path.read_text(encoding="utf-8").splitlines())
            except OSError:
                pass

            if log_path == path:
                log_path.write_text("", encoding="utf-8")
            else:
                try:
                    log_path.unlink()
                except OSError:
                    pass

    return count
