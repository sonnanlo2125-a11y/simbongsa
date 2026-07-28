from __future__ import annotations

import hmac
import os
import threading
from functools import wraps
from typing import Any, Callable, Mapping

from dotenv import load_dotenv
from flask import Flask, jsonify, redirect, render_template, request, session, url_for
from redis.exceptions import RedisError

from redis_store import FIXED_CONFIG_VERSION, RedisStore
from runtime_log import clear_runtime_logs, read_runtime_logs

load_dotenv()


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
    raw_value = os.getenv(name, str(default)).strip()
    try:
        value = int(raw_value)
    except ValueError as error:
        raise ValueError(f"{name} must be an integer: {raw_value!r}") from error
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


app = Flask(__name__)
app.config.update(
    SECRET_KEY=os.getenv("FLASK_SECRET_KEY", "development-only-change-me"),
    JSON_AS_ASCII=False,
    MAX_CONTENT_LENGTH=1 * 1024 * 1024,
    SESSION_COOKIE_HTTPONLY=True,
    SESSION_COOKIE_SAMESITE="Lax",
)
store = RedisStore()

USER_UI_STATE_KEY = "assistive_robot:user_ui_state"
_fixed_data_initialized = False
_fixed_data_lock = threading.Lock()


def initialize_fixed_data_once() -> dict[str, Any] | None:
    """Redis 고정 좌표와 스캔 CASE를 프로세스당 한 번만 초기화합니다."""
    global _fixed_data_initialized

    if _fixed_data_initialized:
        return None

    with _fixed_data_lock:
        if _fixed_data_initialized:
            return None
        result = store.initialize_fixed_data()
        app.logger.info("Fixed data initialization: %s", result)
        _fixed_data_initialized = True
        return result


def _query_limit(name: str = "limit", default: int = 500) -> int:
    raw_value = request.args.get(name, str(default))
    try:
        value = int(raw_value)
    except (TypeError, ValueError):
        value = default
    return max(1, min(value, 5000))


def admin_required(view: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(view)
    def wrapped(*args: Any, **kwargs: Any) -> Any:
        if not session.get("admin_authenticated"):
            if request.path.startswith("/api/"):
                return jsonify({"ok": False, "message": "admin login required"}), 401
            return redirect(url_for("admin_login"))
        return view(*args, **kwargs)

    return wrapped


@app.errorhandler(RedisError)
def handle_redis_error(error: RedisError):
    app.logger.exception("Redis error")
    return jsonify({
        "ok": False,
        "message": "Redis 연결 또는 인증에 실패했습니다.",
        "detail": str(error),
    }), 503


@app.before_request
def ensure_fixed_data() -> None:
    if request.endpoint != "static":
        initialize_fixed_data_once()


@app.get("/")
def user_home():
    return render_template("user.html")


@app.route("/admin/login", methods=["GET", "POST"])
def admin_login():
    error = None
    if request.method == "POST":
        expected_user = os.getenv("ADMIN_USERNAME", "admin")
        expected_password = os.getenv("ADMIN_PASSWORD", "admin123")
        submitted_user = request.form.get("username", "")
        submitted_password = request.form.get("password", "")

        if hmac.compare_digest(submitted_user, expected_user) and hmac.compare_digest(
            submitted_password, expected_password
        ):
            session.clear()
            session["admin_authenticated"] = True
            return redirect(url_for("admin_database"))
        error = "아이디 또는 비밀번호를 확인하세요."
    return render_template("login.html", error=error)


@app.get("/admin/logout")
def admin_logout():
    session.clear()
    return redirect(url_for("admin_login"))


@app.get("/admin/database")
@admin_required
def admin_database():
    return render_template("database.html")


@app.get("/api/health")
def api_health():
    connected = store.ping()
    return jsonify({
        "ok": connected,
        "redis": "connected" if connected else "disconnected",
    })


@app.get("/api/user/state")
def api_user_state_get():
    state = store.redis.hgetall(USER_UI_STATE_KEY)
    return jsonify({
        "state": state.get("state", "idle"),
        "message": state.get("message", "도움이 필요하시면 말씀해 주세요."),
        "user_text": state.get("user_text", ""),
        "assistant_text": state.get("assistant_text", ""),
    })


@app.post("/api/user/transcript")
def api_user_transcript():
    data = request.get_json(silent=True) or {}
    if not isinstance(data, Mapping):
        return jsonify({"ok": False, "message": "JSON object is required"}), 400

    state = str(data.get("state", "idle"))
    message = str(data.get("message", ""))
    session_id = str(data.get("session_id", "default"))
    user_text = str(data.get("user_text", "")).strip()
    assistant_text = str(data.get("assistant_text", "")).strip()

    mapping = {"state": state, "message": message}
    if user_text:
        mapping["user_text"] = user_text
        store.append_conversation(
            role="user",
            text=user_text,
            session_id=session_id,
            source="http_transcript",
            state=state,
        )
    if assistant_text:
        mapping["assistant_text"] = assistant_text
        store.append_conversation(
            role="assistant",
            text=assistant_text,
            session_id=session_id,
            source="http_transcript",
            state=state,
        )
    store.redis.hset(USER_UI_STATE_KEY, mapping=mapping)
    return jsonify({"ok": True})


@app.get("/api/admin/objects")
@admin_required
def api_admin_objects():
    return jsonify({"ok": True, "objects": store.list_objects()})


def _extract_freeform_object(
    payload: Mapping[str, Any],
) -> tuple[str, Mapping[str, Any], bool]:
    record_name = (
        payload.get("record_name")
        or payload.get("object_name")
        or payload.get("class_name")
    )
    if record_name is None:
        raise ValueError("record_name is required")

    if "data" in payload:
        object_data = payload.get("data")
        if not isinstance(object_data, Mapping):
            raise ValueError("data must be a JSON object")
    else:
        # 이전 API 형식과의 호환을 위해 나머지 필드를 자유 JSON으로 저장합니다.
        object_data = {
            key: value
            for key, value in payload.items()
            if key not in {"record_name", "object_name", "replace"}
        }

    return str(record_name), object_data, bool(payload.get("replace", True))


@app.post("/api/admin/objects")
@admin_required
def api_admin_objects_create():
    payload = request.get_json(silent=True) or {}
    if not isinstance(payload, Mapping):
        return jsonify({"ok": False, "message": "JSON object is required"}), 400

    try:
        record_name, object_data, replace = _extract_freeform_object(payload)
        item = store.save_object_record(
            record_name=record_name,
            data=object_data,
            replace=replace,
        )
    except (KeyError, TypeError, ValueError) as error:
        return jsonify({"ok": False, "message": str(error)}), 400
    return jsonify({"ok": True, "object": item})


@app.delete("/api/admin/objects/<record_name>")
@admin_required
def api_admin_objects_delete(record_name: str):
    return jsonify({"ok": True, "deleted": store.delete_object(record_name)})


@app.get("/api/admin/fixed-config")
@admin_required
def api_admin_fixed_config():
    return jsonify({
        "ok": True,
        "version": FIXED_CONFIG_VERSION,
        "fixed_points": store.list_fixed_points(),
        "scan_cases": store.list_scan_cases(),
    })


# 이전 프런트엔드와 외부 코드 호환용 API입니다.
@app.get("/api/admin/fixed-points")
@admin_required
def api_admin_fixed_points():
    return jsonify({"ok": True, "fixed_points": store.list_fixed_points()})


@app.get("/api/admin/scan-cases")
@admin_required
def api_admin_scan_cases():
    return jsonify({"ok": True, "scan_cases": store.list_scan_cases()})


@app.get("/api/admin/conversations")
@admin_required
def api_admin_conversations():
    return jsonify({
        "ok": True,
        "conversations": store.list_conversations(_query_limit()),
    })


@app.delete("/api/admin/conversations")
@admin_required
def api_admin_conversations_delete():
    return jsonify({"ok": True, "deleted": store.clear_conversations()})


@app.get("/api/admin/runtime-logs")
@admin_required
def api_admin_runtime_logs():
    return jsonify({
        "ok": True,
        "logs": read_runtime_logs(_query_limit()),
    })


@app.delete("/api/admin/runtime-logs")
@admin_required
def api_admin_runtime_logs_delete():
    return jsonify({"ok": True, "deleted": clear_runtime_logs()})


def main() -> None:
    store.ping()
    initialize_fixed_data_once()

    host = os.getenv("FLASK_HOST", "0.0.0.0").strip() or "0.0.0.0"
    port = _env_int("FLASK_PORT", 5000, 1, 65535)
    debug = _env_bool("FLASK_DEBUG", False)

    app.run(
        host=host,
        port=port,
        debug=debug,
        use_reloader=False,
        threaded=True,
    )


if __name__ == "__main__":
    main()
