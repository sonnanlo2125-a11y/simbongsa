#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$PROJECT_DIR/.ui_db.pids"
KEEP_REDIS=false

if [[ "${1:-}" == "--keep-redis" ]]; then
    KEEP_REDIS=true
fi

stop_pid() {
    local name="$1"
    local pid="${2:-}"

    if [[ -z "$pid" ]]; then
        return
    fi

    if kill -0 "$pid" 2>/dev/null; then
        echo "[STOP] $name PID $pid"
        kill "$pid" 2>/dev/null || true

        for _ in {1..20}; do
            if ! kill -0 "$pid" 2>/dev/null; then
                return
            fi
            sleep 0.1
        done

        echo "[WARN] $name 강제 종료"
        kill -9 "$pid" 2>/dev/null || true
    fi
}

if [[ -f "$PID_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PID_FILE" || true
    stop_pid "Bridge" "${BRIDGE_PID:-}"
    stop_pid "Flask" "${FLASK_PID:-}"
    rm -f "$PID_FILE"
else
    echo "[INFO] 저장된 Bridge/Flask PID가 없습니다."
fi

if [[ "$KEEP_REDIS" == false ]]; then
    cd "$PROJECT_DIR"
    if command -v docker >/dev/null 2>&1; then
        echo "[STOP] Redis"
        docker compose down
    fi
fi

echo "[OK] UI/DB 종료 완료"
