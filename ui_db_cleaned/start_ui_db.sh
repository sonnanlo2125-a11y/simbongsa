#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
ROS_WS="${ROS_WS:-$HOME/ws_cobot_pjt/ws_dsr}"

PYTHON_BIN="$PROJECT_DIR/.venv/bin/python3"
PID_FILE="$PROJECT_DIR/.ui_db.pids"
LOG_DIR="$PROJECT_DIR/logs"
REDIS_CONTAINER_NAME="${REDIS_CONTAINER_NAME:-assistive-robot-redis}"

read_dotenv_value() {
    local key="$1"
    local value=""

    if [[ -f "$PROJECT_DIR/.env" ]]; then
        value="$(
            awk -F= -v key="$key" '
                $0 !~ /^[[:space:]]*#/ && $1 == key {
                    sub(/^[^=]*=/, "")
                    print
                    exit
                }
            ' "$PROJECT_DIR/.env"
        )"
        value="${value%$'\r'}"
        value="${value#\"}"
        value="${value%\"}"
        value="${value#\'}"
        value="${value%\'}"
    fi

    printf '%s' "$value"
}

FLASK_BIND_HOST="${FLASK_HOST:-$(read_dotenv_value FLASK_HOST)}"
FLASK_BIND_HOST="${FLASK_BIND_HOST:-0.0.0.0}"
FLASK_PORT="${FLASK_PORT:-$(read_dotenv_value FLASK_PORT)}"
FLASK_PORT="${FLASK_PORT:-5000}"
REDIS_PORT_VALUE="${REDIS_PORT:-$(read_dotenv_value REDIS_PORT)}"
REDIS_PORT_VALUE="${REDIS_PORT_VALUE:-6379}"

cd "$PROJECT_DIR"
mkdir -p "$LOG_DIR"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

find_lan_ip() {
    local address=""

    if command -v ip >/dev/null 2>&1; then
        address="$(
            ip route get 1.1.1.1 2>/dev/null |
            awk '{
                for (i = 1; i <= NF; i++) {
                    if ($i == "src") {
                        print $(i + 1)
                        exit
                    }
                }
            }'
        )"
    fi

    if [[ -z "$address" ]] && command -v hostname >/dev/null 2>&1; then
        address="$(hostname -I 2>/dev/null | awk '{print $1}')"
    fi

    printf '%s' "$address"
}

[[ -f "$PROJECT_DIR/.env" ]] \
    || fail ".env 파일이 없습니다: $PROJECT_DIR/.env"
[[ -f "$PROJECT_DIR/app.py" ]] \
    || fail "app.py 파일이 없습니다: $PROJECT_DIR/app.py"
[[ -f "$PROJECT_DIR/redis_store.py" ]] \
    || fail "redis_store.py 파일이 없습니다: $PROJECT_DIR/redis_store.py"
[[ -f "$PROJECT_DIR/docker-compose.yml" \
    || -f "$PROJECT_DIR/compose.yml" \
    || -f "$PROJECT_DIR/compose.yaml" ]] \
    || fail "docker-compose.yml 또는 compose.yml 파일이 없습니다."
[[ -f "$ROS_DISTRO_SETUP" ]] \
    || fail "ROS 2 setup 파일이 없습니다: $ROS_DISTRO_SETUP"

BRIDGE_FILE=""
if [[ -f "$PROJECT_DIR/ros_object_bridge.py" ]]; then
    BRIDGE_FILE="$PROJECT_DIR/ros_object_bridge.py"
elif [[ -f "$PROJECT_DIR/ros_object_bridge_with_query.py" ]]; then
    BRIDGE_FILE="$PROJECT_DIR/ros_object_bridge_with_query.py"
else
    fail "ros_object_bridge.py 또는 ros_object_bridge_with_query.py가 없습니다."
fi

# 이미 실행 중인 UI/DB 프로세스 확인
if [[ -f "$PID_FILE" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$PID_FILE" || true
    set -u

    BRIDGE_ALIVE=false
    FLASK_ALIVE=false

    [[ -n "${BRIDGE_PID:-}" ]] \
        && kill -0 "$BRIDGE_PID" 2>/dev/null \
        && BRIDGE_ALIVE=true

    [[ -n "${FLASK_PID:-}" ]] \
        && kill -0 "$FLASK_PID" 2>/dev/null \
        && FLASK_ALIVE=true

    if [[ "$BRIDGE_ALIVE" == true || "$FLASK_ALIVE" == true ]]; then
        LAN_IP="$(find_lan_ip)"

        echo "[INFO] UI/DB가 이미 실행 중입니다."
        echo "       Bridge PID : ${BRIDGE_PID:-없음}"
        echo "       Flask PID  : ${FLASK_PID:-없음}"
        echo "       로컬 접속  : http://127.0.0.1:${FLASK_PORT}/"

        if [[ -n "$LAN_IP" ]]; then
            echo "       외부 접속  : http://${LAN_IP}:${FLASK_PORT}/"
        else
            echo "       외부 접속  : hostname -I로 이 PC의 IP를 확인하세요."
        fi

        echo "       로그 확인  : ./logs_ui_db.sh"
        exit 0
    fi

    rm -f "$PID_FILE"
fi

# ROS setup 스크립트는 정의되지 않은 환경변수를 참조할 수 있으므로
# source 중에만 nounset을 해제한다.
set +u
# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"

if [[ -f "$ROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_WS/install/setup.bash"
else
    echo "[WARN] 워크스페이스 setup 파일이 없습니다:"
    echo "       $ROS_WS/install/setup.bash"
fi
set -u

command -v docker >/dev/null 2>&1 \
    || fail "docker 명령을 찾을 수 없습니다."

docker compose version >/dev/null 2>&1 \
    || fail "docker compose 플러그인을 사용할 수 없습니다."

# ROS Python 패키지를 사용할 수 있도록 가상환경 생성
if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "[SETUP] Python 가상환경 생성"
    python3 -m venv --system-site-packages "$PROJECT_DIR/.venv"
fi

if [[ -f "$PROJECT_DIR/requirements.txt" ]]; then
    echo "[SETUP] Python 패키지 확인"
    "$PYTHON_BIN" -m pip install -q -r "$PROJECT_DIR/requirements.txt"
fi

check_redis() {
    "$PYTHON_BIN" - "$PROJECT_DIR/.env" <<'PY'
import os
import sys

from dotenv import load_dotenv
import redis

load_dotenv(sys.argv[1], override=True)

host = os.getenv("REDIS_HOST", "127.0.0.1")
port = int(os.getenv("REDIS_PORT", "6379"))
db = int(os.getenv("REDIS_DB", "0"))
username = os.getenv("REDIS_USERNAME", "").strip() or None
password = os.getenv("REDIS_PASSWORD", "") or None
ssl = os.getenv("REDIS_SSL", "false").strip().lower() in {
    "1", "true", "yes", "on"
}

try:
    client = redis.Redis(
        host=host,
        port=port,
        db=db,
        username=username if password else None,
        password=password,
        ssl=ssl,
        socket_connect_timeout=1.0,
        socket_timeout=1.0,
        decode_responses=True,
    )
    print("READY" if client.ping() else "NOT_READY")
except redis.AuthenticationError:
    print("AUTH_ERROR")
except (redis.ConnectionError, redis.TimeoutError, OSError):
    print("UNREACHABLE")
except Exception as exc:
    print(f"ERROR:{type(exc).__name__}:{exc}")
PY
}

show_port_owner() {
    echo "[INFO] ${REDIS_PORT_VALUE} 포트 점유 정보:"

    if command -v ss >/dev/null 2>&1; then
        sudo ss -ltnp "( sport = :${REDIS_PORT_VALUE} )" 2>/dev/null \
            || ss -ltn "( sport = :${REDIS_PORT_VALUE} )" 2>/dev/null \
            || true
    fi

    echo "[INFO] ${REDIS_PORT_VALUE}를 노출 중인 Docker 컨테이너:"
    docker ps --format 'table {{.Names}}\t{{.Ports}}' |
        grep -E "NAMES|${REDIS_PORT_VALUE}" || true
}

REDIS_STATUS="$(check_redis)"

case "$REDIS_STATUS" in
    READY)
        echo "[OK] 기존 Redis 연결 확인 완료"
        ;;

    AUTH_ERROR)
        show_port_owner
        fail "실행 중인 Redis와 .env 인증정보가 일치하지 않습니다."
        ;;

    UNREACHABLE|NOT_READY)
        echo "[START] Redis"

        if docker ps -a --format '{{.Names}}' |
            grep -qx "$REDIS_CONTAINER_NAME"; then

            if docker ps --format '{{.Names}}' |
                grep -qx "$REDIS_CONTAINER_NAME"; then
                echo "[INFO] Redis 컨테이너가 이미 실행 중입니다."
            else
                echo "[START] 기존 Redis 컨테이너 시작"

                if ! docker start "$REDIS_CONTAINER_NAME" >/dev/null; then
                    show_port_owner
                    fail "Redis 컨테이너 시작 실패"
                fi

            fi
        else
            if ! docker compose up -d redis; then
                show_port_owner
                fail "Docker Redis 시작 실패"
            fi

        fi

        for _ in {1..30}; do
            REDIS_STATUS="$(check_redis)"

            [[ "$REDIS_STATUS" == "READY" ]] && break
            [[ "$REDIS_STATUS" == "AUTH_ERROR" ]] && break

            sleep 0.5
        done

        if [[ "$REDIS_STATUS" != "READY" ]]; then
            docker logs --tail 30 "$REDIS_CONTAINER_NAME" \
                2>/dev/null || true
            fail "Redis 준비 실패: $REDIS_STATUS"
        fi

        echo "[OK] Redis 연결 확인 완료"
        ;;

    ERROR:*)
        fail "Redis 확인 중 오류: $REDIS_STATUS"
        ;;

    *)
        fail "알 수 없는 Redis 상태: $REDIS_STATUS"
        ;;
esac

{
    echo
    echo "===== $(date '+%Y-%m-%d %H:%M:%S') Bridge started ====="
} >> "$LOG_DIR/bridge.log"

{
    echo
    echo "===== $(date '+%Y-%m-%d %H:%M:%S') Flask started ====="
} >> "$LOG_DIR/flask.log"

echo "[START] ROS Object Bridge: $(basename "$BRIDGE_FILE")"
nohup "$PYTHON_BIN" -u "$BRIDGE_FILE" \
    >> "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!

echo "[START] Flask UI: ${FLASK_BIND_HOST}:${FLASK_PORT}"

# .env에서 읽은 Flask 바인딩 값을 실행 환경에도 명시한다.
FLASK_HOST="$FLASK_BIND_HOST" \
FLASK_PORT="$FLASK_PORT" \
nohup "$PYTHON_BIN" -u "$PROJECT_DIR/app.py" \
    >> "$LOG_DIR/flask.log" 2>&1 &
FLASK_PID=$!

cat > "$PID_FILE" <<PIDS
BRIDGE_PID=$BRIDGE_PID
FLASK_PID=$FLASK_PID
PIDS

# Flask와 Bridge가 시작될 시간을 준다.
sleep 2

FAILED=false

if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "[ERROR] Bridge가 시작 직후 종료되었습니다."
    tail -n 30 "$LOG_DIR/bridge.log" || true
    FAILED=true
fi

if ! kill -0 "$FLASK_PID" 2>/dev/null; then
    echo "[ERROR] Flask가 시작 직후 종료되었습니다."
    tail -n 30 "$LOG_DIR/flask.log" || true
    FAILED=true
fi

if [[ "$FAILED" == true ]]; then
    "$PROJECT_DIR/stop_ui_db.sh" --keep-redis \
        >/dev/null 2>&1 || true
    exit 1
fi

# 설정한 Flask 포트가 실제로 열렸는지 확인한다.
FLASK_LISTEN_OK=false
if command -v ss >/dev/null 2>&1; then
    for _ in {1..10}; do
        if ss -ltn 2>/dev/null |
            awk '{print $4}' |
            grep -Eq "(:|])${FLASK_PORT}$"; then
            FLASK_LISTEN_OK=true
            break
        fi
        sleep 0.3
    done
else
    FLASK_LISTEN_OK=true
fi

LAN_IP="$(find_lan_ip)"

echo
echo "[OK] UI/DB 실행 완료"
echo "     Redis     : 연결 확인 완료"
echo "     Bridge    : PID $BRIDGE_PID"
echo "     Flask     : PID $FLASK_PID"
echo "     바인딩    : http://${FLASK_BIND_HOST}:${FLASK_PORT}"
echo "     로컬 접속 : http://127.0.0.1:${FLASK_PORT}/"

if [[ -n "$LAN_IP" ]]; then
    echo "     외부 접속 : http://${LAN_IP}:${FLASK_PORT}/"
else
    echo "     외부 접속 : hostname -I로 이 PC의 IP를 확인하세요."
fi

if [[ "$FLASK_LISTEN_OK" != true ]]; then
    echo "[WARN] ${FLASK_PORT}번 포트의 Flask 바인딩을 확인하지 못했습니다."
    echo "       ./logs_ui_db.sh 또는 logs/flask.log를 확인하세요."
fi

echo "     로그      : ./logs_ui_db.sh"
echo "     종료      : ./stop_ui_db.sh"
