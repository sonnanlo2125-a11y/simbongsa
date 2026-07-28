#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$LOG_DIR"
touch "$LOG_DIR/bridge.log" "$LOG_DIR/flask.log"

echo "Ctrl+C를 누르면 로그 보기만 종료됩니다. 실행 중인 UI/DB는 유지됩니다."
tail -n 40 -F "$LOG_DIR/bridge.log" "$LOG_DIR/flask.log"
