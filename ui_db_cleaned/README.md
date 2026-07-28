# 두팔이 UI · Redis · ROS 2 Bridge

시각장애인 보조 로봇 **두팔이** 프로젝트의 사용자 UI, 관리자 UI, Redis 저장소, ROS 2 통신 Bridge를 한 폴더에서 실행하는 구성입니다.

## 1. 구성 요소

| 파일/폴더 | 역할 |
|---|---|
| `app.py` | Flask 사용자 UI와 관리자 UI, HTTP API |
| `redis_store.py` | Redis 키, 고정 좌표, 스캔 CASE, 객체 및 대화 기록 관리 |
| `ros_object_bridge.py` | ROS 2 토픽/서비스와 Redis 사이 Bridge |
| `runtime_log.py` | Redis에 저장하지 않는 로컬 JSONL 런타임 로그 |
| `seed_demo.py` | 고정 데이터 초기화 및 선택적 데모 객체 생성 |
| `templates/` | 사용자·로그인·관리자 HTML |
| `static/` | CSS와 JavaScript |
| `docker-compose.yml` | 비밀번호가 설정된 Redis 컨테이너 |
| `start_ui_db.sh` | 가상환경, Redis, ROS Bridge, Flask 일괄 실행 |
| `stop_ui_db.sh` | Flask, Bridge, Redis 종료 |
| `logs_ui_db.sh` | Flask와 Bridge 로그 실시간 확인 |
| `.env` | 현재 프로젝트의 실제 환경 설정 |
| `.env.example` | 새 환경을 만들 때 참고할 설정 예시 |
| `requirements.txt` | pip로 설치하는 Python 패키지 |

## 2. 정리된 내용

- `.venv` 제거: 새 컴퓨터에서 다시 생성합니다.
- `__pycache__`, `.pyc` 제거: Python 실행 시 자동으로 다시 생성됩니다.
- 기존 PID 파일과 누적 실행 로그 제거
- 중복된 간이 README 폴더 제거 후 본 문서로 통합
- `.env`는 요청에 따라 원본 그대로 포함
- Flask 실행 설정을 고정값이 아닌 `.env` 기준으로 통일
- Redis Docker 외부 포트를 `.env`의 `REDIS_PORT`와 연동
- 고정 데이터 초기화가 같은 Flask 프로세스에서 중복 실행되지 않도록 정리
- 관리자 런타임 로그 삭제 시 회전된 백업 로그도 함께 삭제
- 관리자 화면에서 사용자 UI로 돌아가는 링크 제거
- 기존 로봇 제어 코드와 연동되는 호환 ROS 서비스는 유지

> `.env`에는 관리자 비밀번호와 Redis 비밀번호가 포함될 수 있습니다. 이번 전달 ZIP에는 포함되어 있지만, GitHub와 공개 저장소에는 업로드하지 마세요.

## 3. 요구 환경

- Ubuntu 22.04
- Python 3.10
- ROS 2 Humble
- Docker Engine 및 Docker Compose 플러그인
- `python3-venv`
- 프로젝트에서 빌드된 `hey_doopal_msg`
- `rclpy`, `std_msgs`, `rcl_interfaces`

설치 확인:

```bash
python3 --version
ros2 --help
docker --version
docker compose version
```

가상환경 생성에 필요한 패키지가 없다면:

```bash
sudo apt update
sudo apt install -y python3-venv
```

## 4. 다른 컴퓨터로 옮긴 뒤 실행

### 4.1 압축 해제 및 이동

```bash
unzip ui_db_cleaned.zip
cd ui_db_cleaned
```

### 4.2 실행 권한 부여

```bash
chmod +x start_ui_db.sh stop_ui_db.sh logs_ui_db.sh
```

### 4.3 ROS 워크스페이스 확인

기본 Doosan ROS 워크스페이스 경로는 다음입니다.

```text
~/ws_cobot_pjt/ws_dsr
```

`hey_doopal_msg`가 별도 워크스페이스에 있다면 실행 전에 해당 워크스페이스를 먼저 source 하거나, Doosan 워크스페이스의 overlay에 포함되도록 빌드해야 합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source <hey_doopal_msg_워크스페이스>/install/setup.bash

ros2 interface show hey_doopal_msg/srv/GetFixedPose
```

`start_ui_db.sh`는 기본적으로 다음 경로를 자동 source 합니다.

```text
/opt/ros/humble/setup.bash
~/ws_cobot_pjt/ws_dsr/install/setup.bash
```

다른 경로를 사용한다면:

```bash
ROS_WS=~/다른_워크스페이스 ./start_ui_db.sh
```

별도의 ROS 배포판 setup 경로도 지정할 수 있습니다.

```bash
ROS_DISTRO_SETUP=/opt/ros/humble/setup.bash \
ROS_WS=~/cobot_ws \
./start_ui_db.sh
```

### 4.4 일괄 실행

```bash
./start_ui_db.sh
```

최초 실행에서는 스크립트가 다음 작업을 자동 수행합니다.

1. `.venv` 생성 (`--system-site-packages`)
2. `requirements.txt` 설치
3. Redis 연결 확인
4. Redis가 없으면 Docker 컨테이너 시작
5. `ros_object_bridge.py` 실행
6. Flask UI 실행
7. PID와 로그 파일 생성

기본 접속 주소:

```text
사용자 UI: http://127.0.0.1:5000/
관리자 로그인: http://127.0.0.1:5000/admin/login
```

같은 Wi-Fi의 다른 기기에서는 실행 PC의 IP를 사용합니다.

```bash
hostname -I
```

예시:

```text
http://192.168.0.25:5000/
```

외부 접근이 안 되면 방화벽을 확인합니다.

```bash
sudo ufw allow 5000/tcp
sudo ufw status
```

## 5. 종료와 로그

실시간 로그:

```bash
./logs_ui_db.sh
```

전체 종료:

```bash
./stop_ui_db.sh
```

Flask와 ROS Bridge만 종료하고 Redis는 유지:

```bash
./stop_ui_db.sh --keep-redis
```

로그 위치:

```text
logs/flask.log
logs/bridge.log
runtime_logs.jsonl
runtime_logs.jsonl.1
runtime_logs.jsonl.2
...
```

`logs/`와 `runtime_logs.jsonl*`은 실행 중 생성되는 파일이므로 다른 컴퓨터로 전달할 필요가 없습니다.

## 6. `.env` 설정

현재 `.env`는 원본 그대로 포함되어 있습니다. 새 프로젝트용 설정 예시는 `.env.example`을 참고합니다.

### Flask

| 변수 | 설명 |
|---|---|
| `FLASK_SECRET_KEY` | Flask 세션 서명 키 |
| `ADMIN_USERNAME` | 관리자 아이디 |
| `ADMIN_PASSWORD` | 관리자 비밀번호 |
| `FLASK_HOST` | Flask 바인딩 주소. 외부 접속은 `0.0.0.0` |
| `FLASK_PORT` | Flask 포트 |
| `FLASK_DEBUG` | 디버그 모드. 실제 로봇 운용에서는 `false` 권장 |

### Redis

| 변수 | 설명 |
|---|---|
| `REDIS_HOST` | Redis 주소. Docker 로컬 실행은 `127.0.0.1` |
| `REDIS_PORT` | Redis 외부 포트 |
| `REDIS_DB` | Redis DB 번호 |
| `REDIS_USERNAME` | Redis ACL 사용자명 |
| `REDIS_PASSWORD` | Redis 비밀번호 |
| `REDIS_SSL` | TLS Redis 연결 여부 |

### 로그 및 선택 기능

| 변수 | 설명 | 기본값 |
|---|---|---|
| `SEED_DEMO_OBJECTS` | `seed_demo.py` 실행 시 데모 객체 생성 | `false` |
| `RUNTIME_LOG_FILE` | 관리자 런타임 로그 파일 | `runtime_logs.jsonl` |
| `RUNTIME_LOG_MAX_BYTES` | 로그 회전 기준 용량 | `2097152` |
| `RUNTIME_LOG_BACKUP_COUNT` | 로그 백업 개수 | `3` |
| `ROSOUT_LOG_FILTER` | `/rosout`에서 저장할 노드/메시지 키워드 | 코드 기본값 사용 |

`.env`에 선택 변수가 없어도 코드의 기본값으로 동작합니다.

## 7. Python 의존성

직접 설치:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
pip install -r requirements.txt
```

`requirements.txt`에는 pip 패키지만 기록합니다.

```text
Flask
redis
python-dotenv
```

다음 항목은 pip가 아니라 ROS 2와 워크스페이스에서 제공되어야 합니다.

```text
rclpy
std_msgs
rcl_interfaces
hey_doopal_msg
```

따라서 `ModuleNotFoundError: rclpy`가 나오면 `.venv`를 복사하지 말고 다음 순서로 다시 만듭니다.

```bash
rm -rf .venv
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 8. Redis 데이터 구조

주요 키:

| Redis 키 | 형식 | 내용 |
|---|---|---|
| `assistive_robot:object:<name>` | Hash | 인식 객체의 `x,y,z,rx,ry,rz` |
| `assistive_robot:objects:index` | Set | 저장된 객체 이름 목록 |
| `assistive_robot:fixed_point:<name>` | Hash | 고정 posx 좌표 |
| `assistive_robot:fixed_points:index` | Set | 고정 좌표 이름 목록 |
| `assistive_robot:scan_case:<name>` | List | CASE의 웨이포인트 이름 순서 |
| `assistive_robot:scan_cases:index` | Set | CASE 이름 목록 |
| `assistive_robot:conversation_history` | List | 사용자와 VLA 대화 기록 |
| `assistive_robot:user_ui_state` | Hash | 사용자 UI 상태와 최근 텍스트 |

고정 좌표 및 CASE는 `redis_store.py`의 다음 값에서 관리합니다.

```python
FIXED_CONFIG_VERSION
FIXED_POINTS
SCAN_CASES
```

`FIXED_CONFIG_VERSION`을 변경하면 다음 실행 시 고정 좌표와 CASE만 새 버전으로 다시 생성됩니다. 객체와 대화 기록은 유지됩니다.

## 9. ROS 2 인터페이스

### 구독 토픽

| 토픽 | 타입 | 역할 |
|---|---|---|
| `/assistive/object_detection` | `std_msgs/msg/String` | 객체 인식 결과를 Redis에 저장 |
| `/assistive/object_moved` | `std_msgs/msg/String` | 이동 완료 객체의 위치 갱신 또는 손 전달 후 삭제 |
| `/assistive/vla_state` | `std_msgs/msg/String` | 사용자 UI 상태 갱신 |
| `/ui_chat_log` | `std_msgs/msg/String` | USER와 VLA 대화 기록 저장 |
| `/hand_tracking_request` | `std_msgs/msg/Bool` | 손 추적 시작 로그 |
| `/hand_arrived` | `std_msgs/msg/Bool` | 손 도착 로그 |
| `/task_completed` | `std_msgs/msg/Bool` | 현재 객체 이동 작업 완료 처리 |
| `/assistive/system_log` | `std_msgs/msg/String` | 외부 노드의 구조화 로그 |
| `/rosout` | `rcl_interfaces/msg/Log` | 로봇 제어 로그 수집 및 이동 상태 동기화 |

객체 인식 메시지 예시:

```bash
ros2 topic pub --once /assistive/object_detection std_msgs/msg/String \
  "{data: '{\"class_name\":\"airpods\",\"data\":{\"pose\":[467.70,20.40,288.40,113.71,-180.00,-66.90]}}'}"
```

대화 기록 예시:

```bash
ros2 topic pub --once /ui_chat_log std_msgs/msg/String \
  "{data: '{\"speaker\":\"USER\",\"text\":\"에어팟 손에 줘\",\"session_id\":\"default\"}'}"
```

```bash
ros2 topic pub --once /ui_chat_log std_msgs/msg/String \
  "{data: '{\"speaker\":\"VLA\",\"text\":\"에어팟을 가져다드릴게요\",\"session_id\":\"default\"}'}"
```

### 서비스

| 서비스 | 타입 | 요청 필드 |
|---|---|---|
| `/assistive/get_db_data` | `hey_doopal_msg/srv/GetDbData` | `data_type`, `name` |
| `/assistive/get_object_pose` | `hey_doopal_msg/srv/GetObjectPose` | `object_name` |
| `/get_fixed_pose` | `hey_doopal_msg/srv/GetFixedPose` | `pose_name` |
| `/get_scan_case` | `hey_doopal_msg/srv/GetScanCase` | `case_name` |
| `/assistive/get_fixed_pose` | `hey_doopal_msg/srv/GetFixedPose` | 호환 별칭 |
| `/assistive/get_scan_case` | `hey_doopal_msg/srv/GetScanCase` | 호환 별칭 |

고정 좌표 조회:

```bash
ros2 service call /get_fixed_pose \
  hey_doopal_msg/srv/GetFixedPose \
  "{pose_name: 'HAND_SCAN'}"
```

객체 pose 조회:

```bash
ros2 service call /assistive/get_object_pose \
  hey_doopal_msg/srv/GetObjectPose \
  "{object_name: 'airpods'}"
```

스캔 CASE 조회:

```bash
ros2 service call /get_scan_case \
  hey_doopal_msg/srv/GetScanCase \
  "{case_name: 'CASE_2'}"
```

인터페이스 필드가 맞는지 확인하려면:

```bash
ros2 interface show hey_doopal_msg/srv/GetDbData
ros2 interface show hey_doopal_msg/srv/GetObjectPose
ros2 interface show hey_doopal_msg/srv/GetFixedPose
ros2 interface show hey_doopal_msg/srv/GetScanCase
```

## 10. HTTP API

| Method | Endpoint | 인증 | 기능 |
|---|---|---|---|
| GET | `/api/health` | 없음 | Redis 연결 확인 |
| GET | `/api/user/state` | 없음 | 사용자 UI 상태 조회 |
| POST | `/api/user/transcript` | 없음 | 사용자/VLA 텍스트와 상태 저장 |
| GET | `/api/admin/objects` | 관리자 | 객체 목록 |
| POST | `/api/admin/objects` | 관리자 | 객체 저장/수정 |
| DELETE | `/api/admin/objects/<name>` | 관리자 | 객체 삭제 |
| GET | `/api/admin/fixed-config` | 관리자 | 고정 좌표와 CASE 조회 |
| GET | `/api/admin/conversations` | 관리자 | 대화 기록 조회 |
| DELETE | `/api/admin/conversations` | 관리자 | 대화 기록 삭제 |
| GET | `/api/admin/runtime-logs` | 관리자 | 런타임 로그 조회 |
| DELETE | `/api/admin/runtime-logs` | 관리자 | 현재 및 회전 로그 삭제 |

## 11. 선택적 데모 데이터

고정 좌표와 CASE만 초기화:

```bash
source .venv/bin/activate
python3 seed_demo.py
```

데모 객체까지 만들려면 `.env`에서 다음 값을 설정하고 실행합니다.

```text
SEED_DEMO_OBJECTS=true
```

실제 프로젝트에서는 다시 `false`로 돌려놓는 것을 권장합니다.

## 12. 문제 해결

### `Bridge가 시작 직후 종료되었습니다`

```bash
tail -n 100 logs/bridge.log
```

다음 항목을 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 interface show hey_doopal_msg/srv/GetFixedPose
```

`ImportError: cannot import name GetFixedPose`가 나오면 현재 `hey_doopal_msg` 패키지에 필요한 `.srv` 파일이 없거나 빌드 후 source가 되지 않은 상태입니다.

### Redis 인증 오류

```bash
docker ps -a
sudo ss -ltnp '( sport = :6379 )'
docker logs assistive-robot-redis --tail 50
```

실행 중인 Redis 비밀번호와 `.env`의 `REDIS_PASSWORD`가 같아야 합니다.

### 6379 포트 충돌

기존 로컬 Redis가 점유 중인지 확인합니다.

```bash
sudo ss -ltnp '( sport = :6379 )'
systemctl status redis-server
```

기존 Redis를 사용할 경우 `.env` 인증정보를 맞추거나, 기존 서비스를 중지한 뒤 Docker Redis를 실행합니다.

### 같은 Wi-Fi에서 접속되지 않음

```bash
ss -ltnp | grep 5000
hostname -I
sudo ufw status
```

`.env`는 다음처럼 설정되어야 합니다.

```text
FLASK_HOST=0.0.0.0
FLASK_PORT=5000
```

브라우저에서는 `0.0.0.0`이 아니라 실행 PC의 실제 IP로 접속합니다.

### `.venv`를 다른 컴퓨터에서 복사했더니 실행되지 않음

복사한 가상환경을 삭제하고 새로 만듭니다.

```bash
rm -rf .venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 13. 안전 및 운영 주의사항

- Flask 개발 서버는 부트캠프·시연·내부망 용도입니다.
- 관리자 계정과 Redis 비밀번호는 기본값을 사용하지 마세요.
- `.env`를 메신저, GitHub, 공개 클라우드에 올리지 마세요.
- Redis 포트는 `127.0.0.1`에만 노출되므로 같은 Wi-Fi에서 Redis에 직접 접근할 수 없습니다. 외부 프로그램은 ROS Bridge나 Flask API를 이용하는 구성이 안전합니다.
- `ros_object_bridge.py`의 호환 서비스는 현재 Robot Control과의 연동을 위해 유지되어 있습니다. 호출 노드를 모두 수정하기 전에는 삭제하지 마세요.
