# M0609 RealSense Hand Scan & SpeedL Tracking

Doosan M0609의 그리퍼 상단 RealSense 카메라로 손바닥을 찾고, 손 위치까지 영상 기반으로 접근하는 ROS 2 노드 두 개입니다.

- `hand_detection.py`
  - 손바닥을 검출하고 `base_link` 기준 3D 좌표를 Action Result로 반환합니다.
- `hand_tracking.py`
  - 손바닥의 영상 중심 오차와 Depth 오차를 이용해 `/dsr01/speedl_stream`을 발행합니다.

## 1. 정리하면서 변경한 내용

### Action Server

- 매 프레임 `T_gripper2camera.npy`를 다시 읽던 처리를 제거하고 시작 시 한 번만 로드합니다.
- TF의 Quaternion을 ZYZ Euler 각도로 변환한 뒤 다시 회전행렬로 복원하던 중복 계산을 제거했습니다.
- 좌표 변환을 다음 4×4 행렬 계산으로 단순화했습니다.

```text
p_base = T_base_calibration(TF) @ T_calibration_camera(NPY) @ p_camera
```

- Action이 대기 중일 때 이미지 변환과 MediaPipe 추론을 수행하지 않도록 변경했습니다.
- 화면 표시가 꺼져 있을 때 landmark drawing과 영상 복사를 수행하지 않습니다.
- 사용되지 않던 `transform_direction` 파라미터를 제거했습니다.
- Action Result, PointStamped 진단 토픽, JSON 진단 좌표를 모두 소수점 둘째 자리까지 반올림합니다.
- 종료 시 `/find_hand_order/active=False`, `/find_hand_order/succeeded=False`를 발행해 재시작한 추적 노드가 오래된 성공 상태로 arm되는 가능성을 줄였습니다.

### SpeedL Tracking Node

- 실제 출력이나 제어 판단에 사용되지 않던 `status_text` 상태 문자열을 제거했습니다.
- `tracking_enabled=False`로 충분히 막을 수 있던 중복 완료 플래그를 제거했습니다.
- 손 스캔 중에는 callback 순서와 관계없이 `control_callback()`에서 SpeedL을 다시 차단합니다.
- NPY 회전행렬은 시작 시 한 번만 로드하고 SVD로 정규직교화합니다.
- NaN/Inf가 포함된 SpeedL 명령은 0 속도로 대체합니다.
- `link_6 -> gripper_tcp` static TF 발행 여부를 `publish_gripper_tcp_static_tf` 파라미터로 제어할 수 있습니다.

## 2. 시스템 흐름

```text
Robot Control
    |
    | FindOrder Action Goal: target_name="hand"
    v
/find_hand_order Action Server
    |-- /find_hand_order/active=True  -> SpeedL 강제 차단
    |-- 손 검출 + 좌표 안정화
    |-- Result.coordinate=[x, y, z]
    `-- /find_hand_order/succeeded=True

Robot Control
    |
    | /arrived_goal (std_srvs/srv/Trigger)
    v
SpeedL Tracking Node
    |-- /hand_tracking_request=True
    |-- 영상 오차 기반 SpeedL 제어
    |-- 목표 픽셀 + 목표 Depth 도달
    `-- /hand_arrived=True
```

`/arrived_goal`은 성공한 손 스캔 한 번당 한 번만 허용됩니다. 손 스캔 도중 호출하거나 성공한 스캔 없이 호출하면 거부됩니다.

## 3. 파일 배치

두 Python 파일과 캘리브레이션 파일을 같은 디렉터리에 둡니다.

```text
hand_tracking/
├── hand_detection.py
├── hand_tracking.py
├── T_gripper2camera.npy
├── README.md
└── requirements.txt
```

기본 경로는 각 Python 파일이 있는 디렉터리의 `T_gripper2camera.npy`입니다. 다른 위치를 사용할 때는 ROS 파라미터 `transform_path`를 지정합니다.

## 4. 요구 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Doosan Robotics M0609
- Intel RealSense RGB-D Camera
- MediaPipe Hands
- `hey_doopal_msg/action/FindOrder`
- `dsr_msgs2/msg/SpeedlStream`

### ROS 패키지 의존성

아래 항목은 `requirements.txt`가 아니라 apt 또는 ROS workspace의 `package.xml`/`colcon`으로 설치해야 합니다.

```text
rclpy
sensor_msgs
geometry_msgs
std_msgs
std_srvs
tf2_ros
cv_bridge
message_filters
hey_doopal_msg
dsr_msgs2
```

설치 예시는 다음과 같습니다.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-tf2-ros \
  ros-humble-realsense2-camera
```

Python 패키지는 다음과 같이 설치합니다.

```bash
python3 -m pip install --user -r requirements.txt
```

이미 다른 OpenCV wheel인 `opencv-contrib-python` 또는 `opencv-python-headless`를 사용 중이라면 OpenCV 배포판을 여러 개 동시에 설치하지 마십시오. 해당 환경에서는 `requirements.txt`의 `opencv-python` 줄을 제거하고 기존 OpenCV를 사용합니다.

## 5. FindOrder Action 정의

`hey_doopal_msg/action/FindOrder.action`은 다음 구조여야 합니다.

```action
string target_name
---
bool found
float64[3] coordinate
string message
---
string state
```

인터페이스 패키지를 수정했다면 다시 빌드합니다.

```bash
cd <ROS2_WORKSPACE>
colcon build --packages-select hey_doopal_msg
source install/setup.bash
```

## 6. ROS 패키지에 등록하는 방법

예를 들어 두 파일을 `rokey/rokey/` 안에 배치했다면 `setup.py`의 `console_scripts`에 다음 항목을 추가할 수 있습니다.

```python
entry_points={
    "console_scripts": [
        "palm_3d_action_server = rokey.hand_detection:main",
        "palm_speedl_follow = rokey.hand_tracking:main",
    ],
},
```

`package.xml`에는 ROS 의존성을 추가합니다.

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
<depend>tf2_ros</depend>
<depend>cv_bridge</depend>
<depend>message_filters</depend>
<depend>hey_doopal_msg</depend>
<depend>dsr_msgs2</depend>
```

빌드 후에는 다음과 같이 실행합니다.

```bash
cd <ROS2_WORKSPACE>
colcon build --symlink-install --packages-select rokey
source install/setup.bash

ros2 run rokey palm_3d_action_server
ros2 run rokey palm_speedl_follow
```

패키지에 등록하지 않고 Python 파일을 직접 실행할 수도 있습니다.

```bash
python3 hand_detection.py
python3 hand_tracking.py
```

## 7. 실행 순서

### 터미널 1: Doosan bringup

환경에 맞는 workspace를 source한 뒤 실제 로봇을 연결합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  host:=192.168.1.100 \
  port:=12345 \
  model:=m0609
```

### 터미널 2: RealSense

Aligned Depth를 활성화해야 합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

필수 토픽을 확인합니다.

```bash
ros2 topic list | grep camera
```

기본 토픽은 다음과 같습니다.

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

### 터미널 3: 손 좌표 Action Server

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source <ROS2_WORKSPACE>/install/setup.bash

python3 hand_detection.py
```

### 터미널 4: SpeedL 추적 노드

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source <ROS2_WORKSPACE>/install/setup.bash

python3 hand_tracking.py
```

## 8. 단독 통신 테스트

### 손 스캔 Action

```bash
ros2 action send_goal \
  /find_hand_order \
  hey_doopal_msg/action/FindOrder \
  "{target_name: 'hand'}" \
  --feedback
```

성공 Result 예시는 다음과 같습니다.

```text
found: true
coordinate: [445.29, -23.52, 283.56]
message: hand detected
```

좌표는 `base_link` 기준 mm이며 소수점 둘째 자리까지 반환됩니다.

### 추적 시작 서비스

손 스캔 성공 후 호출합니다.

```bash
ros2 service call /arrived_goal std_srvs/srv/Trigger "{}"
```

성공 시 다음 신호가 발행됩니다.

```text
/hand_tracking_request = True
```

목표 위치에 도착하면 다음 신호가 발행되고 SpeedL은 0으로 정지합니다.

```text
/hand_arrived = True
```

## 9. 주요 ROS 인터페이스

| 구분 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Action Server | `/find_hand_order` | `hey_doopal_msg/action/FindOrder` | 손 검출과 base 좌표 반환 |
| Publisher | `/find_hand_order/active` | `std_msgs/msg/Bool` | 손 스캔 수행 중 여부 |
| Publisher | `/find_hand_order/succeeded` | `std_msgs/msg/Bool` | 손 좌표 확정 여부 |
| Publisher | `/mediapipe_palm_3d/camera_point_mm` | `geometry_msgs/msg/PointStamped` | 카메라 기준 손 좌표 |
| Publisher | `/mediapipe_palm_3d/base_point_mm` | `geometry_msgs/msg/PointStamped` | base 기준 손 좌표 |
| Publisher | `/mediapipe_palm_3d/detected` | `std_msgs/msg/Bool` | 현재 손 검출 여부 |
| Publisher | `/mediapipe_palm_3d/info` | `std_msgs/msg/String` | JSON 진단 정보 |
| Service | `/arrived_goal` | `std_srvs/srv/Trigger` | SpeedL 손 추적 시작 |
| Publisher | `/hand_tracking_request` | `std_msgs/msg/Bool` | VLA에 추적 시작 알림 |
| Publisher | `/hand_arrived` | `std_msgs/msg/Bool` | VLA에 도착 알림 |
| Publisher | `/dsr01/speedl_stream` | `dsr_msgs2/msg/SpeedlStream` | 로봇 Cartesian 속도 명령 |

## 10. 주요 파라미터

### Action Server

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `transform_path` | 같은 폴더의 NPY | 손-눈 캘리브레이션 행렬 |
| `transform_translation_unit` | `mm` | NPY 평행이동 단위: `mm`, `m`, `auto` |
| `base_z_offset_mm` | `250.0` | 최종 base Z 보정값 |
| `stable_frames` | `5` | 좌표 확정에 필요한 연속 프레임 수 |
| `stable_max_jump_mm` | `35.0` | 안정 좌표로 인정할 프레임 간 최대 이동량 |
| `find_hand_timeout_sec` | `15.0` | Action 제한 시간 |
| `smoothing_alpha` | `0.35` | 좌표 EMA 필터 계수 |
| `show_window` | `False` | OpenCV 확인 화면 표시 |

실행 예시:

```bash
python3 hand_detection.py --ros-args \
  -p find_hand_timeout_sec:=20.0 \
  -p stable_frames:=7 \
  -p show_window:=true
```

### SpeedL Tracking

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `target_u_ratio` | `0.5` | 목표 가로 픽셀 비율 |
| `target_v_ratio` | `0.6667` | 목표 세로 픽셀 비율 |
| `target_depth_mm` | `230.0` | 카메라와 손 사이 목표 Depth |
| `kp_lateral` | `1.2` | 횡방향 비례 이득 |
| `kp_depth` | `0.8` | 전진 방향 비례 이득 |
| `max_lateral_speed_mm_s` | `250.0` | 횡방향 속도 제한 |
| `max_forward_speed_mm_s` | `500.0` | 전진 속도 제한 |
| `max_total_speed_mm_s` | `500.0` | 전체 선속도 제한 |
| `linear_acc_mm_s2` | `60.0` | 선가속도 설정 |
| `hand_timeout_sec` | `0.35` | 측정값 만료 시간 |
| `arrival_stable_cycles` | `3` | 도착 판정 연속 제어 주기 |
| `publish_gripper_tcp_static_tf` | `True` | `link_6 -> gripper_tcp` TF 발행 |

속도를 낮춘 안전 테스트 예시:

```bash
python3 hand_tracking.py --ros-args \
  -p max_lateral_speed_mm_s:=80.0 \
  -p max_forward_speed_mm_s:=120.0 \
  -p max_total_speed_mm_s:=120.0 \
  -p linear_acc_mm_s2:=40.0
```

카메라 축 방향이 반대로 움직이면 해당 축의 sign만 변경합니다.

```bash
python3 hand_tracking.py --ros-args \
  -p camera_x_sign:=-1.0
```

## 11. TF 및 NPY 주의사항

### 필수 TF

두 노드는 최소한 다음 TF를 조회할 수 있어야 합니다.

```text
base_link -> link_6
```

확인 명령:

```bash
ros2 run tf2_ros tf2_echo base_link link_6
```

### gripper_tcp static TF

추적 노드가 기본적으로 다음 TF를 발행합니다.

```text
link_6 -> gripper_tcp
translation = [0.0, 0.0, 0.250] m
rotation = [0.0, 0.0, 0.0] rad
```

외부에서 같은 child frame의 static TF를 이미 발행하고 있다면 TF authority 충돌을 막기 위해 다음 파라미터를 사용합니다.

```bash
-p publish_gripper_tcp_static_tf:=false
```

### NPY 방향

프로젝트의 기존 계산을 유지하여 Action Server는 NPY를 역행렬 없이 직접 적용합니다. SpeedL 노드는 기본적으로 NPY 회전을 `camera_to_link6`로 해석합니다.

```text
Action 좌표: T_base_link6 @ NPY @ p_camera
Speed 변환:   R_base_link6 @ R_link6_camera @ v_camera
```

파일 이름이 `T_gripper2camera.npy`더라도 실제 저장 방향과 코드의 해석이 일치하는지 반드시 확인해야 합니다. 방향이 반대라면 추적 노드에서 다음 값을 시험합니다.

```bash
-p npy_rotation_direction:=link6_to_camera
```

Action Server의 좌표 방향까지 반대라면 NPY 생성 단계에서 올바른 방향으로 다시 저장하는 것이 안전합니다. 코드 안에서 임의로 역행렬을 추가하지 마십시오.

## 12. 안전 점검

실제 로봇에서 처음 실행할 때는 다음 순서를 권장합니다.

1. 로봇 주변과 예상 이동 경로를 비웁니다.
2. 비상정지 버튼에 즉시 접근할 수 있도록 합니다.
3. SpeedL 최대 속도를 낮춰 시작합니다.
4. `/find_hand_order/active=True`일 때 `/dsr01/speedl_stream`이 0인지 확인합니다.
5. 손을 화면 여러 위치에 두고 X/Y 이동 방향을 검증합니다.
6. 목표 Depth보다 가까워졌을 때 로봇이 후퇴하지 않고 정지하는지 확인합니다.
7. 축 방향이 반대면 `camera_x_sign`, `camera_y_sign`, `camera_z_sign`을 한 축씩 수정합니다.
8. 검증 후에만 속도와 이득을 단계적으로 올립니다.

SpeedL 확인:

```bash
ros2 topic echo /dsr01/speedl_stream
```

상태 확인:

```bash
ros2 topic echo /find_hand_order/active
ros2 topic echo /find_hand_order/succeeded
ros2 topic echo /hand_tracking_request
ros2 topic echo /hand_arrived
```

## 13. 문제 해결

### `Transform file not found`

`T_gripper2camera.npy`를 Python 파일과 같은 위치에 두거나 절대 경로를 전달합니다.

```bash
-p transform_path:=/absolute/path/T_gripper2camera.npy
```

### `TF unavailable: base_link <- link_6`

- Doosan bringup이 실행 중인지 확인합니다.
- 실제 TF frame 이름이 `base_link`, `link_6`인지 확인합니다.
- 필요하면 `base_frame`, `calibration_frame` 파라미터를 실제 이름으로 변경합니다.

```bash
ros2 run tf2_tools view_frames
```

### `/arrived_goal`이 거부됨

다음 순서를 지켜야 합니다.

1. `/find_hand_order` Goal 전송
2. Action 성공
3. `/arrived_goal` 호출

테스트 목적으로 gate를 끌 수는 있지만 실제 로봇에서는 권장하지 않습니다.

```bash
-p require_hand_scan_success_before_tracking:=false
```

### 손을 찾았지만 좌표가 확정되지 않음

- `stable_frames`를 줄입니다.
- `stable_max_jump_mm`를 조금 늘립니다.
- Depth 영상 노이즈와 카메라 진동을 확인합니다.
- `coordinate_max_age_sec`가 너무 짧지 않은지 확인합니다.

### 손을 놓쳤을 때 로봇이 계속 움직임

현재 코드는 다음 조건에서 0 속도를 발행합니다.

- 손 측정값이 `hand_timeout_sec`보다 오래됨
- 안정 프레임 수 부족
- TF 조회 실패
- 손 스캔 시작
- 노드 종료

계속 움직인다면 `/dsr01/speedl_stream`을 발행하는 다른 노드가 있는지 확인합니다.

```bash
ros2 topic info /dsr01/speedl_stream --verbose
```

## 14. 남겨 둔 기능

다음 항목은 중복처럼 보일 수 있지만 현재 시스템 통합과 디버깅에 필요할 수 있어 유지했습니다.

- `/mediapipe_palm_3d/*` 진단 토픽
- TRANSIENT_LOCAL 방식의 스캔 상태 토픽
- 선택형 OpenCV 확인 화면
- `link_6 -> gripper_tcp` static TF 발행 기능
- 카메라 축별 sign 파라미터

사용하지 않는 것이 확실해지면 진단 publisher와 OpenCV 표시 코드를 별도 디버그 노드로 분리할 수 있습니다.
