"""
비전 노드 (책상 전체 스캔 + 특정 물체 찾기) 통합
카메라 구독/좌표변환 로직은 공용으로 한 번만 두고,
그 위에 세 가지(서비스 2개 + 액션 1개)를 얹음:

  - ScanRequest 서비스 ('yolo_scan_request')
      -> 책상 전체 스캔. 로봇제어가 웨이포인트마다 도착할 때 1번씩 호출.
         1프레임 인식 + DB Bridge 전송 후 바로 응답.

  - GripBoundingBox 서비스 ('grip_bounding_box')
      -> DB에 저장된 좌표로 이미 이동한 상태에서, 그 자리에 물건이 실제로
         있는지 재확인 + 그립에 필요한 정밀 정보(좌표/bbox/raw depth) 응답.
         응답 필드에 found가 없어서, 전부 0이면 "없음"으로 간주하는 규칙 사용
         (로봇제어와 확인 필요).

  - FindTargetOrder 액션 ('find_target_order')
      -> grip_bounding_box에서 못 찾았을 때 호출. 특정 물체를 찾을 때까지
         계속 최신 프레임 확인, state로 진행상황 feedback,
         찾으면 result(found, coordinate, message) 반환.

좌표 변환은 팀 공용 모듈 rgbd_pixel_to_base.py의 RgbdPixelToBase를 그대로 사용함.
(Hand-Eye 캘리브레이션 행렬 + TF(base_link<->link_6)로 로봇 베이스 좌표계 계산)

★ 이번 변경사항: DB 연동 방식만 HTTP(requests) -> Redis Bridge(ROS2 토픽 publish)로 교체.
   나머지(좌표 변환, 서비스/액션 구조, 필드명)는 통합 테스트에서 검증된 원본 그대로 유지.
   pose는 현재 [x, y, z] 3개만 전송 (rx, ry, rz 로봇 자세 각도는 추후 그리퍼 계산 추가 시 반영 예정).
"""

import json
import math
import sys
import time
import threading
from pathlib import Path
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import message_filters

import tf2_ros

# 같은 패키지 안에 넣어둔 팀 공용 좌표변환 모듈
# (패키지 구조에 맞게 상대/절대 import 조정 필요)
from .rgbd_pixel_to_base import RgbdPixelToBase, bbox_center_xyxy

# TODO 실제 인터페이스 패키지명 다시 한번 확인
from hey_doopal_msg.srv import ScanRequest, GripBoundingBox
from hey_doopal_msg.action import FindTargetOrder


class ObjectDetectionNode(Node):
    def __init__(
        self,
        model,
        color_topic='/camera/camera/color/image_raw',
        depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/camera/color/camera_info',
        hand_eye_transform_path='/home/rokey/cobot_ws/src/simbongsa/desk_detect/desk_detect/T_gripper2camera.npy',  # 실제 경로
        base_frame='base_link',          # 로봇제어 실제 이름 확인 필요
        calibration_frame='link_6',      # 실제 TF 이름 확인 필요
        full_scan_conf=0.6,
        find_target_conf=0.5,
        find_target_timeout=15.0,        # 초. 이 시간 안에 못 찾으면 is_found=False
        find_target_interval=0.15,       # 재시도 간격(초)
        grip_retry_attempts=3,           # grip_bounding_box: 순간 인식실패 대비 짧은 재시도 횟수
        grip_retry_interval=0.15,        # grip_bounding_box: 재시도 간격(초)
        per_class_conf_threshold=None,   # 예: {'airpods': 0.9} -> airpods만 0.9, 나머진 기존 threshold 그대로
    ):
        super().__init__('object_detection_node')
        self.model = model
        self.classNames = model.names
        self.bridge = CvBridge()
        self.full_scan_conf = full_scan_conf            # 전체 스캔용 
        self.find_target_conf = find_target_conf        # 타겟 스캔용 
        self.find_target_timeout = find_target_timeout
        self.find_target_interval = find_target_interval
        self.grip_retry_attempts = grip_retry_attempts
        self.grip_retry_interval = grip_retry_interval
        self.per_class_conf_threshold = per_class_conf_threshold or {}

        # ---------- TF2 + 팀 공용 좌표 변환기 ----------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.coordinate_transformer = RgbdPixelToBase(
            tf_buffer=self.tf_buffer,
            transform_path=hand_eye_transform_path,
            base_frame=base_frame,
            calibration_frame=calibration_frame,
            transform_direction='gripper_to_camera',
            transform_translation_unit='auto',
        )

        # ---------- 카메라 관련 (공용) ----------
        self.latest_camera_info = None
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.latest_color = None
        self.latest_depth = None
        self.latest_depth_encoding = None
        self.frame_lock = threading.Lock()

        # 서비스(전체 스캔)와 액션(타겟 찾기)이 동시에 self.model(...)을 호출할 수 있어서
        # (MultiThreadedExecutor라 실제로 동시 실행 가능) 모델 추론만은 순서 강제
        self.inference_lock = threading.Lock()

        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.frame_callback)

        # ---------- ★ Redis Bridge 전송용 Publisher (신규) ----------
        # UI ZIP의 ros_object_bridge.py가 이 토픽의 JSON을 받아 Redis에 저장함.
        self.object_detection_pub = self.create_publisher(
            String,
            '/assistive/object_detection',
            10,
        )

        # ---------- 서비스 서버: 전체 스캔 ----------
        service_cb_group = ReentrantCallbackGroup()
        self.scan_srv = self.create_service(
            ScanRequest, 'yolo_scan_request',
            self.handle_scan_request, callback_group=service_cb_group)

        # ---------- 서비스 서버: 그립 직전 정밀 확인 ----------
        self.grip_srv = self.create_service(
            GripBoundingBox, 'grip_bounding_box',
            self.handle_grip_bounding_box, callback_group=service_cb_group)

        # ---------- 액션 서버: 타겟 찾기 ----------
        action_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FindTargetOrder,
            'find_target_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_cb_group,
        )

        self.get_logger().info(
            'ObjectDetectionNode 준비 완료. yolo_scan_request(서비스), '
            'grip_bounding_box(서비스), find_target_order(액션) 대기 중...')

    # ================= 카메라 수신 (공용) =================
    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg  # raw 메시지 그대로 저장 (변환기가 직접 씀)

    def frame_callback(self, color_msg: Image, depth_msg: Image):
        color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        # depth는 encoding 그대로 유지해야 변환기가 mm/m 판단 가능 -> passthrough
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        with self.frame_lock:
            self.latest_color = color_img
            self.latest_depth = depth_img
            self.latest_depth_encoding = depth_msg.encoding

    # ================= 인식 + 좌표 변환 (공용) =================
    def run_inference_on_latest_frame(self, conf_threshold, target_label=None):
        """target_label=None -> 전체 스캔, 지정하면 그 라벨만 필터링 (타겟 찾기)"""
        with self.frame_lock:
            if self.latest_color is None or self.latest_depth is None:
                return []
            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()
            depth_encoding = self.latest_depth_encoding

        camera_info = self.latest_camera_info
        if camera_info is None:
            return []

        with self.inference_lock:
            results = self.model(color_img, verbose=False)
        detections = []

        for r in results:
            for box in r.boxes:
                confidence = math.ceil((box.conf[0] * 100)) / 100
                if confidence < conf_threshold:
                    continue

                cls = int(box.cls[0])
                label = self.classNames.get(cls, f'class_{cls}')
                # 클래스별로 threshold를 다르게 주고 싶으면 여기서 오버라이드
                # (없으면 기존처럼 conf_threshold 그대로 사용)
                effective_threshold = self.per_class_conf_threshold.get(label, conf_threshold)
                if confidence < effective_threshold:
                    continue

                if target_label and label != target_label:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cu, cv = bbox_center_xyxy(x1, y1, x2, y2)

                coord_result = self.coordinate_transformer.pixel_to_base(
                    u=cu, v=cv,
                    depth_image=depth_img,
                    depth_encoding=depth_encoding,
                    camera_info=camera_info,
                )
                if coord_result is None:
                    # depth 유효값 부족 또는 TF(base_link<->calibration_frame) 조회 실패
                    continue

                bx, by, bz = coord_result.base_point_m

                detections.append({
                    'class_name': label,
                    'confidence': confidence,
                    'x': round(float(bx), 4),          # 로봇 베이스 좌표계 (m)
                    'y': round(float(by), 4),
                    'z': round(float(bz), 4),
                    'camera_depth_z': round(float(coord_result.depth_m), 4),  # 카메라 raw depth (m)
                    'bbox_width': x2 - x1,
                    'bbox_height': y2 - y1,
                })

        return detections

    # ================= ★ Redis Bridge 전송 (신규 - 기존 upsert_to_db 대체) =================
    def publish_to_db_bridge(self, detections):
        """
        UI ZIP의 ros_object_bridge.py가 구독하는
        /assistive/object_detection 토픽으로 객체 정보를 전송한다.

        메시지 형식:
          {
            "record_name": "객체명",
            "data": {"pose": [x_mm, y_mm, z_mm]},
            "replace": true
          }

        ※ pose는 현재 [x, y, z] 3개만 전송함 (mm 단위로 변환해서 보냄).
          로봇 자세 각도(rx, ry, rz)는 아직 계산 안 하고 있어서 미포함.
          -> 그리퍼 계산 로직 추가할 때 pose를 6개로 확장하면 됨.
        """
        for detection in detections:
            class_name = detection.get('class_name')

            if not class_name:
                self.get_logger().error(
                    '[DB Bridge] class_name이 없어 전송하지 않습니다.')
                continue

            pose = [
                round(float(detection['x']) * 1000.0, 2),  # m -> mm
                round(float(detection['y']) * 1000.0, 2),
                round(float(detection['z']) * 1000.0, 2),
            ]

            payload = {
                'record_name': class_name,
                'data': {'pose': pose},
                'replace': True,
            }

            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            self.object_detection_pub.publish(message)

            self.get_logger().info(
                f'[DB Bridge] 객체 정보 발행: {class_name} -> {pose}')

    # ================= 서비스: 전체 스캔 =================
    def handle_scan_request(self, request, response):
        self.get_logger().info(f'[전체 스캔] 요청 수신 (waypoint_id="{request.waypoint_id}")')

        detections = self.run_inference_on_latest_frame(self.full_scan_conf)

        if detections:
            self.publish_to_db_bridge(detections)  # ★ upsert_to_db -> publish_to_db_bridge
            response.success = True
            response.message = f'{len(detections)}개 객체 스캔 및 DB Bridge 전송 완료'
            response.detected_count = len(detections)
        else:
            response.success = True
            response.message = '탐지된 객체 없음 (또는 depth/TF 문제로 계산 실패)'
            response.detected_count = 0

        self.get_logger().info(f'[전체 스캔] 응답: {response.message}')
        return response

    # ================= 서비스: 그립 직전 정밀 확인 =================
    def handle_grip_bounding_box(self, request, response):
        target_label = request.target
        self.get_logger().info(f'[그립 확인] 요청 수신: target="{target_label}"')

        # "여기 물건 있다"는 전제긴 하지만, 한 프레임만 보고 판단하면
        # 모션블러/조명/depth 노이즈로 순간 인식 실패할 수 있어서 짧게 재시도
        best = None
        for attempt in range(1, self.grip_retry_attempts + 1):
            detections = self.run_inference_on_latest_frame(
                self.find_target_conf, target_label=target_label)

            if detections:
                best = max(detections, key=lambda d: d['confidence'])
                break

            self.get_logger().warn(
                f'[그립 확인] "{target_label}" {attempt}/{self.grip_retry_attempts}번째 '
                f'시도 실패, 재시도...')
            if attempt < self.grip_retry_attempts:
                time.sleep(self.grip_retry_interval)

        if best is not None:
            self.publish_to_db_bridge([best])  # ★ upsert_to_db -> publish_to_db_bridge

            response.coordinate = [best['x'] * 1000, best['y'] * 1000, best['z'] * 1000]
            response.bbox_width = float(best['bbox_width'])
            response.bbox_height = float(best['bbox_height'])
            response.camera_depth_z = float(best['camera_depth_z'])
            response.is_find = True
            self.get_logger().info(f'[그립 확인] "{target_label}" 확인됨')
        else:
            # found 필드가 없는 인터페이스라 "전부 0"으로 없음을 표현
            # (로봇제어와 이 규칙 확인 필요) - grip_retry_attempts번 다 실패한 경우에만 여기 도달
            response.coordinate = [0.0, 0.0, 0.0]
            response.bbox_width = 0.0
            response.bbox_height = 0.0
            response.camera_depth_z = 0.0
            response.is_find = False
            self.get_logger().warn(
                f'[그립 확인] "{target_label}" {self.grip_retry_attempts}번 재시도했지만 '
                f'끝내 못 찾음')

        return response

    # ================= 액션: 타겟 찾기 =================
    def goal_callback(self, goal_request):
        self.get_logger().info(f'[타겟 찾기] 목표 수신: target_name="{goal_request.target_name}"')
        if not goal_request.target_name:
            self.get_logger().warn('[타겟 찾기] target_name이 비어있는 goal -> 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # TODO 예외처리 필요
    def cancel_callback(self, goal_handle):
        self.get_logger().info('[타겟 찾기] 취소 요청 수신')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_label = goal_handle.request.target_name
        feedback_msg = FindTargetOrder.Feedback()
        result = FindTargetOrder.Result()

        start = time.time()
        attempts = 0
        found_detection = None

        while time.time() - start < self.find_target_timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.found = False
                result.coordinate = [0.0] * 3
                result.message = ''
                self.get_logger().info(f'[타겟 찾기] "{target_label}" 취소됨 ({attempts}번 시도)')
                return result

            attempts += 1
            detections = self.run_inference_on_latest_frame(
                self.find_target_conf, target_label=target_label)

            feedback_msg.state = 'detected' if detections else 'searching'
            goal_handle.publish_feedback(feedback_msg)

            if detections:
                found_detection = max(detections, key=lambda d: d['confidence'])
                break

            time.sleep(self.find_target_interval)

        if found_detection:
            feedback_msg.state = 'calculating'
            goal_handle.publish_feedback(feedback_msg)

            # run_inference_on_latest_frame에서 이미 base 좌표까지 다 계산해서 넣어둠
            self.publish_to_db_bridge([found_detection])  # ★ upsert_to_db -> publish_to_db_bridge
            goal_handle.succeed()

            result.found = True
            result.coordinate = [
                found_detection['x'] * 1000,
                found_detection['y'] * 1000,
                found_detection['z'] * 1000,
            ]
            result.message = found_detection['class_name']
            self.get_logger().info(f'[타겟 찾기] "{target_label}" {attempts}번 시도 만에 발견')
        else:
            goal_handle.abort()
            result.found = False
            result.coordinate = [0.0] * 3
            result.message = ''
            self.get_logger().warn(
                f'[타겟 찾기] "{target_label}" {self.find_target_timeout}초 동안 못 찾음 '
                f'({attempts}번 시도)')

        return result


def main():
    model_path = '/home/rokey/cobot_ws/src/simbongsa/desk_detect/my_seg_best.pt'  # 학습된 모델 경로로 수정

    if not Path(model_path).exists():
        print(f'File not found: {model_path}')
        sys.exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f'Unsupported model format: {suffix}')
        sys.exit(1)

    rclpy.init()
    node = ObjectDetectionNode(
        model,
        per_class_conf_threshold={'airpods': 0.9},
    )

    # 서비스/액션 콜백이 서로, 그리고 frame_callback과 동시에 돌아야 하므로 MultiThreadedExecutor 필수
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('Ctrl+C received. Exiting...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()