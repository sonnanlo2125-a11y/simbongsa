import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from pymodbus.client.sync import ModbusTcpClient as ModbusClient

from hey_doopal_msg.action import FindOrder
from hey_doopal_msg.srv import ScanRequest
from hey_doopal_msg.srv import VoiceKeyword
from hey_doopal_msg.srv import GripBoundingBox
from robot_control.cone_scan import ConeScanner
from rclpy.executors import MultiThreadedExecutor

import DR_init

# =========================
# Gripper Configuration
# =========================

GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = 502

# =========================
# Robot Configuration
# =========================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 60
ACCELERATION = 60

# =========================
# Scan Configuration
# =========================

SCAN_TILT_ANGLE = 30.0
SCAN_POINT_COUNT = 8

SCAN_VELOCITY = [20, 10]
SCAN_ACCELERATION = [40, 20]

# YOLO 서비스가 나타날 때까지 기다리는 시간
YOLO_SERVICE_WAIT_TIMEOUT = 5.0

# YOLO 한 번의 스캔 응답을 기다리는 시간
YOLO_SCAN_RESPONSE_TIMEOUT = 10.0

# =========================
# Position Configuration
# =========================
# 나중에 데이터베이스 또는 통신으로 바뀔 부분 ctrl+f "CONFIG"
CONFIG = {
    "SCAN_WAYPOINT1":[434.7, 21.07, 552.72, 63.44, -179.21, 62.54],
    "SCAN_WAYPOINT2":[434.7, -187.14, 552.72, 63.44, -179.21, 62.54],
    "POS1": [744.33, -127.61, 228.65, 169.90,-142.46, 97.08],
    "POS2": [342.82, 171.74, 395.16, 26.39, -176.26, -1.66],
    "HAND_SCAN": [406.06, -167.10, 555.08, 95.52, -104.05, 24.04],
    "TARGET_SCAN": [684.13, -25.36, 235.69, 169.25, -140.48, 159.67],
    "apple": [434.7, -187.14, 302.72, 63.44, -179.21, 62.54],
}

# 1. DSR 정보 등록
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 2. ROS 초기화
rclpy.init()

# 3. DSR_ROBOT2가 사용할 노드 생성
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movel,
        get_current_posx,
        DR_TOOL,
        DR_BASE,
    )
    from DR_common2 import posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# ##############################################################################
# [ADD] GRIPPER CONTROL CLASS START
# ##############################################################################
class AdaptiveGripper:
    def __init__(self, ip, port=502):
        self.client = ModbusClient(ip, port=port, stopbits=1, bytesize=8, parity="E", baudrate=115200, timeout=1)
        self.client.connect()
        self.max_width = 1100

    def move_gripper(self, width_val, force_val):
        params = [force_val, width_val, 16] 
        self.client.write_registers(address=0, values=params, unit=65)

    def get_status(self):
        result = self.client.read_holding_registers(address=268, count=1, unit=65)
        status = format(result.registers[0], "016b")
        return [int(status[-2])] # 1이면 grip detected
    def close_connection(self):
        self.client.close()
# ##############################################################################
# [ADD] GRIPPER CONTROL CLASS END
# ##############################################################################

class TargetScanNode(Node):

    def __init__(self):
        super().__init__("robot_control_node")

        self.gripper = AdaptiveGripper(GRIPPER_IP,GRIPPER_PORT) 

        self.scan_thread = None
        self.detected_coordinate = None

        self.find_order_client = ActionClient(
            self,
            FindOrder,
            "/find_order",
        )

        self.scan_table_client= self.create_client(
            ScanRequest,
            "/yolo_scan_request"
        )
        
        self.get_keyword_service = self.create_service(
            VoiceKeyword,
            "/get_keyword",
            self.command_callback,
        )

        self.grip_bbox_client = self.create_client(
            GripBoundingBox,
            "/grip_bounding_box",
        )

        self.cone_scanner = ConeScanner(
            node=self,
            action_client=self.find_order_client,
            scan_tilt_angle=SCAN_TILT_ANGLE,
            scan_point_count=SCAN_POINT_COUNT,
            scan_velocity=SCAN_VELOCITY,
            scan_acceleration=SCAN_ACCELERATION,
        )

        self.get_logger().info("target_scan_node 시작")

    def command_callback(self, request, response):
        target_names = request.target.strip().split()
        goal_name = request.goal.strip()

        self.get_logger().info(
            f"명령 수신: target={target_names}, goal={goal_name}"
        )

        threading.Thread(
            target=self.execute_robot_task,
            args=(target_names, goal_name),
            daemon=True,
        ).start()

        response.accepted = True
        return response


    # ##############################################################################
    # [ADD] ADAPTIVE GRIP LOGIC START
    def run_adaptive_grip(self, bbox_w, bbox_h, dist):
        pixel_area = bbox_w * bbox_h
        real_area_estimate = pixel_area * (dist ** 2)
        K = 0.00001
        
        force = min(int(150 + (real_area_estimate * K)), 400)
        
        self.get_logger().info(f"그리퍼 계산 [면적:{pixel_area} | 거리:{dist}m | 힘:{force}]")
        
        self.gripper.move_gripper(width_val=0, force_val=force)
        
        time.sleep(2.0)
        if self.gripper.get_status()[0] == 1:
            self.get_logger().info(">> 파지 성공!")
        else:
            self.get_logger().warn(">> 파지 실패")
    # [ADD] ADAPTIVE GRIP LOGIC END
    # ##############################################################################
    
    def execute_robot_task(self, target_names, goal_name):
        task_is_done = False
        goal_is_hand = False

        if not goal_name:
            goal_is_hand = True
        if goal_name == "hand":
            goal_is_hand = True

        # goal 좌표 만들기        
        if goal_is_hand:
            center_pose = CONFIG["HAND_SCAN"]
            target = "hand"
            self.run_cone_scan(center_pose, target)

            goal_coordinate = list(center_pose)

            goal_coordinate[0] = float(self.detected_coordinate[0])
            goal_coordinate[1] = float(self.detected_coordinate[1])
            goal_coordinate[2] = float(self.detected_coordinate[2])

        else:
            if goal_name not in CONFIG:
                self.get_logger().error(
                    f"등록되지 않은 goal: {goal_name}"
                )
                return 

            goal_coordinate = list(CONFIG[goal_name])
                
        # target 좌표 만들기
        for target_name in target_names:
            if target_name in CONFIG:
                target_coordinate = list(CONFIG[target_name])

            self.get_logger().info(f"좌료 설정 완료")

            target_above = list(target_coordinate)
            target_above[2] += 100.0

            movel(target_above,vel=VELOCITY,acc=ACCELERATION)

            self.gripper.move_gripper(width_val=1000, force_val=200)

            grip_request = GripBoundingBox.Request()
            grip_request.target = target_name

            grip_future = self.grip_bbox_client.call_async(grip_request)

            grip_event = threading.Event()

            grip_future.add_done_callback(lambda _future: grip_event.set())

            grip_event.wait()

            grip_response = grip_future.result()

            self.get_logger().info(
                f"YOLO 파지 정보 수신: "
                f"coordinate={list(grip_response.coordinate)}, "
                f"bbox_width={grip_response.bbox_width}, "
                f"bbox_height={grip_response.bbox_height}, "
                f"depth={grip_response.camera_depth_z}"
                f"is_find={grip_response.is_find}"
            )
            # =========================
            # YOLO 좌표로 이동
            # =========================
            if not grip_response.is_find:
                center_pose = target_above
                target = target_name
                self.run_cone_scan(center_pose, target)
                
                target_coordinate = list(center_pose)
                
                target_coordinate[0] = float(self.detected_coordinate[0])
                target_coordinate[1] = float(self.detected_coordinate[1])
                target_coordinate[2] = float(self.detected_coordinate[2])

            grip_coordinate = list(target_coordinate)
            grip_coordinate[0] = float(grip_response.coordinate[0])  
            grip_coordinate[1] = float(grip_response.coordinate[1])
            grip_coordinate[2] = float(grip_response.coordinate[2])

            movel(
                grip_coordinate,
                vel=VELOCITY,
                acc=ACCELERATION,
            )

            # =========================
            # 물체 파지
            # =========================
            self.run_adaptive_grip(
                bbox_w=grip_response.bbox_width,
                bbox_h=grip_response.bbox_height,
                dist=grip_response.camera_depth_z,
            )

            # 물체를 잡은 후 위로 이동
            grip_above = list(grip_coordinate)
            grip_above[2] += 100.0

            movel(
                grip_above,
                vel=VELOCITY,
                acc=ACCELERATION,
            )

            # =========================
            # goal로 이동
            # =========================

            if goal_is_hand:
                movel(
                    goal_coordinate,
                    vel=VELOCITY,
                    acc=ACCELERATION,
                )

            else:
                goal_above = list(goal_coordinate)
                goal_above[2] += 100.0

                movel(
                    goal_above,
                    vel=VELOCITY,
                    acc=ACCELERATION,
                )

                movel(
                    goal_coordinate,
                    vel=VELOCITY,
                    acc=ACCELERATION,
                )

            self.get_logger().info(
                "Pick and Place 작업 완료"
            )

        self.get_logger().info("로봇 작업 완료")
        # 여기까지 오면 스레드 종료


    def run_cone_scan(self, center_pose, target_name):
        coordinate = self.cone_scanner.scan(center_pose, target_name)

        if coordinate is None:
            self.get_logger().warning(f"{target_name} 좌표 탐색 실패")
            return
        
        self.detected_coordinate = coordinate

        self.get_logger().info(f"{target_name} 최종 좌표: {coordinate}")


    # ##############################################################################
        # table_scan 
    def run_table_scan(self):
        """
        로봇을 두 개의 Waypoint로 순차 이동시킨다.

        각 Waypoint에 도착한 후 YOLO 노드에
        ScanRequest 서비스를 요청하고 응답을 기다린다.
        """

        waypoints = [
            "SCAN_WAYPOINT1",
            "SCAN_WAYPOINT2",
        ]

        self.get_logger().info(
            "테이블 스캔 작업을 시작합니다."
        )

        try:
            for waypoint_id in waypoints:
                if not rclpy.ok():
                    self.get_logger().warning( "ROS가 종료되어 테이블 스캔을 중단합니다.")
                    return
                
                # 1. 로봇 이동
                move_success = self.move_to_position(waypoint_id)

                if not move_success:
                    self.get_logger().error( f"{waypoint_id} 이동 실패로 테이블 스캔을 중단합니다.")
                    return

                # 2. YOLO 스캔 요청
                scan_success = self.request_yolo_scan(waypoint_id)

                if not scan_success:
                    self.get_logger().error(f"{waypoint_id} YOLO 스캔 실패로 다음 Waypoint 이동을 중단합니다.")
                    return

                self.get_logger().info(f"{waypoint_id} 작업 완료")

            self.get_logger().info("모든 테이블 Waypoint 스캔이 완료되었습니다.")

        except Exception as error:
            self.get_logger().error(f"테이블 스캔 중 예외 발생: {error}")

    def request_yolo_scan(self, waypoint_id):
        """
        YOLO 노드의 /yolo_scan_request 서비스를 호출한다.

        반환값:
            True:
                서비스 통신 성공 및 response.success == True

            False:
                서비스 없음, 응답 시간 초과,
                통신 예외 또는 response.success == False
        """

        self.get_logger().info(f"YOLO 서비스 확인 중: {waypoint_id}")
        # 서비스 서버가 살아있는지 확인 => True/ False
        service_ready = self.scan_table_client.wait_for_service(
            timeout_sec=YOLO_SERVICE_WAIT_TIMEOUT
        )
        if not service_ready:
            self.get_logger().error("/yolo_scan_request 서비스를 찾을 수 없습니다.")
            return False

        request = ScanRequest.Request()
        request.waypoint_id = waypoint_id

        self.get_logger().info(
            f"YOLO 스캔 요청 전송: waypoint_id={waypoint_id}"
        )

        try:
            future = self.scan_table_client.call_async(request)

        except Exception as error:
            self.get_logger().error(
                f"YOLO 서비스 요청 전송 실패: {error}"
            )
            return False

        # 이 함수는 별도 작업 스레드에서 실행된다.
        # MultiThreadedExecutor는 ROS 서비스 응답을 처리하고,
        # 완료되면 Event를 활성화한다.
        response_event = threading.Event()

        def response_done_callback(_future):
            response_event.set()

        future.add_done_callback(
            response_done_callback
        )

        response_received = response_event.wait(
            timeout=YOLO_SCAN_RESPONSE_TIMEOUT
        )

        if not response_received:
            self.get_logger().error(
                f"YOLO 스캔 응답 시간 초과: "
                f"{waypoint_id}, "
                f"{YOLO_SCAN_RESPONSE_TIMEOUT}초"
            )
            return False

        try:
            response = future.result()

        except Exception as error:
            self.get_logger().error(
                f"YOLO 서비스 응답 처리 실패: {error}"
            )
            return False

        if response is None:
            self.get_logger().error(
                f"YOLO 서비스 응답이 없습니다: {waypoint_id}"
            )
            return False

        # YOLO가 전달한 정보는 현재 제어 판단에는 사용하지 않고
        # 로그만 출력한다.
        self.get_logger().info(
            f"YOLO 응답 수신: "
            f"waypoint_id={waypoint_id}, "
            f"success={response.success}, "
            f"detected_count={response.detected_count}, "
            f"message='{response.message}'"
        )

        if not response.success:
            self.get_logger().error(
                f"YOLO 스캔 자체가 실패했습니다: "
                f"{response.message}"
            )
            return False

        return True

    def move_to_position(self, position_key):
        if position_key not in CONFIG:
            self.get_logger().warning(f"등록되지 않은 위치 키입니다: {position_key}")
            return False

        target_position = position_key

        self.get_logger().info(
            f"{position_key} 이동 시작"
        )
        self.get_logger().info(
            f"이동 좌표: {target_position}"
        )

        try:
            movel(target_position, vel=VELOCITY, acc=ACCELERATION)

        except Exception as error:
            self.get_logger().error(f"{position_key} 이동 중 오류 발생: {error}")
            return False

        self.get_logger().info(f"{position_key} 이동 완료")
        return True

def main(args=None):
    node = TargetScanNode()

    # 중요:
    # rclpy.spin(node)의 기본 global executor를 사용하지 않는다.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info("노드를 종료합니다.")

    finally:
        executor.shutdown()

        node.destroy_node()
        dsr_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()