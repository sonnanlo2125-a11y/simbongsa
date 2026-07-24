import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

from hey_doopal_msg.action import FindOrder
from hey_doopal_msg.srv import ScanRequest
from robot_control.cone_scan import ConeScanner
from rclpy.executors import MultiThreadedExecutor

import DR_init


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

# =========================
# Position Configuration
# =========================

CONFIG = {
    "SCAN_WAYPOINT1":[434.7, 21.07, 552.72, 63.44, -179.21, 62.54],
    "SCAN_WAYPOINT2":[434.7, -187.14, 552.72, 63.44, -179.21, 62.54],
    "POS1": [744.33, -127.61, 228.65, 169.90,-142.46, 97.08],
    "POS2": [342.82, 171.74, 395.16, 26.39, -176.26, -1.66],
    "HAND_SCAN": [406.06, -167.10, 555.08, 95.52, -104.05, 24.04],
    "TARGET_SCAN": [684.13, -25.36, 235.69, 169.25, -140.48, 159.67],
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


class TargetScanNode(Node):

    def __init__(self):
        super().__init__("target_scan_node")

        self.target_name = None

        self.find_order_client = ActionClient(
            self,
            FindOrder,
            "/find_order",
        )
        self.scan_table_client= self.create_client(ScanRequest, "yolo_scan_request", self.table_scan)

        self.cone_scanner = ConeScanner(
            node=self,
            action_client=self.find_order_client,
            scan_tilt_angle=SCAN_TILT_ANGLE,
            scan_point_count=SCAN_POINT_COUNT,
            scan_velocity=SCAN_VELOCITY,
            scan_acceleration=SCAN_ACCELERATION,
        )
        self.scan_thread = None

        self.subscription = self.create_subscription(
            String,
            "/target_command",
            self.command_callback,
            10,
        )

        self.get_logger().info("target_scan_node 시작")
    def run_cone_scan(
        self,
        center_pose,
        target_name,
    ):
        coordinate = self.cone_scanner.scan(
            center_pose,
            target_name,
        )

        if coordinate is None:
            self.get_logger().warning(
                f"{target_name} 좌표 탐색 실패"
            )
            return

        self.get_logger().info(
            f"{target_name} 최종 좌표: {coordinate}"
        )

    def command_callback(self, msg):
        target_name = msg.data.strip()

        if not target_name:
            self.get_logger().warning(
                "빈 메시지를 받았습니다."
            )
            return

        if (
            self.scan_thread is not None
            and self.scan_thread.is_alive()
        ):
            self.get_logger().warning(
                "이미 스캔이 진행 중입니다."
            )
            return

        self.target_name = target_name

        if target_name == "hand":
            center_pose = CONFIG["HAND_SCAN"]
        else:
            center_pose = CONFIG["HAND_SCAN"]

        self.get_logger().info(
            f"스캔 명령 수신: {target_name}"
        )

        # 구독 콜백은 바로 끝내고,
        # 실제 스캔은 별도 스레드에서 실행한다.
        self.scan_thread = threading.Thread(
            target=self.run_cone_scan,
            args=(center_pose, target_name),
            daemon=True,
        )
        self.scan_thread.start()

    def move_to_position(self, position_key):
        if position_key not in CONFIG:
            self.get_logger().warn(f"등록되지 않은 위치 키입니다: {position_key}")
            return

        # 키를 입력해서 실제 좌표 값을 가져온다.
        target_position = CONFIG[position_key]

        self.get_logger().info(f"위치 키: {position_key}")
        self.get_logger().info(f"이동 좌표: {target_position}")

        movel(target_position, vel=VELOCITY, acc=ACCELERATION,)

        self.get_logger().info(f"{position_key} 이동 완료")

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