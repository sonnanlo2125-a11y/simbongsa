# cone_scan.py

import math
import threading
import time

import rclpy

from hey_doopal_msg.action import FindOrder


class ConeScanner:

    def __init__(
        self,
        node,
        action_client,
        
        scan_tilt_angle=30.0,
        scan_point_count=8,
        scan_velocity=None,
        scan_acceleration=None,
    ):
        self.node = node
        self.action_client = action_client

        self.scan_tilt_angle = scan_tilt_angle
        self.scan_point_count = scan_point_count

        self.scan_velocity = (
            scan_velocity
            if scan_velocity is not None
            else [20, 10]
        )

        self.scan_acceleration = (
            scan_acceleration
            if scan_acceleration is not None
            else [40, 20]
        )

    def scan(self, center_pose, target_name):
        from DSR_ROBOT2 import (
            DR_BASE,
            amovel,
            check_motion,
            movel,
        )
        from DR_common2 import posx

        node = self.node

        result_event = threading.Event()
        goal_event = threading.Event()

        result_data = {
            "accepted": False,
            "found": False,
            "coordinate": None,
            "goal_handle": None,
        }

        def feedback_callback(feedback_msg):
            node.get_logger().info(f"YOLO 상태: {feedback_msg.feedback.state}")

        def result_callback(future):
            try:
                response = future.result()
                result = response.result

                result_data["found"] = result.found

                if result.found:
                    result_data["coordinate"] = list(
                        result.coordinate
                    )

                result_data["message"] = result.message

                node.get_logger().info(
                    f"YOLO 결과: found={result.found}, "
                    f"message={result.message}"
                )

            except Exception as error:
                result_data["found"] = False
                result_data["coordinate"] = None
                result_data["message"] = str(error)

                node.get_logger().error(
                    f"YOLO 결과 처리 오류: {error}"
                )

            finally:
                result_event.set()

        def goal_callback(future):
            try:
                goal_handle = future.result()

                result_data["goal_handle"] = goal_handle
                result_data["accepted"] = (
                    goal_handle.accepted
                )

                if goal_handle.accepted:
                    node.get_logger().info(
                        f"탐색 요청 승인: {target_name}"
                    )

                    result_future = (
                        goal_handle.get_result_async()
                    )
                    result_future.add_done_callback(
                        result_callback
                    )
                else:
                    node.get_logger().warning(
                        "탐색 요청이 거절되었습니다."
                    )

            except Exception as error:
                node.get_logger().error(
                    f"Goal 처리 오류: {error}"
                )

            finally:
                goal_event.set()

        if center_pose is None or len(center_pose) != 6:
            node.get_logger().error(
                "스캔 중심 좌표가 올바르지 않습니다."
            )
            return None

        center_pose = list(center_pose)
        center_pos = posx(*center_pose)

        scan_poses = []

        for index in range(self.scan_point_count):
            theta = (
                2.0
                * math.pi
                * index
                / self.scan_point_count
            )

            tilt_rx = (
                self.scan_tilt_angle
                * math.cos(theta)
            )
            tilt_ry = (
                self.scan_tilt_angle
                * math.sin(theta)
            )

            pose = [
                center_pose[0],
                center_pose[1],
                center_pose[2],
                center_pose[3] + tilt_rx,
                center_pose[4] + tilt_ry,
                center_pose[5],
            ]

            scan_poses.append(posx(*pose))

        # 중심 위치로 이동
        movel(
            center_pos,
            vel=self.scan_velocity,
            acc=self.scan_acceleration,
            ref=DR_BASE,
        )

        # YOLO Action Server 확인
        if not self.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            node.get_logger().error(
                "/find_order 서버가 없습니다."
            )
            return None

        goal = FindOrder.Goal()
        goal.target_name = target_name

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )
        future.add_done_callback(goal_callback)

        if not goal_event.wait(timeout=5.0):
            node.get_logger().error(
                "YOLO Goal 응답 시간 초과"
            )
            return None

        if not result_data["accepted"]:
            return None

        node.get_logger().info(
            f"원뿔 스캔 시작: {target_name}"
        )

        for index, scan_pose in enumerate(scan_poses):
            amovel(
                scan_pose,
                vel=self.scan_velocity,
                acc=self.scan_acceleration,
                ref=DR_BASE,
            )

            while rclpy.ok():
                if result_event.is_set():

                    if result_data["found"]:
                        return result_data["coordinate"]

                    return None

                if check_motion() == 0:
                    break

                time.sleep(0.03)

            node.get_logger().info(
                f"scan_pose[{index}] 이동 완료"
            )

        # 모든 스캔이 끝났는데 결과가 안 왔으면 취소
        goal_handle = result_data["goal_handle"]

        if (
            goal_handle is not None
            and not result_event.is_set()
        ):
            goal_handle.cancel_goal_async()

        movel(
            center_pos,
            vel=self.scan_velocity,
            acc=self.scan_acceleration,
            ref=DR_BASE,
        )

        return None