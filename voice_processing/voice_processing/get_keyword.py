import os
import rclpy
import pyaudio
import time
import json # [ADD] UI 전송용 JSON 포맷을 위한 모듈 추가
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool # [ADD] TTS 연동을 위한 메시지 타입

from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate

# ★ 우리가 정의한 커스텀 메시지 import (Trigger 대신 사용)
from hey_doopal_msg.srv import VoiceKeyword
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")
        self.is_busy = False
        # [ADD] TTS 퍼블리셔 추가
        self.say_pub = self.create_publisher(String, '/say', 10)

        # [ADD] QoS 설정 (로봇 제어 노드와 통신을 확실하게 하기 위함)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # [ADD] UI로 채팅 내용을 전달하기 위한 퍼블리셔 추가
        self.chat_pub = self.create_publisher(String, '/ui_chat_log', qos_profile)

        # [수정] 상태 수신 (변수 할당하여 가비지 컬렉션 방지)
        self.sub_status = self.create_subscription(String, '/robot_status', self.status_callback, qos_profile)

        # [수정] 기존 구독자들 (전부 변수에 저장하여 메모리 유지)
        self.sub_table_scan = self.create_subscription(Bool, '/table_scan_finished', self.table_scan_finished_cb, qos_profile)
        self.sub_hand_start = self.create_subscription(Bool, '/hand_scan_start', self.hand_scan_start_cb, qos_profile)
        self.sub_hand_finish = self.create_subscription(Bool, '/hand_scan_finished', self.hand_scan_finished_cb, qos_profile)
        self.sub_task_completed = self.create_subscription(Bool, '/task_completed', self.task_completed_cb, qos_profile)
        self.sub_table_rescan_start = self.create_subscription(Bool, '/table_rescan_started', self.table_rescan_started_cb, qos_profile)
        self.sub_table_rescan_finish = self.create_subscription(Bool, '/table_rescan_finished', self.table_rescan_finished_cb, qos_profile)
        self.sub_hand_tracking = self.create_subscription(Bool, '/hand_tracking_request', self.hand_tracking_request_cb, qos_profile)
        self.sub_hand_arrived = self.create_subscription(Bool, '/hand_arrived', self.hand_arrived_cb, qos_profile)
        self.sub_error_status = self.create_subscription(String, '/robot_error_status', self.error_status_cb, qos_profile)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )
        # 프롬프트는 기존 그대로 유지
        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - airpods, cable, drink, mouse, pos1, pos2, pos3, hand

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 pos3 hand]
            - 도구와 위치는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "여기" → hand, "손" → hand )는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 목적지가 명시되지 않은 경우, 기본 목적지는 'hand'로 간주합니다.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.
            - 손은 도구에 들어갈 수 없습니다.
            - "나왔어","나 왔어", "왔어"를 감지했다면 목적지를 'table_scan'으로 입력하세요.

            <예시>
            - 입력: "에어팟을 손에 가져다 놔"  
            출력: airpods / hand

            - 입력: "왼쪽에 있는 에어팟과 케이블을 포즈원에 넣어줘"  
            출력: airpods cable / pos1

            - 입력: "왼쪽에 있는 마우스를줘"  
            출력: mouse /

            - 입력: "왼쪽에 있는 드링크를 줘"  
            출력: drink /

            - 입력: "airpods는 pos2에 두고 cable는 pos1에 둬"  
            출력: airpods cable / pos2 pos1

            <사용자 입력>
            "{user_input}"              
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=openai_api_key)

        # 오디오 설정
        mic_config = MicConfig(
            chunk=3840,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            # device_index=3,
            buffer_size=3840,
        )
        self.mic_controller = MicController(config=mic_config)

        self.mic_controller.open_stream()
        self.wakeup_word = WakeupWord(mic_config.buffer_size)
        self.wakeup_word.set_stream(self.mic_controller.stream)

        # 현재 노드의 모드 관리 ("WAKEUP": 최초 시동어 대기, "CONFIRM": "받았어" 대기, "BUSY": 로봇 동작 중)
        self.current_mode = "WAKEUP"
        self.last_goal = "" # [추가] 마지막 목적지 저장
        self.last_command_time = 0 
        self.cooldown_duration = 5
        self.is_speaking = False # [ADD] 로봇이 말하고 있는지 확인하는 상태 변수
        self.speech_end_time = 0 # [ADD] 발화 종료 시간 기록

        # 1. 로봇 제어 노드로부터 모드 변경 명령을 받는 서비스 서버
        self.mode_srv = self.create_service(
            VoiceKeyword, "set_voice_mode_service", self.handle_mode_change
        )

        # 2. 분석된 키워드(target, goal) 또는 "받았어" 트리거를 로봇 제어 노드로 쏘기 위한 클라이언트
        self.client = self.create_client(VoiceKeyword, "/get_keyword")
        self.receive_client = self.create_client(Trigger, "/ungrip")

        self.get_logger().info("GetKeyword Node initialized with Mode Control.")
        self.last_wakeup_switch_time = 0
        # 타이머를 통해 메인 루프 실행 (spin과 병행하기 위함)
        self.timer = self.create_timer(0.1, self.main_loop)
        
    def status_callback(self, msg):
        # [수정] WAKEUP이나 CONFIRM 모드일 때는 외부 토픽에 의해 덮어씌워지지 않도록 방어막 추가
        if self.current_mode in ["WAKEUP", "CONFIRM"]:
            self.get_logger().info(f"★ 현재 상태({self.current_mode}) 유지 (로봇 상태 토픽 무시) ★")
            return
            
        self.current_mode = msg.data
        self.get_logger().info(f"★ 상태 토픽 수신 및 모드 변경: {self.current_mode} ★")
        
    # [ADD] UI로 화자와 텍스트를 JSON 형태로 전송하는 헬퍼 함수
    def publish_chat_to_ui(self, speaker, text):
        if not text.strip():  # 빈 텍스트는 보내지 않음
            return
            
        chat_data = {
            "speaker": speaker,
            "text": text
        }
        
        msg = String()
        msg.data = json.dumps(chat_data, ensure_ascii=False)
        self.chat_pub.publish(msg)
        
    # [ADD] TTS 출력 및 발화 대기 함수
    def say(self, text, sleep_time=0.8):
        self.is_speaking = True # 말하기 시작
        
        # [ADD] 특수 명령어가 아닌 실제 대화 내용만 UI로 전송
        if text not in ["STOP_SOUND", "BEEP::", "SCAN::"]:
            self.publish_chat_to_ui("ROBOT", text)
            
        # [핵심] 로봇이 말하기 전에 귀를 완전히 막아버립니다 (스트림 정지)
        try:
            if self.mic_controller.stream.is_active():
                self.mic_controller.stream.stop_stream()
        except Exception as e:
            pass

        # TTS 출력
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)
        self.get_logger().info(f"🗣️ [TTS 출력 중]: {text} ({sleep_time}초 대기)") 
        time.sleep(sleep_time)
        
        # [핵심] 말이 끝나면 귀를 다시 엽니다 (스트림 재시작)
        try:
            if not self.mic_controller.stream.is_active():
                self.mic_controller.stream.start_stream()
        except Exception as e:
            pass

        self.flush_mic_buffer() # 혹시 모를 잔여물 비우기
        self.speech_end_time = time.time() # 발화 종료 시간 기록
        self.is_speaking = False # 말하기 끝

    def flush_mic_buffer(self):
        """마이크 버퍼에 남은 쓰레기 오디오 데이터를 강제로 비웁니다."""
        try:
            # [FIX] MacOS에서 완벽하게 비우기 위해 남아있는 프레임을 통째로 읽음
            available = self.mic_controller.stream.get_read_available()
            if available > 0:
                self.mic_controller.stream.read(available, exception_on_overflow=False)
        except Exception as e:
            # [FIX] 오버플로우로 스트림이 멈췄을 경우 강제로 껐다 켜서 복구
            try:
                self.mic_controller.stream.stop_stream()
                self.mic_controller.stream.start_stream()
            except:
                pass
            
    def handle_mode_change(self, request, response):
        """로봇 제어 노드가 모드를 바꿀 때 호출되는 서비스"""
        
        # [FIX] WAKEUP 모드로 전환되는 순간 버퍼를 싹 비움
        if self.current_mode == "WAKEUP":
            self.flush_mic_buffer()
            self.last_wakeup_switch_time = time.time()
        
        # [ADD] CONFIRM 모드 진입 시 안내 멘트와 비프음 출력
        elif self.current_mode == "CONFIRM":
            self.say("손 위치에 도달했습니다. 받을 준비가 되셨다면, '받았어'라고 말씀해주세요.", sleep_time=7.0)
            self.say("BEEP::", sleep_time=0.2)

        # [ADD] 스캔 종료 시 음악 정지 및 완료 멘트
        elif self.current_mode == "SCAN_DONE":
            self.say("STOP_SOUND", sleep_time=0.1)
            self.say("스캔이 완료되었습니다.", sleep_time=3)
            self.current_mode = "WAKEUP"
            self.last_wakeup_switch_time = time.time()

        self.get_logger().info(f"★ 모드 변경됨 -> {self.current_mode} ★")
        response.success = True
        return response

    def table_scan_finished_cb(self, msg):
        self.get_logger().info(f"★ [수신완료] table_scan_finished 토픽 수신! ★")
        if msg.data:
            self.say("STOP_SOUND", sleep_time=0.1) # 일단 음악 멈추고
            self.say("테이블 스캔이 완료되었습니다. 대기모드로 전환됩니다.", sleep_time=6)
            # 완료 후 시동어 오인식 방지를 위해 쿨타임 강제 부여
            self.current_mode = "WAKEUP"
            self.last_wakeup_switch_time = time.time()
            self.get_logger().info("★ WAKEUP 모드로 전환 완료 ★")
            
    def hand_scan_start_cb(self, msg):
        if msg.data:
            self.say("손을 보여주세요.", sleep_time=3)

    def hand_scan_finished_cb(self, msg):
        if msg.data:
            self.say("손 위치를 확인했습니다. 손을 내리고 기다려 주세요.", sleep_time=8)

    def task_completed_cb(self, msg):
        self.get_logger().info(f"★ [수신완료] task_completed 토픽 수신! ★")
        if msg.data:
            # 복귀 전 멘트 출력
            self.say("여기 있습니다. 원위치로 복귀하겠습니다.", sleep_time=6.0)
            # 복귀 후 바로 WAKEUP 모드로 진입하되, 시동어 감지 방지 쿨타임 부여
            self.last_wakeup_switch_time = time.time()
            self.current_mode = "WAKEUP"
            self.get_logger().info("★ WAKEUP 모드로 전환 완료 ★")

    def table_rescan_started_cb(self, msg):
        if msg.data:
            self.say("물건이 보이지 않아 다시 한번 찾아보겠습니다.", sleep_time=5.0)

    def table_rescan_finished_cb(self, msg):
        if msg.data:
            self.say("재스캔을 완료했습니다.", sleep_time=2.5)
            self.current_mode = "BUSY"

    def hand_tracking_request_cb(self, msg):
        if msg.data:
            self.say("물체를 집었습니다. 손을 다시 보여주세요.", sleep_time=5.0)

    def hand_arrived_cb(self, msg):
        self.get_logger().info(f"★ [디버깅] hand_arrived 토픽 수신됨! 값: {msg.data}")

        # [수정] msg.data가 True이고 목적지가 hand 혹은 빈 값일 때만 모드 변경 (BUSY일 때만)
        if msg.data and (self.last_goal == "hand" or self.last_goal == "") and self.current_mode == "BUSY":
            self.get_logger().info("손 도착 완료! CONFIRM 모드 진입.")
            self.current_mode = "CONFIRM"
            self.say("손 위치에 도달했습니다. 받을 준비가 되셨다면, 받았어라고 말씀해주세요.", sleep_time=8.0)
            self.say("BEEP::", sleep_time=0.5)

    def error_status_cb(self, msg):
        self.get_logger().error(f"★ [에러 수신] 로봇 제어 노드로부터 에러 감지: {msg.data} ★")
        
            # 진행 중이던 소리가 있다면 정지시킴
        self.say("STOP_SOUND", sleep_time=0.1)
            
            # 안내 멘트 출력 (비동기로 말하고 기다림)
        self.say("오류가 발생했습니다. 복귀모드로 전환됩니다.", sleep_time=5.0)
            
            # WAKEUP 모드로 강제 전환 및 시동어 오인식 방지를 위한 쿨타임 초기화
        self.current_mode = "WAKEUP"
        self.last_wakeup_switch_time = time.time()
        self.flush_mic_buffer()
        
        self.get_logger().info("★ 시스템 WAKEUP 모드로 강제 복귀 완료 ★")

        

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content

        # 예외 처리 안전장치 추가
        if "/" not in result:
            return [], []

        obj_str, target_str = result.strip().split("/")
        objects = obj_str.split()
        targets = target_str.split()

        print(f"llm's response: {result}")
        print(f"object: {objects}")
        print(f"target: {targets}")
        return objects, targets

    def send_to_robot(self, target_val, goal_val):
        """로봇 제어 노드로 서비스 요청 전송"""
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 제어 서버 연결 대기 중...')

        self.last_goal = goal_val # [수정] 목적지 저장
        request = VoiceKeyword.Request()
        request.target = target_val
        request.goal = goal_val

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"서버 응답 수신 성공! (Success: {response.success})")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")

    def main_loop(self):
        """모드에 따라 동작을 다르게 가져가는 메인 루프 (루프 제거, 상태 머신 방식)"""
        
        # 1. 최초 시동어("헤이 두팔") 대기 모드
        if self.current_mode == "WAKEUP":
            
            # 발화 중에는 스트림이 정지되어 있으므로 아무것도 하지 않음
            if self.is_speaking:
                return

            # 로봇이 말을 끝낸 직후 1.5초간은 스피커 진동/잔향이 남아있으므로 강제 비움
            if time.time() - self.speech_end_time < 1.5:
                self.flush_mic_buffer()
                return

            # [핵심 해결] 1.5초 ~ 3.0초 사이의 블라인드 타임: 
            # 데이터를 강제로 비우면 시동어 엔진의 오디오 파형이 끊겨서(Discontinuity) 
            # 다음번 감지 때 Confidence가 폭발하는 버그(False Positive)가 발생합니다.
            # 따라서 엔진이 정상적으로 데이터를 삼키게 하되, 결과값만 무시합니다.
            try:
                triggered = self.wakeup_word.is_wakeup()
            except Exception:
                triggered = False

            # 아직 모드 전환 후 3초가 지나지 않았다면 (엔진 안정화 기간), 설령 감지되었어도 무시
            if time.time() - self.last_wakeup_switch_time < 3.0:
                return
            
            if not triggered:
                return

            # --- 여기서부터는 진짜 시동어 감지 성공 ---
            self.get_logger().info("시동어 감지! 음성 인식을 시작합니다.")
            self.say("네. 말씀하세요.", sleep_time=3)
            self.say("BEEP::", sleep_time=0.2)
            
            # 명령 분석 직전 버퍼 초기화 및 휴지기
            self.flush_mic_buffer()
            time.sleep(1.0) 
            
            # 분석을 위해 SLEEP 모드로 전환 (다음 타이머 틱에서 STT 처리)
            self.current_mode = "SLEEP" 
            return

        # 2. 분석 모드 (WAKEUP에서 넘어옴)
        elif self.current_mode == "SLEEP":
            self.flush_mic_buffer()
            output_message = self.stt.speech2text(self.mic_controller.stream)

            # [ADD] UI로 사용자의 음성 인식 결과 전송
            self.publish_chat_to_ui("USER", output_message)

            cancel_keywords = ["아니야", "취소", "됐어", "그만", "괜찮아", "쉿", "조용히", "닥쳐", "셧업"]
            if any(keyword in output_message for keyword in cancel_keywords):
                self.get_logger().info("사용자 취소 명령 감지. 대기 모드로 돌아갑니다.")
                self.say("네, 대기할게요.", sleep_time=2.5)
                
                # WAKEUP 모드로 복귀
                self.current_mode = "WAKEUP"
                self.last_wakeup_switch_time = time.time()
                self.flush_mic_buffer()
                return
            if "범퍼카" in output_message:
                self.get_logger().info("🚨 이스터에그 '범퍼카' 감지됨!")
                # 로봇이 대사를 치고 3초 정도 충분히 기다려줍니다.
                self.say("장 용 준 처 어 러엄~", sleep_time=3.0)
                
                # 대사를 친 후 다시 "헤이 두팔"을 기다리는 모드로 복귀
                self.current_mode = "WAKEUP"
                self.last_wakeup_switch_time = time.time()
                self.flush_mic_buffer()
                return
            # "나왔어" 스캔 명령어 감지 로직
            if "나왔어" in output_message or "스캔" in output_message or "왔어" in output_message or "나 왔어" in output_message:
                self.say("알겠습니다. 테이블 스캔을 시작합니다.", sleep_time=3)
                self.say("SCAN::", sleep_time=0.2)
                self.send_to_robot("", "table_scan")
                self.current_mode = "BUSY" 
                return

            # 키워드 추출
            objects, targets = self.extract_keyword(output_message)

            if objects and targets:
                target_str = objects[0]
                goal_str = targets[0]
                self.get_logger().warn(f"추출된 도구: {target_str}, 목적지: {goal_str}")
                self.say(f"네, 알겠습니다. {target_str}를 {goal_str}로 옮기겠습니다.", sleep_time=4)
                self.send_to_robot(target_str, goal_str)
                self.current_mode = "BUSY"
            else:
                self.get_logger().info("명령을 이해하지 못했습니다.")
                self.say("다시 한 번 말씀해주시겠어요?", sleep_time=3.5)
                self.say("BEEP::", sleep_time=0.2)
                
                # 실패 시 헤이두팔 없이 바로 다시 듣기
                self.current_mode = "SLEEP" 
                return
            return

        # 3. 손 건네주기 상황에서 "받았어" 입력을 기다리는 모드
        elif self.current_mode == "CONFIRM":
            output_message = self.stt.speech2text(self.mic_controller.stream)
            
            # [ADD] UI로 사용자의 음성 인식 결과 전송
            self.publish_chat_to_ui("USER", output_message)
            
            if "받았어" in output_message or "았어" in output_message or "받" in output_message:
                self.get_logger().warn("'받았어' 키워드 검출! 그리퍼 열기(ungrip) 요청 전송")
                
                # [수정] ROS2 비동기 서비스 호출 방식으로 변경
                ungrip_request = Trigger.Request()
                future = self.receive_client.call_async(ungrip_request)
                
                # 서비스 응답 확인용 콜백 추가 (선택 사항이지만 디버깅에 매우 좋습니다)
                future.add_done_callback(
                    lambda f: self.get_logger().info(f"그리퍼 열기 응답: {f.result().success if f.result() else '실패'}")
                )
                
                self.say("네 알겠습니다.", sleep_time=1.0)
                self.current_mode = "BUSY"
            
            elif output_message.strip() != "":
                self.say("잘 들리지 않았어요. '받았어'라고 말씀해주세요.", sleep_time=5.0)
                self.say("BEEP::", sleep_time=0.2)
            return

        # BUSY 모드일 때는 아무것도 하지 않음
        elif self.current_mode == "BUSY":
            self.flush_mic_buffer()
            return
        
def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
