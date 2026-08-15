#실행하고 싶으면 /home/pi/robot_project/venv/bin/python3 /home/pi/robot_project/stella_subscriber.py입력
# ros2 launch ydlidar_ros2_driver ydlidar_launch.py port:=/dev/ttyUSB1
import time
import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class StellaIntegratedNode(Node):
    def __init__(self):
        super().__init__('stella_integrated_node')
        print("Node 생성 완료")
        print(self.get_name())
        print(self.get_namespace())
        self.oled = None
        self.reader = None
        self.draw = None

        # =========================================================
        # 1. 하드웨어 (OLED & NFC) 초기화 및 예외 처리
        # =========================================================
        try:
            print("\n========== 하드웨어 초기화 시작 ==========")

            # 1. I2C 초기화
            print("① I2C 초기화 시작")
            i2c = busio.I2C(board.SCL, board.SDA)
            print("① I2C 초기화 성공")

            # 2. OLED 초기화
            print("② OLED 초기화 시작")
            self.oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)
            print("② OLED 초기화 성공")

            # 3. NFC 초기화
            print("③ NFC 초기화 시작")
            self.reader = SimpleMFRC522()
            print("③ NFC 초기화 성공")

            # 4. 이미지 객체 생성
            self.WIDTH, self.HEIGHT = self.oled.width, self.oled.height
            self.image = Image.new("1", (self.WIDTH, self.HEIGHT))
            self.draw = ImageDraw.Draw(self.image)
            self.font = ImageFont.load_default()

            self.get_logger().info("✅ NFC 리더기 및 OLED 디스플레이 하드웨어 연결 성공!")

        except Exception as e:
            import traceback
            traceback.print_exc()

            self.get_logger().error(f'❌ 하드웨어 초기화 실패: {e}')

        # =========================================================
        # 2. 로봇 상태 및 보안 카드 변수
        # =========================================================
        self.robot_state = "IDLE"          # "IDLE" (OFF) / "ACTIVE" (ON)
        self.authorized_card_id = None     # ON 시킨 카드의 ID 저장
        self.last_tag_time = 0.0           # 태그 중복 인식 방지 타이머

        # =========================================================
        # 3. UWB 보정 및 디바운스 알고리즘 변수
        # =========================================================
        self.OFFSET_CM = 18.0              # 영점 보정 오프셋 (cm)
        self.stable_distance = 0.0         # 확정된 보정 거리
        self.candidate_distance = 0.0      # 후보 거리
        self.debounce_counter = 0          # 디바운스 카운터
        self.THRESHOLD = 15.0              # 임계값 (cm)
        self.DEBOUNCE_LIMIT = 4            # 연속 반영 제한 횟수

        # =========================================================
        # 4. ROS 2 통신 설정 (Subscriber & Publisher)
        # =========================================================
        self.subscription = self.create_subscription(
            Point,
            '/uwb/data',
            self.uwb_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 0.1초 마다 NFC 태그 여부를 실시간으로 감시하는 타이머
        if self.reader is not None:
            self.nfc_timer = self.create_timer(0.1, self.check_nfc_loop)

        # 초기 OLED 상태 적용 (IDLE / OFF)
        self.update_oled_display("IDLE")

        self.get_logger().info('====================================================')
        self.get_logger().info(' 🤖 NFC + OLED + UWB 통합 스텔라 제어 노드 시작!')
        self.get_logger().info(' 🔒 [IDLE 상태] NFC 카드를 대면 시스템이 켜집니다.')
        self.get_logger().info('====================================================')

    # -------------------------------------------------------------
    # OLED 화면 업데이트 함수 (하드웨어 연결 미완료 시 세이프가드 적용)
    # -------------------------------------------------------------
    def update_oled_display(self, state):
        if self.oled is None or self.draw is None:
            return

        try:
            self.draw.rectangle((0, 0, self.WIDTH, self.HEIGHT), outline=0, fill=0)
            
            if state == "IDLE":
                self.oled.fill(0)
                self.oled.show()
                self.get_logger().info('[OLED] 화면 OFF (IDLE 대기 중)')

            elif state == "ACTIVE":
                self.draw.rectangle((0, 0, self.WIDTH - 1, self.HEIGHT - 1), outline=255, fill=0)
                self.draw.text((15, 12), "SYSTEM ACTIVE!", font=self.font, fill=255)
                self.draw.text((25, 30), "Robot Running", font=self.font, fill=255)
                self.draw.text((8, 48), "Tag SAME Card to OFF", font=self.font, fill=255)
                
                self.oled.image(self.image)
                self.oled.show()
                self.get_logger().info('[OLED] 화면 ON (SYSTEM ACTIVE!)')
        except Exception as e:
            self.get_logger().error(f'OLED 화면 업데이트 실패: {e}')

    # -------------------------------------------------------------
    # NFC 태그 실시간 루프 (비동기 처리)
    # -------------------------------------------------------------
    def check_nfc_loop(self):
        if self.reader is None:
            return

        # 연속 중복 태그 방지 (2초 쿨타임)
        if time.time() - self.last_tag_time < 2.0:
            return

        try:
            card_id, _ = self.reader.read_no_block()

            if card_id is not None:
                # [상태 1: IDLE(OFF) 상태에서 카드를 댔을 때 -> ON으로 전환]
                if self.robot_state == "IDLE":
                    self.authorized_card_id = card_id
                    self.robot_state = "ACTIVE"
                    self.last_tag_time = time.time()
                    
                    print(f"\n[🔓 NFC 인증 성공!] 등록된 Tag ID: {card_id}")
                    print(">>> [상태 변경] IDLE(OFF) ➔ ACTIVE(ON) <<<")
                    self.update_oled_display("ACTIVE")

                # [상태 2: ACTIVE(ON) 상태에서 카드를 댔을 때]
                elif self.robot_state == "ACTIVE":
                    if card_id == self.authorized_card_id:
                        print(f"\n[🔒 NFC OFF 인증 성공!] Tag ID: {card_id}")
                        print(">>> [상태 변경] ACTIVE(ON) ➔ IDLE(OFF) <<<")
                        
                        self.robot_state = "IDLE"
                        self.authorized_card_id = None
                        self.last_tag_time = time.time()
                        self.update_oled_display("IDLE")
                        
                        self.cmd_vel_pub.publish(Twist())
                    else:
                        print(f"\n[⚠️ NFC 경고] 등록되지 않은 다른 카드가 감지되었습니다. (Tag ID: {card_id})")
                        print(">>> 동일한 카드를 대어야 시스템이 꺼집니다. <<<")
                        self.last_tag_time = time.time()
        except Exception as e:
            self.get_logger().error(f'NFC 읽기 중 오류 발생: {e}')

    # -------------------------------------------------------------
    # UWB 수신 및 로봇 이동 제어 콜백
    # -------------------------------------------------------------
    def uwb_callback(self, msg):

        twist = Twist()

        # [IDLE / OFF 상태일 때: UWB 수신되더라도 모터 완전 정지]
        if self.robot_state != "ACTIVE":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # [ACTIVE / ON 상태일 때: UWB 오프셋 및 디바운스 계산]
        raw_dist = msg.x
        azimuth_deg = msg.y

        if raw_dist > 500 or raw_dist <= 0:
            return

        # 1) 영점 보정 (-18cm)
        calibrated_dist = max(0.0, raw_dist - self.OFFSET_CM)

        # 2) 베릴로그 디바운스 처리
        diff = abs(calibrated_dist - self.stable_distance)

        if diff >= self.THRESHOLD:
            if abs(calibrated_dist - self.candidate_distance) < 10.0:
                self.debounce_counter += 1
            else:
                self.candidate_distance = calibrated_dist
                self.debounce_counter = 1

            if self.debounce_counter >= self.DEBOUNCE_LIMIT:
                self.stable_distance = self.candidate_distance
                self.debounce_counter = 0
        else:
            self.debounce_counter = 0
            self.candidate_distance = self.stable_distance

        # 3) 거리별 판단
        dist = self.stable_distance
        status_msg = ""

        if dist < 25:
            twist.linear.x = 0.0
            status_msg = f"🛑 [정지] 목적지에 도착했습니다. ({dist:.0f}cm)"
        elif dist < 100:
            twist.linear.x = 0.2
            status_msg = f"🚶 [천천히 전진] 추적 중... ({dist:.0f}cm)"
        else:
            twist.linear.x = 0.4
            status_msg = f"🏃 [빠른 전진] 추적 중... ({dist:.0f}cm)"

        # 방향 판단 (추후 확장용)
        if azimuth_deg < -15:
            twist.angular.z = 0.3
            status_msg += " | 👈 [좌회전]"
        elif azimuth_deg > 15:
            twist.angular.z = -0.3
            status_msg += " | 👉 [우회전]"
        else:
            status_msg += " | ⬆️ [직진]"

        self.cmd_vel_pub.publish(twist)
        print(f"[ON 구동 중] Raw: {raw_dist:3.0f}cm | 보정: {calibrated_dist:3.0f}cm | 확정: {dist:3.0f}cm ➔ {status_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = StellaIntegratedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.oled is not None:
            node.oled.fill(0)
            node.oled.show()
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
