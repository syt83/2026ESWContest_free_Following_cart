import time
import subprocess
import signal
import sys
import os

# OLED 디스플레이 라이브러리
import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont

# NFC RC522 라이브러리
from mfrc522 import SimpleMFRC522

class RobotControlSystem:
    def __init__(self):
        # 1. I2C & OLED 초기화 (0x3C 및 0x3D 주소 자동 감지)
        self.oled = None
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            try:
                self.oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)
            except Exception:
                self.oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3d)
            
            self.oled.fill(0)
            self.oled.show()
            print("✅ SSD1306 OLED 디스플레이 초기화 완료")
        except Exception as e:
            print(f"⚠️ OLED 연결 실패 (디스플레이 없이 동작합니다): {e}")

        # 2. NFC RC522 초기화
        try:
            self.reader = SimpleMFRC522()
            print("✅ RFID-RC522 NFC 리더기 초기화 완료")
        except Exception as e:
            print(f"❌ RC522 연결 실패 (SPI 설정 확인 필요): {e}")
            self.reader = None

        self.is_running = False
        self.processes = []
        self.font_large = ImageFont.load_default()

        # 3. ROS 2 및 프로젝트 환경 설정 명령어
        # 라이다 워크스페이스(ros2_ws) 및 venv 환경을 자동으로 가져옵니다.
        self.ros_setup_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi && "
            "source ~/robot_project/venv/bin/activate"
        )

    def update_display(self, title_text, status_text, sub_text=""):
        """OLED 화면에 상태 텍스트 출력"""
        if not self.oled:
            return

        try:
            image = Image.new("1", (self.oled.width, self.oled.height))
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, self.oled.width - 1, self.oled.height - 1), outline=255, fill=0)

            draw.text((8, 8), title_text, font=self.font_large, fill=255)
            draw.text((8, 26), f"> {status_text}", font=self.font_large, fill=255)
            if sub_text:
                draw.text((8, 44), sub_text, font=self.font_large, fill=255)

            self.oled.image(image)
            self.oled.show()
        except Exception:
            pass

    def start_robot_nodes(self):
        """NFC 카드가 인식되었을 때 모든 자율주행 관련 노드 가동"""
        print("\n🚀 [START] 로봇 자율주행 시스템을 가동합니다!")
        self.update_display("ROBOT SYSTEM", "STATUS: RUNNING", "Tag NFC to STOP")

        try:
            # 1. 라이다(LiDAR) 노드 실행
            # (로그 폭주로 인한 CPU 과부하 및 라이다 속도 저하를 막기 위해 stdout/stderr는 DEVNULL 처리)
            print("  - 라이다(LiDAR) 실행 중...")
            cmd_lidar = f"{self.ros_setup_cmd} && ros2 launch sllidar_ros2 sllidar_a1_launch.py"
            p_lidar = subprocess.Popen(
                cmd_lidar, 
                shell=True, 
                executable='/bin/bash', 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            self.processes.append(p_lidar)
            time.sleep(2)

            # 2. UWB 퍼블리셔 실행
            print("  - UWB 퍼블리셔 실행 중...")
            cmd_uwb = f"{self.ros_setup_cmd} && python3 uwb_publisher.py"
            p_uwb = subprocess.Popen(
                cmd_uwb, 
                shell=True, 
                executable='/bin/bash', 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            self.processes.append(p_uwb)
            time.sleep(1)

            # 3. UWB 추종 및 아두이노 모터 제어 노드 실행
            print("  - 아두이노 추종 제어 노드 실행 중...")
            cmd_follower = f"{self.ros_setup_cmd} && python3 uwb_follower_node.py"
            p_follower = subprocess.Popen(
                cmd_follower, 
                shell=True, 
                executable='/bin/bash'
            )
            self.processes.append(p_follower)

            self.is_running = True
            print("✅ 모든 노드 가동 완료!\n")

        except Exception as e:
            print(f"❌ 노드 실행 중 오류 발생: {e}")
            self.stop_robot_nodes()

    def stop_robot_nodes(self):
        """NFC 카드가 다시 태그되었을 때 모든 시스템 정지 및 안전 처리"""
        print("\n🛑 [STOP] 로봇 시스템을 정지하고 대기 모드로 전환합니다.")
        self.update_display("ROBOT SYSTEM", "STATUS: STOPPING", "Closing Nodes...")

        # 1. 실행 중인 백그라운드 프로세스 및 ROS 노드 일괄 정지
        try:
            subprocess.run(["killall", "-9", "python3", "sllidar_node"], stderr=subprocess.DEVNULL)
        except Exception:
            pass

        self.processes.clear()

        # 2. 아두이노에 모터 긴급 정지 명령('0,0') 전송
        try:
            subprocess.run(
                ["python3", "-c", "import serial; s=serial.Serial('/dev/ttyACM0', 9600); s.write(b'0,0\\n'); s.close()"], 
                stderr=subprocess.DEVNULL
            )
        except Exception:
            pass

        self.is_running = False
        print("💤 대기 상태 전환 완료.\n")
        self.update_display("ROBOT SYSTEM", "STATUS: STANDBY", "Tag NFC Card...")

    def run(self):
        """메인 실행 루프"""
        self.update_display("ROBOT SYSTEM", "STATUS: STANDBY", "Tag NFC Card...")

        print("==================================================")
        print("🤖 NFC 대기 시스템 가동 중... 카드를 접촉해 주세요.")
        print("==================================================")

        try:
            while True:
                if self.reader:
                    # NFC 카드 태그 감지 (Blocking)
                    id, text = self.reader.read()
                    print(f"\n[NFC 감지 성공] Card ID: {id}")

                    # 상태 토글 (대기 -> 가동 / 가동 -> 대기)
                    if not self.is_running:
                        self.start_robot_nodes()
                    else:
                        self.stop_robot_nodes()

                    # 중복 인식 방지 대기 (3초)
                    time.sleep(3)
                else:
                    time.sleep(1)

        except KeyboardInterrupt:
            print("\n프로그램을 종료합니다.")
            self.stop_robot_nodes()
            if self.oled:
                self.oled.fill(0)
                self.oled.show()

if __name__ == '__main__':
    system = RobotControlSystem()
    system.run()