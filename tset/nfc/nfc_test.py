import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import time

reader = SimpleMFRC522()

# 로봇/장치 상태 (IDLE: 대기 모드, ACTIVE: 작동/주행 모드)
robot_state = "IDLE"

print("=" * 50)
print(" 🤖 NFC 대기 및 상태 전환 시스템 시작")
print(" 현재 상태: [IDLE] - NFC 태그를 기다리는 중...")
print("=" * 50)

try:
    while True:
        if robot_state == "IDLE":
            print("\n[IDLE 대기 중...] NFC 카드가 감지되면 시스템을 활성화합니다.")

            # NFC 태그가 닿을 때까지 대기
            id, text = reader.read()

            print(f"\n[NFC 감지 성공!] Tag ID: {id}")
            print(">>> [상태 변경] 대기 모드(IDLE) -> 활성화 모드(ACTIVE) <<<")
            robot_state = "ACTIVE"

            time.sleep(2) # 중복 인식 방지 대기

        elif robot_state == "ACTIVE":
            print("\n--------------------------------------------------")
            print(" 🚀 [ACTIVE 모드] 시스템 작동 중 (UWB 추종 / 모터 제어 등)")
            print(" (NFC를 다시 대면 대기 모드로 돌아갑니다)")
            print("--------------------------------------------------")

            # 비동기적으로 NFC 태그가 들어오는지 확인 (non-blocking)
            id, text = reader.read_no_block()

            if id is not None:
                print(f"\n[NFC 감지!] Tag ID: {id}")
                print(">>> [상태 변경] 활성화 모드(ACTIVE) -> 대기 모드(IDLE) <<<")
                robot_state = "IDLE"
                time.sleep(2)

            time.sleep(0.5)

except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")
finally:
    GPIO.cleanup()
