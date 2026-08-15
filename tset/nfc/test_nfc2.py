#nfc테스트

import rclpy
from rclpy.node import Node
import time
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

# 💡 내용이 비어있는 클래스에는 반드시 'pass'를 넣어주어야 에러가 나지 않습니다.
class UwbImuFollowerMaster(Node):
    pass 

def main(args=None):
    RELAY_PIN = 21
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    
    # ❌ 카드를 대기 전: 대기 상태
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print("====================================")
    print("💤 시스템 대기 중 (WAIT MODE)")
    print("NFC 카드를 태그하면 로봇이 깨어납니다...")
    print("====================================")

    reader = SimpleMFRC522()
    try:
        card_id, text = reader.read()
        print(f"\n✅ 인증 완료! (Card ID: {card_id})")
        
        # 🟢 카드를 댄 후: 전원 ON
        print("⚡ 라이다 및 시스템 전원 ON!")
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        
        time.sleep(3)
        print("🚀 로봇 주행 알고리즘을 시작합니다! (테스트 완료)")

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.output(RELAY_PIN, GPIO.LOW)
        GPIO.cleanup()

if __name__ == '__main__':
    main()