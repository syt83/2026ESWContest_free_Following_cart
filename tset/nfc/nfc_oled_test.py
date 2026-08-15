#특정카드로 nfc에 태그하면 디스플레이가 on이 되고 그 카드를 동일하게 댈시 off가 되는 코드
import time
import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

# 1. I2C 및 OLED 디스플레이 초기화 (주소 0x3C / 0x3D 자동 감지)
i2c = busio.I2C(board.SCL, board.SDA)

oled = None
for addr in [0x3C, 0x3D]:
    try:
        oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=addr)
        print(f"✅ OLED 디스플레이 연결 성공! (I2C 주소: 0x{addr:02X})")
        break
    except (ValueError, OSError):
        continue

if oled is None:
    print("❌ [오류] OLED 디스플레이를 찾을 수 없습니다.")
    print("   1. 'sudo i2cdetect -y 1' 명령어로 I2C 연결을 확인하세요.")
    print("   2. SDA/SCL 점퍼 와이어 헐거움 및 VCC/GND 전원을 점검하세요.")
    exit(1)

# 2. NFC 리더기 초기화
reader = SimpleMFRC522()

# 3. 그래픽 버퍼 생성
WIDTH, HEIGHT = oled.width, oled.height
image = Image.new("1", (WIDTH, HEIGHT))
draw = ImageDraw.Draw(image)

font = ImageFont.load_default()

# 로봇/장치 상태 및 등록된 카드 ID 저장 변수
robot_state = "IDLE"
authorized_card_id = None  # ON 시킨 카드의 ID를 저장할 변수

def update_oled_display(state):
    """상태에 따라 OLED 화면을 끄고 켜는 함수"""
    draw.rectangle((0, 0, WIDTH, HEIGHT), outline=0, fill=0)
    
    if state == "IDLE":
        # IDLE 상태: 화면 완전히 끄기 (OFF)
        oled.fill(0)
        oled.show()
        print("[OLED] 화면 OFF (IDLE 대기 중)")

    elif state == "ACTIVE":
        # ACTIVE 상태: 화면 확 켜기 (ON)
        draw.rectangle((0, 0, WIDTH - 1, HEIGHT - 1), outline=255, fill=0)
        draw.text((15, 12), "SYSTEM ACTIVE!", font=font, fill=255)
        draw.text((25, 30), "Robot Running", font=font, fill=255)
        draw.text((8, 48), "Tag SAME Card to OFF", font=font, fill=255)
        
        oled.image(image)
        oled.show()
        print("[OLED] 화면 ON (SYSTEM ACTIVE!)")

# 초기 상태 적용 (OFF)
update_oled_display("IDLE")

print("=" * 50)
print(" 🤖 스마트 보안 NFC + SSD1306 OLED 시스템 시작")
print("=" * 50)

try:
    while True:
        if robot_state == "IDLE":
            # 1. IDLE 상태: 어떤 카드든 대면 ON 상태로 전환
            print("\n[IDLE 대기 중...] 카드를 대면 시스템이 켜집니다.")
            id, text = reader.read()
            
            authorized_card_id = id  # ★ 현재 카드의 ID를 등록/기억
            robot_state = "ACTIVE"
            
            print(f"\n[NFC 감지 성공!] 등록된 Tag ID: {authorized_card_id}")
            print(">>> [상태 변경] IDLE(OFF) -> ACTIVE(ON) <<<")
            
            update_oled_display(robot_state)
            time.sleep(2)  # 중복 인식 방지 대기

        elif robot_state == "ACTIVE":
            # 2. ACTIVE 상태: 비동기로 카드 인식 체크
            id, text = reader.read_no_block()
            
            if id is not None:
                # ★ 대어진 카드가 처음에 켰던 카드와 동일한지 검증
                if id == authorized_card_id:
                    print(f"\n[NFC 인증 성공!] 일치하는 Tag ID: {id}")
                    print(">>> [상태 변경] ACTIVE(ON) -> IDLE(OFF) <<<")
                    
                    robot_state = "IDLE"
                    authorized_card_id = None  # 저장된 카드 ID 초기화
                    update_oled_display(robot_state)
                    time.sleep(2)
                else:
                    print(f"\n[NFC 경고!] 등록되지 않은 다른 카드가 감지되었습니다. (Tag ID: {id})")
                    print(">>> OFF 거부: 동일한 카드를 대어야 꺼집니다. <<<")
                    time.sleep(1.5)  # 경고 문구 출력 후 짧은 대기
            
            time.sleep(0.3)

except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")
    oled.fill(0)
    oled.show()
finally:
    GPIO.cleanup()