import time
import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont

# I2C 설정
i2c = board.I2C()
WIDTH = 128
HEIGHT = 64

# 디스플레이 객체 생성
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C)

# 화면 초기화
oled.fill(0)
oled.show()

# Pillow 이미지 및 드로잉 객체 생성 (1비트 모드)
image = Image.new("1", (WIDTH, HEIGHT))
draw = ImageDraw.Draw(image)

# 시스템 기본 폰트 로드
font = ImageFont.load_default()

print("SSD1306 OLED 테스트 시작 (Pillow 버전)!")

# 1. 텍스트 그리기
draw.text((0, 0), "SSD1306 Test!", fill=255, font=font)
draw.text((0, 16), "Hello, Python!", fill=255, font=font)

# 이미지 전송 및 화면 출력
oled.image(image)
oled.show()
time.sleep(2)

# 2. 반복문 테스트
while True:
    draw.rectangle((0, 0, WIDTH, HEIGHT), outline=0, fill=0) # 화면 지우기
    draw.rectangle((0, 0, WIDTH - 1, HEIGHT - 1), outline=255, fill=0) # 테두리
    draw.text((30, 25), "Running...", fill=255, font=font)
    
    oled.image(image)
    oled.show()
    time.sleep(1)
