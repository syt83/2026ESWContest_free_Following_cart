import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

# RC522 Version Register(0x37) 읽기 테스트 (주소: 0x37 << 1 | 0x80 = 0x6E)
version = spi.xfer2([0x6E, 0x00])[1]

print("=" * 50)
print(f"🔍 RC522 레지스터 응답 값: {hex(version)}")
print("=" * 50)

if version in [0x91, 0x92]:
    print("✅ [통신 성공] 라즈베리 파이와 RC522가 정상적으로 연결되어 있습니다.")
    print("   👉 이 상태에서 태그가 찍히지 않는다면 '전률/전압 부족(배터리)' 문제입니다.")
elif version in [0x00, 0xFF]:
    print("❌ [통신 실패] 전원은 들어왔으나 통신 라인이 연결되지 않았습니다.")
    print("   👉 MISO(21), MOSI(19), SCK(23), RST(22), SDA(24) 핀 배선을 재확인하세요.")
else:
    print(f"⚠️ [신호 불안정] 알 수 없는 응답 값입니다 ({hex(version)}).")
    print("   👉 점퍼선이 헐겁거나 핀 접촉 불량일 가능성이 높습니다.")

spi.close()
