import serial
import time

port = '/dev/ttyUSB0'

try:
    ser = serial.Serial(port, 9600, timeout=1)
    print(f"✅ {port} 연결 성공! UWB 깨우기 명령어 전송 중...")

    # Qorvo 모듈 리셋 및 스트리밍 시작 명령어
    ser.write(b'\r\n\r\n')
    time.sleep(0.3)
    ser.write(b'lec\r\n')
    time.sleep(0.1)

    print("=" * 50)
    print("수신 시작 (데이터가 나오는지 확인하세요 / 종료: Ctrl+C)")
    print("=" * 50)

    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print(f"[RAW]: {data}")

except Exception as e:
    print(f"❌ 에러 발생: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
