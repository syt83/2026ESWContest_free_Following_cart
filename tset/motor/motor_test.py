import serial
import time

ARDUINO_PORT = '/dev/ttyACM0'  # ACM0 지정
BAUDRATE = 9600

def run_sequence():
    try:
        print(f"1. 아두이노 연결 중... ({ARDUINO_PORT})")
        ser = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=1)
        time.sleep(2.0) # 아두이노 부팅 대기
        print("★ 연결 성공!\n")

        # 1. 전진 (1초)
        #print("🟢 [1/4] 전진 (F) - 1초")
        #ser.write(b'R')
        #time.sleep(2.0)

        #print("🛑 [완료] 정지 (S)")
        #ser.write(b'S')
        #time.sleep(0.5)
        
        # 2. 후진 (1초)
        ##print("🔴 [2/4] 후진 (B) - 1초")
        #ser.write(b'L')
        #time.sleep(2.0)

        # 3. 좌회전 (1초)
        #print("🟡 [3/4] 좌회전 (L) - 1초")
        #ser.write(b'B')
        #time.sleep(1.0)

        # 4. 우회전 (1초)
        print("🔵 [4/4] 우회전 (R) - 1초")
        ser.write(b'F')
        time.sleep(1.0)

        # 5. 정지
        print("🛑 [완료] 정지 (S)")
        ser.write(b'S')
        time.sleep(0.5)

        print("\n모든 순차 동작 테스트가 끝났습니다.")

    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.write(b'S')
            ser.close()
            print("시리얼 포트를 안전하게 닫았습니다.")

if __name__ == '__main__':
    run_sequence()