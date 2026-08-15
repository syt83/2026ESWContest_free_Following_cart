import serial
import time

ARDUINO_PORT = '/dev/ttyACM0'
BAUDRATE = 9600

ser = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=1)

time.sleep(2)

print("Arduino 연결 성공")
print("F: 전진")
print("B: 후진")
print("L: 좌회전")
print("R: 우회전")
print("1: 왼쪽 모터")
print("2: 오른쪽 모터")
print("S: 정지")
print("Q: 종료")

try:
    while True:
        command = input("\n명령 입력: ").strip().upper()

        if command == 'Q':
            ser.write(b'S')
            ser.flush()
            break

        if command in ['F', 'B', 'L', 'R', 'S', '1', '2']:
            ser.write(command.encode())
            ser.flush()
            print(f"'{command}' 전송")
        else:
            print("잘못된 명령입니다.")

finally:
    ser.write(b'S')
    ser.flush()
    time.sleep(0.2)
    ser.close()
    print("Arduino 연결 종료")