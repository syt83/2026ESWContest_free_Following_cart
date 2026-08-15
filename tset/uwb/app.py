import time
import threading
import serial
from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
app.config['SECRET_KEY'] = 'stella_board_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 9600

ser = None

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✅ 스텔라보드 시리얼 연결 성공: {SERIAL_PORT}")
except Exception as e:
    print(f"❌ 시리얼 포트 연결 실패: {e}")

def read_serial_thread():
    global ser
    while True:
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 'UW:' 및 'UWB:' 둘 다 허용하도록 필터링 완화
                    if line and ('UW:' in line or 'UWB:' in line):
                        print(f"📥 실시간 수신: {line}")
                        socketio.emit('uwb_response', {'data': line})
            except Exception as e:
                print(f"⚠️ 시리얼 읽기 오류: {e}")
        time.sleep(0.01)

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    thread = threading.Thread(target=read_serial_thread)
    thread.daemon = True
    thread.start()

    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
