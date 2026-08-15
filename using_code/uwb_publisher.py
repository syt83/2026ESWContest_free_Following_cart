import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time
import smbus2
import re

class UwbPublisher(Node):
    def __init__(self):
        super().__init__('uwb_publisher_node')
        self.publisher_ = self.create_publisher(Point, '/uwb/data', 10)
        self.offset_cm = 20
        
        # 💡 [핵심] 엉뚱한 라이다 센서를 버리고, 진짜 Qorvo UWB(Nano 33 BLE) 포트로 연결!
        self.port = '/dev/serial/by-id/usb-Arduino_Nano_33_BLE_302ACF640E50F7E9-if00'
        
        # 아두이노 나노 33 BLE 기반 UWB 펌웨어의 표준 통신 속도
        self.baudrate = 115200 
        self.ser = None
        
        self.IMU_ADDR = 0x68
        self.bus = None
        self.last_imu_time = time.time()
        
        self.init_imu()
        self.init_serial()
        self.buffer = ""
        self.timer = self.create_timer(0.05, self.read_and_publish)

    def init_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(0.5)
            
            # 스트리밍 명령어 (아두이노 기반 펌웨어용)
            self.ser.write(b'\r\n')
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'✅ 진짜 UWB(Nano 33 BLE) 연결 성공! ({self.baudrate}bps)')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ UWB 연결 실패: {e}')

    def init_imu(self):
        try:
            self.bus = smbus2.SMBus(1)
            self.bus.write_byte_data(self.IMU_ADDR, 0x6B, 0x00)
            self.get_logger().info('✅ 로봇 IMU 연동 성공!')
        except Exception:
            self.bus = None

    def update_imu_yaw(self):
        if self.bus is None: return 0.0
        now = time.time()
        dt = now - self.last_imu_time
        self.last_imu_time = now
        try:
            h = self.bus.read_byte_data(self.IMU_ADDR, 0x47)
            l = self.bus.read_byte_data(self.IMU_ADDR, 0x48)
            val = (h << 8) | l
            if val > 32767: val -= 65536
            rate = val / 131.0
            return rate if abs(rate) > 0.5 else 0.0
        except Exception:
            return 0.0

    def read_and_publish(self):
        gyro_rate = self.update_imu_yaw()
        if self.ser is None or not self.ser.is_open:
            self.init_serial()
            return

        try:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += raw_data

                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if not line: continue
                    
                    # UWB 원본 텍스트 확인 (디버그)
                    if len(line) > 3:
                        self.get_logger().info(f'[원본 데이터] {line}')

                    dist_match = re.search(r'(?i)(dist|distance|range)[\s:=,]+([+-]?\d+\.?\d*)', line)
                    if dist_match:
                        raw_val = float(dist_match.group(2))
                        raw_cm = int(raw_val)
                        dist_cm = max(0, raw_cm - self.offset_cm)
                        
                        msg = Point()
                        msg.x = float(dist_cm)  
                        msg.y = 0.0             
                        msg.z = float(gyro_rate) 

                        self.publisher_.publish(msg)
                        self.get_logger().info(f'🚀 [퍼블리시] 거리: {msg.x:.0f}cm | IMU 회전: {msg.z:.1f}')
                        
        except Exception as e:
            self.ser = None
            self.buffer = ""

def main(args=None):
    rclpy.init(args=args)
    node = UwbPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node.ser and node.ser.is_open: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()