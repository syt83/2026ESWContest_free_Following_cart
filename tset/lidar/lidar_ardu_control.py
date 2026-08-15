# 실행 방법: python3 lidar_ardu_control.py
# 직진하다가 장애물 만나면 정지하는 코드


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time
import math

class LidarArduinoControlNode(Node):
    def __init__(self):
        super().__init__('lidar_arduino_control_node')

        # 1. YDLIDAR 호환 QoS
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. 라이다 /scan 토픽 구독
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            lidar_qos
        )

        # ==========================================
        # [핵심 수정 구간] 아두이노 포트를 UWB 때와 동일하게 복구
        # ==========================================
        self.arduino_port = '/dev/ttyACM1'  # ◀ ttyUSB0이 아닌, UWB에서 성공했던 아두이노 포트!
        self.baudrate = 9600
        self.ser = None
        self.init_arduino_serial()

        # 4. 거리 기준 설정
        self.SAFE_DISTANCE_M = 0.40   
        self.current_state = None     
        self.safe_count = 0           

        self.get_logger().info('YDLIDAR 기반 모터 제어 노드가 시작되었습니다.')

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.baudrate, timeout=0.1)
            # ◀ UWB 코드처럼 아두이노 리셋(부팅) 대기시간 2초 반드시 부여
            time.sleep(2.0) 
            self.get_logger().info(f'★ [아두이노 연결 성공]: {self.arduino_port}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 연결 실패 ({self.arduino_port}): {e}')

    def send_cmd_once(self, new_state):
        if self.ser is None or not self.ser.is_open:
            self.init_arduino_serial()
            return

        # 상태가 변했을 때만 순수 문자 1개 전송 (UWB 방식과 동일)
        if self.current_state != new_state:
            try:
                self.ser.write(new_state.encode('utf-8'))
                if new_state == 'F':
                    self.get_logger().info(f'🟢 [상태 변경] -> 전진(F) 명령 전송!')
                else:
                    self.get_logger().warn(f'🚨 [상태 변경] -> 정지(S) 명령 전송!')
                self.current_state = new_state
            except Exception as e:
                self.get_logger().error(f'❌ 아두이노 전송 에러: {e}')
                self.ser = None

    def lidar_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_points = len(ranges)
        if num_points == 0:
            return

        angle_front_deg = 25
        idx_window = int((angle_front_deg / 360.0) * num_points)
        front_ranges = ranges[:idx_window] + ranges[-idx_window:]

        valid_ranges = [r for r in front_ranges if 0.18 < r < msg.range_max and not math.isnan(r)]

        if not valid_ranges:
            self.safe_count = 0
            self.send_cmd_once('S')
            return

        min_front_distance = min(valid_ranges)

        if min_front_distance < self.SAFE_DISTANCE_M:
            self.safe_count = 0
            self.send_cmd_once('S')
        else:
            self.safe_count += 1
            if self.safe_count >= 3:  
                self.send_cmd_once('F')

def main(args=None):
    rclpy.init(args=args)
    node = LidarArduinoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b'S')
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()