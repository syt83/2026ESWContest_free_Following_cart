# 실행 방법: python3 uwb_ardu_control.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time

class UWBArduinoControlNode(Node):
    def __init__(self):
        super().__init__('uwb_arduino_control_node')

        # 1. UWB 퍼블리셔의 /uwb/data 토픽 구독
        self.subscription = self.create_subscription(
            Point,
            '/uwb/data',
            self.uwb_callback,
            10
        )

        # 2. 아두이노 시리얼 포트 설정
        self.arduino_port = '/dev/ttyACM1'  # 아두이노 시리얼 포트
        self.baudrate = 9600                # 아두이노 Serial.begin(9600)
        self.ser = None
        self.init_arduino_serial()

        # 제어 기준 거리 설정 (단위: cm)
        self.STOP_MIN_CM = 60.0   # 50cm 미만: 정지 ('S')
        self.FAR_MAX_CM = 300.0   # 50cm ~ 300cm: 전진 ('F')

        # 상태 변화를 감지하기 위한 변수 (중복 전송 방지)
        self.current_state = None

        self.get_logger().info('UWB-아두이노 원샷 상태 제어 노드가 시작되었습니다.')

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.baudrate, timeout=0.1)
            time.sleep(2)  # 아두이노 DTR 리셋 대기
            self.get_logger().info(f'아두이노 시리얼 연결 성공: {self.arduino_port}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'아두이노 시리얼 연결 실패 ({self.arduino_port}): {e}')

    def send_cmd_once(self, new_state):
        # [핵심] 상태(F, S 등)가 "바뀌었을 때만" 딱 1번 전송!
        if self.current_state != new_state:
            if self.ser and self.ser.is_open:
                # \r\n 줄바꿈 문자 없이 순수 한 글자만 전송! ('F', 'S' 등)
                self.ser.write(new_state.encode('utf-8'))
                
                self.get_logger().info(f'★ [상태 변경] {self.current_state} -> {new_state} (아두이노 전송!)')
                self.current_state = new_state

    def uwb_callback(self, msg: Point):
        if self.ser is None or not self.ser.is_open:
            self.init_arduino_serial()
            return

        distance_cm = msg.x  # UWB 거리(cm)

        # 1. 50cm 미만 -> 정지 ('S')
        if distance_cm < self.STOP_MIN_CM:
            self.send_cmd_once('S')

        # 2. 50cm ~ 300cm 사이 -> 전진 ('F')
        elif self.STOP_MIN_CM <= distance_cm <= self.FAR_MAX_CM:
            self.send_cmd_once('F')

        # 3. 너무 멀어지거나 거리 측정 범위를 벗어남 -> 정지 ('S')
        else:
            self.send_cmd_once('S')

def main(args=None):
    rclpy.init(args=args)
    node = UWBArduinoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b'S')  # 종료 시 정지 명령 전송
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()