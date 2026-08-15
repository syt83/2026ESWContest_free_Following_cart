import rclpy
from rclpy.node import Node
import serial
import time

class SimpleForwardNode(Node):
    def __init__(self):
        super().__init__('simple_forward_node')
        
        # 💡 아두이노 포트 설정
        self.arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
        self.ser = None
        self.init_arduino_serial()

        # [모터 속도 설정]
        self.BASE_PWM = 35  # 직진 속도
        
        # 0.1초마다 계속 직진 명령을 보내기 위한 타이머 생성
        self.timer = self.create_timer(0.1, self.timer_callback)

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            # 아두이노 버퍼 초기화
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(2)  
            self.get_logger().info('✅ 아두이노 연결 완료! 직진을 시작합니다.')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def timer_callback(self):
        """0.1초마다 모터에 직진 명령을 내리는 함수"""
        if self.ser and self.ser.is_open:
            try:
                # 양쪽 바퀴에 동일한 속도를 줍니다 (직진)
                cmd = f"{self.BASE_PWM},{self.BASE_PWM}\n"
                self.ser.write(cmd.encode('utf-8'))
                self.ser.flush()
            except Exception as e:
                self.get_logger().error(f'시리얼 통신 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료될 때 안전하게 멈추기 (0,0 전송)
        if node.ser and node.ser.is_open:
            node.ser.write(b"0,0\n") 
            time.sleep(0.1)
            node.ser.close()
            node.get_logger().info('🛑 모터 정지 및 포트 닫힘.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
