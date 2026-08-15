import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time

class UwbFollowerArduino(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')
        self.subscription = self.create_subscription(Point, '/uwb/data', self.uwb_callback, 10)
        
        self.arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
        self.ser = None
        self.init_arduino_serial()

        # [튜닝 파라미터]
        self.TARGET_DIST_CM = 80.0     
        self.BASE_PWM = 30             
        self.SWING_PWM = 5           
        
        self.MIN_PWM = 35              
        self.MAX_PWM = 60              
        self.SPIN_PWM = 20             #처음 회전하는 속도

        # 🚀 상태 머신 변수 추가
        self.mode = 'SCAN_360'         
        self.fail_count = 0            
        self.min_search_dist = 9999.0  
        self.best_yaw = 0.0            
        self.pause_start_time = 0.0    # 💡 0.5초 대기를 위한 시간 기록 변수

        self.current_yaw = 0.0         
        self.last_msg_time = time.time()

        self.last_dist = 9999.0
        self.steer_dir = 1             
        self.check_time = time.time()
        self.check_interval = 0.25   

        self.last_send_time = time.time()
        self.send_interval = 0.1       

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            time.sleep(2)  
            self.get_logger().info('✅ 아두이노 연동 완료 - 360도 탐색 후 0.5초 대기 출발 적용')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    def uwb_callback(self, msg: Point):
        dist_cm = msg.x
        robot_gyro_rate = msg.z  

        if dist_cm <= 0.0:
            return  

        now = time.time()
        dt = now - self.last_msg_time
        self.last_msg_time = now

        self.current_yaw += robot_gyro_rate * dt

        if dist_cm < self.TARGET_DIST_CM:
            self.send_motor_command(0, 0)
            self.last_dist = dist_cm
            self.mode = 'TRACK'
            self.get_logger().info(f'✅ 정지 (목표 도달: {dist_cm:.0f}cm)')
            return

        # ==========================================
        # 🔍 [모드 1] SCAN_360: 무조건 360도 스캔
        # ==========================================
        if self.mode == 'SCAN_360':
            self.send_motor_command(self.SPIN_PWM, -self.SPIN_PWM) 
            
            if dist_cm < self.min_search_dist:
                self.min_search_dist = dist_cm
                self.best_yaw = self.current_yaw
                
            if abs(self.current_yaw) >= 360.0:
                self.mode = 'TURN_TO_BEST'
                self.get_logger().info(f'🎯 360도 스캔 완료! 최적 각도: {self.best_yaw:.1f}도')

        # ==========================================
        # 🔄 [모드 2] TURN_TO_BEST: 최적의 각도로 되돌아가기
        # ==========================================
        elif self.mode == 'TURN_TO_BEST':
            error = self.best_yaw - self.current_yaw
            
            if abs(error) < 15.0:
                # 💡 [핵심 추가] 정면을 찾으면 모터를 끄고 PAUSE 모드로 돌입!
                self.send_motor_command(0, 0)
                self.mode = 'PAUSE'
                self.pause_start_time = time.time()
                self.get_logger().info('🛑 정면 확인! 0.5초 대기합니다...')
            else:
                turn_speed = 35 
                if error > 0:
                    self.send_motor_command(-turn_speed, turn_speed) 
                else:
                    self.send_motor_command(turn_speed, -turn_speed) 

        # ==========================================
        # ⏳ [모드 3] PAUSE: 0.5초 동안 폼 잡고 기다리기
        # ==========================================
        elif self.mode == 'PAUSE':
            self.send_motor_command(0, 0) # 모터에 확실히 0을 줌
            
            # 0.5초가 지났는지 확인
            if now - self.pause_start_time >= 0.5:
                self.mode = 'TRACK'
                self.fail_count = 0
                self.last_dist = dist_cm
                self.get_logger().info('🚀 대기 완료! S자 추종 출발!')

        # ==========================================
        # 🚀 [모드 4] TRACK: 얕은 S자 궤적으로 쫓아가기
        # ==========================================
        elif self.mode == 'TRACK':
            if now - self.check_time >= self.check_interval:
                dist_diff = dist_cm - self.last_dist
                
                if dist_diff > 3.0:
                    self.steer_dir *= -1
                    self.fail_count += 1
                elif dist_diff < -2.0:
                    self.fail_count = 0  
                
                if self.fail_count >= 4:
                    self.get_logger().warn('🚨 타겟 상실! 다시 360도 탐색 시작!')
                    self.mode = 'SCAN_360'
                    self.current_yaw = 0.0          
                    self.min_search_dist = 9999.0
                    self.fail_count = 0
                else:
                    steer_power = self.SWING_PWM * self.steer_dir
                    damping = robot_gyro_rate * 0.4
                    final_steer = steer_power - damping

                    left_motor = self.BASE_PWM + final_steer
                    right_motor = self.BASE_PWM - final_steer

                    self.send_motor_command(left_motor, right_motor)
                    
                self.last_dist = dist_cm
                self.check_time = now

    def send_motor_command(self, left_speed, right_speed):
        now = time.time()
        if now - self.last_send_time >= self.send_interval:
            self.last_send_time = now
            if self.ser and self.ser.is_open:
                l_val = int(max(-self.MAX_PWM, min(self.MAX_PWM, self.apply_min_pwm(left_speed))))
                r_val = int(max(-self.MAX_PWM, min(self.MAX_PWM, self.apply_min_pwm(right_speed))))
                
                if left_speed == 0 and right_speed == 0:
                    cmd = "0,0\n"
                else:
                    cmd = f"{l_val},{r_val}\n"
                
                self.ser.write(cmd.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = UwbFollowerArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b"0,0\n") 
            time.sleep(0.1)
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()