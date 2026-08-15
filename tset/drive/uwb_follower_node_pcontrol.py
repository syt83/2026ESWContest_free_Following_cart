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

        self.STOP_DIST = 70.0          
        self.START_DIST = 85.0         
        
        self.BASE_PWM = 35             
        self.MIN_PWM = 35              
        self.MAX_PWM = 70              # 가속 한계를 조금 더 열어줌
        
        self.P_GAIN = 3.5              # 예민도를 안정적인 수치로 복구
        self.MAX_SWING = 25.0          # 코너링 파워는 넉넉하게
        self.WIGGLE_PWM = 35           # 도리도리 속도

        self.mode = 'INIT'         
        self.start_time = time.time()
        
        self.fail_count = 0            
        self.best_yaw = 0.0            
        self.min_search_dist = 9999.0  

        self.wiggle_phase = 'LEFT'
        self.center_yaw = 0.0

        self.current_yaw = 0.0         
        self.last_msg_time = time.time()
        self.dist_history = []         

        self.last_dist = 9999.0
        self.steer_dir = 1             
        self.check_time = time.time()
        self.check_interval = 0.15     

        self.smooth_steer = 0.0        
        self.is_stopped = False        
        self.track_start_time = 0.0

        self.last_send_time = time.time()
        self.send_interval = 0.1       

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            time.sleep(2)  
            self.get_logger().info('✅ [스마트 체이서] 가속/조향 분리 및 퀵 도리도리 완료!')
        except Exception as e:
            self.ser = None

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    def normalize_angle(self, angle):
        while angle > 180.0: angle -= 360.0
        while angle < -180.0: angle += 360.0
        return angle

    def uwb_callback(self, msg: Point):
        raw_dist_cm = msg.x
        robot_gyro_rate = msg.z  

        if raw_dist_cm <= 0.0: return  

        if self.mode == 'WIGGLE_SCAN':
            dist_cm = raw_dist_cm
            self.dist_history.clear() 
        else:
            self.dist_history.append(raw_dist_cm)
            if len(self.dist_history) > 5: self.dist_history.pop(0)
            if len(self.dist_history) == 5:
                sorted_history = sorted(self.dist_history) 
                dist_cm = sum(sorted_history[1:4]) / 3.0
            else:
                dist_cm = raw_dist_cm

        now = time.time()
        dt = now - self.last_msg_time
        self.last_msg_time = now
        self.current_yaw += robot_gyro_rate * dt

        if self.is_stopped:
            if dist_cm > self.START_DIST: self.is_stopped = False 
            else:
                self.send_motor_command(0, 0)
                return
        else:
            if dist_cm < self.STOP_DIST:
                self.is_stopped = True  
                self.send_motor_command(0, 0)
                self.last_dist = dist_cm
                self.mode = 'TRACK'
                return

        # ==========================================
        # ⏳ INIT
        # ==========================================
        if self.mode == 'INIT':
            self.send_motor_command(0, 0)
            if now - self.start_time > 1.0: 
                self.mode = 'WIGGLE_SCAN'
                self.wiggle_phase = 'LEFT'
                self.center_yaw = self.current_yaw
                self.min_search_dist = 9999.0

        # ==========================================
        # 🐶 WIGGLE_SCAN (좌우 55도 스캔)
        # ==========================================
        elif self.mode == 'WIGGLE_SCAN':
            if dist_cm < self.min_search_dist:
                self.min_search_dist = dist_cm
                self.best_yaw = self.current_yaw

            if self.wiggle_phase == 'LEFT':
                self.send_motor_command(-self.WIGGLE_PWM, self.WIGGLE_PWM)
                if self.current_yaw >= self.center_yaw + 55.0:
                    self.wiggle_phase = 'RIGHT'
            elif self.wiggle_phase == 'RIGHT':
                self.send_motor_command(self.WIGGLE_PWM, -self.WIGGLE_PWM)
                if self.current_yaw <= self.center_yaw - 55.0:
                    self.wiggle_phase = 'DONE'
            elif self.wiggle_phase == 'DONE':
                self.mode = 'TURN_TO_BEST'
                self.get_logger().info('⚡ 타겟 방향 확인 완료!')

        # ==========================================
        # 🔄 TURN_TO_BEST
        # ==========================================
        elif self.mode == 'TURN_TO_BEST':
            error = self.normalize_angle(self.best_yaw - self.current_yaw)
            
            if abs(error) < 12.0:
                self.mode = 'TRACK'
                self.fail_count = 0
                self.last_dist = dist_cm
                self.smooth_steer = 0.0  
                self.track_start_time = now 
                self.dist_history.clear() 
            else:
                turn_speed = 40 
                if error > 0: self.send_motor_command(-turn_speed, turn_speed) 
                else: self.send_motor_command(turn_speed, -turn_speed) 

        # ==========================================
        # 🚀 TRACK (똑똑한 추종 모드)
        # ==========================================
        elif self.mode == 'TRACK':
            if now - self.check_time >= self.check_interval:
                dist_diff = dist_cm - self.last_dist
                target_steer = 0.0
                
                # 1. 출발 무적 시간
                if now - self.track_start_time < 1.0:
                    self.fail_count = 0
                    target_steer = 0.0
                else:
                    # 💡 2. 지능적 조향 개입 로직
                    if dist_diff < 2.0:
                        # 완벽함 (직진)
                        self.fail_count = 0
                        target_steer = 0.0
                    elif 2.0 <= dist_diff < 5.0:
                        # 주인이 빠름 (엑셀만 밟고 핸들은 꺾지 않음!)
                        self.fail_count = 0
                        target_steer = 0.0
                    else:
                        # 확실히 방향이 틀어짐 (이때만 핸들을 꺾음!)
                        self.fail_count += 1
                        if self.fail_count == 2 or self.fail_count == 4: 
                            self.steer_dir *= -1
                        target_steer = min(dist_diff * self.P_GAIN, self.MAX_SWING) * self.steer_dir

                # 3. 안정적인 스무딩 
                self.smooth_steer = (self.smooth_steer * 0.5) + (target_steer * 0.5)

                if self.fail_count >= 8: 
                    self.get_logger().warn('🚨 타겟 이탈. 즉시 재스캔!')
                    self.mode = 'WIGGLE_SCAN'
                    self.wiggle_phase = 'LEFT'
                    self.center_yaw = self.current_yaw
                    self.min_search_dist = 9999.0
                    self.fail_count = 0
                else:
                    # 💡 4. 가속 시스템: 주인이 멀어질수록 속도를 쭉쭉 뽑아냄
                    extra_speed = (dist_cm - 70.0) * 0.35 
                    extra_speed = max(0.0, min(35.0, extra_speed)) 
                    current_base = self.BASE_PWM + extra_speed

                    damping = robot_gyro_rate * 0.4
                    final_steer = self.smooth_steer - damping

                    self.send_motor_command(current_base + final_steer, current_base - final_steer)
                    
                self.last_dist = dist_cm
                self.check_time = now

    def send_motor_command(self, left_speed, right_speed):
        now = time.time()
        if now - self.last_send_time >= self.send_interval:
            self.last_send_time = now
            if self.ser and self.ser.is_open:
                l_val = int(max(-self.MAX_PWM, min(self.MAX_PWM, self.apply_min_pwm(left_speed))))
                r_val = int(max(-self.MAX_PWM, min(self.MAX_PWM, self.apply_min_pwm(right_speed))))
                cmd = "0,0\n" if left_speed == 0 and right_speed == 0 else f"{l_val},{r_val}\n"
                self.ser.write(cmd.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = UwbFollowerArduino()
    try: rclpy.spin(node)
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