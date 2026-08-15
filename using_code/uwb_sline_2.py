import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time

class UwbImuFollowerMaster(Node):
    def __init__(self):
        super().__init__('uwb_imu_follower_node')
        self.subscription = self.create_subscription(Point, '/uwb/data', self.uwb_callback, 10)
        
        # 🔌 아두이노 포트 설정
        self.arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
        self.ser = None
        self.init_arduino_serial()

        # ==========================================
        # ⚙️ [튜닝 파라미터]
        # ==========================================
        # 1. 정지 및 출발
        self.STOP_DIST = 120.0          
        self.START_DIST = 125.0         
        
        # 2. 모터 전력 안정화
        self.BASE_PWM = 30             
        self.MIN_PWM = 30              
        self.MAX_PWM = 55              
        
        # 3. 회전 속도 설정
        self.SPIN_PWM = 22             
        self.LOCAL_SPIN_PWM = 22       
        
        # 4. IMU 조향 제어 게인
        self.KP_YAW = 3.0              

        # ==========================================
        # 🧠 상태 머신 및 방향 제어 변수
        # ==========================================
        self.mode = 'SCAN_360'         
        self.fail_count = 0            
        self.steer_dir = 1             
        
        self.min_search_dist = 9999.0  
        self.best_yaw = 0.0            
        self.target_yaw = 0.0          
        self.pause_start_time = 0.0    

        # U턴 전용 락(Lock) 변수
        self.is_uturning = False       

        self.current_yaw = 0.0         
        self.last_msg_time = time.time()
        
        self.local_scan_phase = 'LEFT'
        self.center_yaw = 0.0
        
        # 센서 데이터 이동평균 필터
        self.dist_history = []         
        self.last_dist = 9999.0
        self.check_time = time.time()
        self.check_interval = 0.12     

        self.is_stopped = False        
        self.last_send_time = time.time()
        self.send_interval = 0.1       

        self.smooth_base_pwm = self.BASE_PWM

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(2)  
            self.get_logger().info('✅ 아두이노 연결! 🚀점진적 스위핑(측면 방어) + 절대 U턴 모드 가동!')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    # 360도 각도 정규화 (-180 ~ 180도 범위로 맞춤)
    def normalize_angle(self, angle):
        while angle > 180.0: angle -= 360.0
        while angle < -180.0: angle += 360.0
        return angle

    def uwb_callback(self, msg: Point):
        raw_dist_cm = msg.x
        robot_gyro_rate = msg.z  

        # 유령 데이터 방어벽
        if raw_dist_cm <= 20.0:
            return  

        now = time.time()
        dt = now - self.last_msg_time
        self.last_msg_time = now

        # IMU 각도 누적 (정규화 적용)
        self.current_yaw += robot_gyro_rate * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)

        # 이동평균 필터 (3개 유지)
        self.dist_history.append(raw_dist_cm)
        if len(self.dist_history) > 3:  
            self.dist_history.pop(0)
        dist_cm = sum(self.dist_history) / len(self.dist_history)

        # 정지/재출발 판단
        if self.is_stopped:
            if dist_cm > self.START_DIST:
                self.is_stopped = False 
                self.get_logger().info('🚀 타겟 이동 감지! 재출발!')
            else:
                self.send_motor_command(0, 0)
                return
        else:
            if dist_cm < self.STOP_DIST:
                self.is_stopped = True  
                self.send_motor_command(0, 0)
                self.last_dist = dist_cm
                self.mode = 'TRACK'
                self.get_logger().info(f'🛑 목표 도달 정지 (거리: {dist_cm:.0f}cm)')
                return

        # ==========================================
        # 🔍 [모드 1] SCAN_360 (최초 1회 발동)
        # ==========================================
        if self.mode == 'SCAN_360':
            self.send_motor_command(self.SPIN_PWM, -self.SPIN_PWM) 
            if dist_cm < self.min_search_dist:
                self.min_search_dist = dist_cm
                self.best_yaw = self.current_yaw
            
            if not hasattr(self, 'scan_start_yaw'):
                self.scan_start_yaw = self.current_yaw
                self.accumulated_turn = 0.0
                self.last_yaw = self.current_yaw
            else:
                diff = self.normalize_angle(self.current_yaw - self.last_yaw)
                self.accumulated_turn += diff
                
                if abs(self.accumulated_turn) >= 360.0:
                    self.mode = 'TURN_TO_BEST'
                    delattr(self, 'scan_start_yaw')
                    self.get_logger().info(f'🎯 360 스캔 완료. 최적 각도: {self.best_yaw:.1f}')
            self.last_yaw = self.current_yaw

        # ==========================================
        # 🔄 [모드 2] TURN_TO_BEST
        # ==========================================
        elif self.mode == 'TURN_TO_BEST':
            yaw_error = self.normalize_angle(self.best_yaw - self.current_yaw)
            if abs(yaw_error) < 10.0:
                self.send_motor_command(0, 0)
                self.mode = 'PAUSE'
                self.pause_start_time = time.time()
                self.target_yaw = self.best_yaw  
            else:
                turn_speed = 30
                if yaw_error > 0: self.send_motor_command(-turn_speed, turn_speed) 
                else: self.send_motor_command(turn_speed, -turn_speed) 

        # ==========================================
        # ⏳ [모드 3] PAUSE
        # ==========================================
        elif self.mode == 'PAUSE':
            self.send_motor_command(0, 0) 
            if now - self.pause_start_time >= 0.3:
                self.mode = 'TRACK'
                self.fail_count = 0
                self.last_dist = dist_cm
                self.dist_history.clear() 

        # ==========================================
        # 🐍 [모드 4] TRACK : 점진적 스위핑(측면 방어) + U턴 로직
        # ==========================================
        elif self.mode == 'TRACK':
            
            # U턴 전용 락: U턴 중이면 절대 딴짓 안 함
            if self.is_uturning:
                yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                
                # U턴 오차 15도 이내 도달 시 해제
                if abs(yaw_error) < 15.0:
                    self.is_uturning = False
                    self.fail_count = 0
                    self.last_dist = dist_cm
                    self.get_logger().info('✅ 180도 U턴 완료! 추적 재개!')
                else:
                    turn_speed = 35  
                    if yaw_error > 0:
                        self.send_motor_command(-turn_speed, turn_speed)
                    else:
                        self.send_motor_command(turn_speed, -turn_speed)
                return

            # 평상시 (U턴 중이 아닐 때)
            if now - self.check_time >= self.check_interval:
                dist_diff = dist_cm - self.last_dist
                
                # 1. 정답 경로: 거리가 줄어듦
                if dist_diff < 0.0:
                    self.fail_count = 0
                    self.target_yaw = self.normalize_angle(self.current_yaw + (5.0 * self.steer_dir))
                
                # 2. 오답 경로: 거리가 늘어남
                else:
                    self.fail_count += 1
                    
                    # 💡 [1단계 탐색 - 전방] 3번 실패 시: 35도만 꺾어봄
                    if self.fail_count == 3:
                        self.steer_dir *= -1           
                        self.target_yaw = self.normalize_angle(self.current_yaw + (35.0 * self.steer_dir))
                        self.get_logger().info('🔄 1차 스위핑 (35도)')
                    
                    # 💡 [2단계 탐색 - 측면] 5번 실패 시: 옆구리로 빠졌다고 의심! 75도로 고개 확 꺾음
                    elif self.fail_count == 5:
                        self.steer_dir *= -1           
                        self.target_yaw = self.normalize_angle(self.current_yaw + (75.0 * self.steer_dir))
                        self.get_logger().info('👀 2차 측면 정밀 탐색 (75도)')
                    
                    # 💡 [3단계 탐색 - 후방] 7번 연속 실패 시: 측면에도 없음! 180도 U턴 락 발동
                    elif self.fail_count >= 7:
                        self.get_logger().warn('🚨 타겟 후방 감지! 180도 절대 U턴 모드 진입!')
                        self.is_uturning = True  
                        self.target_yaw = self.normalize_angle(self.current_yaw + 180.0)
                        return 

                # 평소 S자 IMU P-제어
                yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                steer_power = max(-35.0, min(35.0, yaw_error * self.KP_YAW))

                # 가속 제어
                extra_speed = max(0.0, min(25.0, (dist_cm - 70.0) * 0.2)) 
                target_base = self.BASE_PWM + extra_speed
                
                self.smooth_base_pwm = (self.smooth_base_pwm * 0.7) + (target_base * 0.3)

                left_motor = self.smooth_base_pwm - steer_power
                right_motor = self.smooth_base_pwm + steer_power

                self.send_motor_command(left_motor, right_motor)
                    
                self.last_dist = dist_cm
                self.check_time = now

    def send_motor_command(self, left_speed, right_speed):
        now = time.time()
        if now - self.last_send_time >= self.send_interval:
            self.last_send_time = now
            if self.ser and self.ser.is_open:
                try:
                    l_val = int(max(-self.MAX_PWM, min(self.MAX_PWM, self.apply_min_pwm(left_speed))))
                    r_val = int(max(-self.MAX_PWM, min(self.MAX_PWM, self.apply_min_pwm(right_speed))))
                    
                    cmd = "0,0\n" if (left_speed == 0 and right_speed == 0) else f"{l_val},{r_val}\n"
                    
                    self.ser.write(cmd.encode('utf-8'))
                    self.ser.flush()
                except Exception as e:
                    self.get_logger().error(f'시리얼 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UwbImuFollowerMaster()
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