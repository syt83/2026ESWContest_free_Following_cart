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
        # 1. 정지 및 출발 딜레마 방지
        self.STOP_DIST = 70.0          # 완벽히 멈추는 거리 (cm)
        self.START_DIST = 75.0         # 다시 출발하는 여유 거리 (cm)
        
        # 2. 모터 전력 안정화 (블랙아웃 방지)
        self.BASE_PWM = 35             # 기본 직진 속도
        self.MIN_PWM = 35              # 모터 최소 기동 PWM
        self.MAX_PWM = 60              # 모터 최대 PWM (전압 강하 방지용 제한)
        
        # 3. 회전 속도 (부드럽게 튜닝됨)
        self.SPIN_PWM = 22             # 360도 스캔 속도 
        self.LOCAL_SPIN_PWM = 22       # 도리도리(Local Scan) 속도
        
        # 4. IMU 나침반 조향 게인 (P-Controller)
        self.KP_YAW = 1.2              # 클수록 각도 오차를 강하게 고칩니다

        # ==========================================
        # 🧠 상태 머신 변수
        # ==========================================
        self.mode = 'SCAN_360'         
        self.fail_count = 0            
        self.min_search_dist = 9999.0  
        self.best_yaw = 0.0            
        self.target_yaw = 0.0          # TRACK 모드에서 유지할 목표 IMU 각도
        self.pause_start_time = 0.0    

        self.current_yaw = 0.0         # 실시간 로봇 각도
        self.last_msg_time = time.time()
        
        self.local_scan_phase = 'LEFT'
        self.center_yaw = 0.0
        
        self.dist_history = []         
        self.last_dist = 9999.0
        self.check_time = time.time()
        self.check_interval = 0.12     # 0.12초 주기 판단

        self.is_stopped = False        
        self.last_send_time = time.time()
        self.send_interval = 0.1       # 10Hz 전송 (아두이노 논-블로킹 처리 완료)

        # 모터 소프트 스타트용 변수
        self.smooth_base_pwm = self.BASE_PWM

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(2)  
            self.get_logger().info('✅ 아두이노 연결 완료! IMU 나침반 퓨전 마스터 버전 가동🚀')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    def uwb_callback(self, msg: Point):
        raw_dist_cm = msg.x
        robot_gyro_rate = msg.z  # Z축 회전 각속도 (deg/s)

        # 🛡️ 1단계 방어벽: 유령 데이터(노이즈) 철벽 무시
        if raw_dist_cm <= 20.0:
            self.get_logger().warn(f'👻 유령 데이터 감지! 무시합니다. ({raw_dist_cm:.1f}cm)')
            return  

        now = time.time()
        dt = now - self.last_msg_time
        self.last_msg_time = now

        # IMU 각도 누적 (나침반 역할)
        self.current_yaw += robot_gyro_rate * dt

        # 반응속도 2배짜리 이동평균 필터
        self.dist_history.append(raw_dist_cm)
        if len(self.dist_history) > 2:  
            self.dist_history.pop(0)
        dist_cm = sum(self.dist_history) / len(self.dist_history)

        # 정지선 / 재출발 로직 (START_DIST를 낮춰서 딜레마 방지)
        if self.is_stopped:
            if dist_cm > self.START_DIST:
                self.is_stopped = False 
                self.get_logger().info('🚀 재출발합니다!')
            else:
                self.send_motor_command(0, 0)
                return
        else:
            if dist_cm < self.STOP_DIST:
                self.is_stopped = True  
                self.send_motor_command(0, 0)
                self.last_dist = dist_cm
                self.mode = 'TRACK'
                self.get_logger().info(f'🛑 목표 도달 정지 (현재 거리: {dist_cm:.0f}cm)')
                return

        # ==========================================
        # 🔍 [모드 1] SCAN_360 : 제자리 탐색
        # ==========================================
        if self.mode == 'SCAN_360':
            self.send_motor_command(self.SPIN_PWM, -self.SPIN_PWM) 
            if dist_cm < self.min_search_dist:
                self.min_search_dist = dist_cm
                self.best_yaw = self.current_yaw
            
            if abs(self.current_yaw) >= 360.0:
                self.mode = 'TURN_TO_BEST'
                self.get_logger().info(f'🎯 360도 스캔 완료! 주인의 각도: {self.best_yaw:.1f}도')

        # ==========================================
        # 👀 [모드 1.5] SCAN_LOCAL : 좌우 45도 도리도리
        # ==========================================
        elif self.mode == 'SCAN_LOCAL':
            if dist_cm < self.min_search_dist:
                self.min_search_dist = dist_cm
                self.best_yaw = self.current_yaw

            if self.local_scan_phase == 'LEFT':
                self.send_motor_command(-self.LOCAL_SPIN_PWM, self.LOCAL_SPIN_PWM)
                if self.current_yaw >= self.center_yaw + 45.0:
                    self.local_scan_phase = 'RIGHT'
            
            elif self.local_scan_phase == 'RIGHT':
                self.send_motor_command(self.LOCAL_SPIN_PWM, -self.LOCAL_SPIN_PWM)
                if self.current_yaw <= self.center_yaw - 45.0:
                    self.local_scan_phase = 'DONE'
            
            elif self.local_scan_phase == 'DONE':
                if self.min_search_dist <= self.last_dist + 25.0:
                    self.mode = 'TURN_TO_BEST'
                    self.get_logger().info(f'🎯 방향 찾음! 목표 각도: {self.best_yaw:.1f}도')
                else:
                    self.get_logger().warn('🚨 시야 완전 상실! 360도 스캔 돌입!')
                    self.mode = 'SCAN_360'
                    self.current_yaw = 0.0
                    self.min_search_dist = 9999.0

        # ==========================================
        # 🔄 [모드 2] TURN_TO_BEST : 목표 각도로 홱 틀기
        # ==========================================
        elif self.mode == 'TURN_TO_BEST':
            yaw_error = self.best_yaw - self.current_yaw
            if abs(yaw_error) < 10.0:
                self.send_motor_command(0, 0)
                self.mode = 'PAUSE'
                self.pause_start_time = time.time()
                self.target_yaw = self.best_yaw  # 목표 방향 확정!
            else:
                turn_speed = 30
                if yaw_error > 0: self.send_motor_command(-turn_speed, turn_speed) 
                else: self.send_motor_command(turn_speed, -turn_speed) 

        # ==========================================
        # ⏳ [모드 3] PAUSE : 0.3초 대기
        # ==========================================
        elif self.mode == 'PAUSE':
            self.send_motor_command(0, 0) 
            if now - self.pause_start_time >= 0.3:
                self.mode = 'TRACK'
                self.fail_count = 0
                self.last_dist = dist_cm
                self.dist_history.clear() 

        # ==========================================
        # 🚀 [모드 4] TRACK : 멈춤 없는 다이내믹 유도 미사일 모드
        # ==========================================
        elif self.mode == 'TRACK':
            if now - self.check_time >= self.check_interval:
                dist_diff = dist_cm - self.last_dist
                
                # 1. 거리가 줄어들거나 유지되면 -> 지금 방향이 정답! (각도 픽스)
                if dist_diff < 1.0:  
                    self.fail_count = 0
                    self.steer_dir = 1
                    # 현재의 IMU 각도를 새 목표 각도로 단단히 고정!
                    self.target_yaw = self.current_yaw 
                
                # 2. 거리가 멀어진다? -> 멈추지 않고, 달리면서 목표 각도를 살짝 꺾어봄 (유도 미사일)
                elif dist_diff > 2.5:
                    self.fail_count += 1
                    
                    # 💡 [핵심] 주행을 멈추지 않고, 목표 각도만 15도씩 좌우로 틀어봅니다!
                    if self.fail_count % 2 == 1:
                        # 홀수 번 실패 시: 기존 궤도에서 15도 꺾어서 추격
                        self.target_yaw = self.current_yaw + (15.0 * self.steer_dir)
                    else:
                        # 짝수 번 실패 시: 반대 방향으로 확 꺾어서 30도 추격
                        self.steer_dir *= -1
                        self.target_yaw = self.current_yaw + (30.0 * self.steer_dir)
                
                # 🧭 IMU P-제어 (부드러운 조향)
                yaw_error = self.target_yaw - self.current_yaw
                # 핸들 꺾는 힘을 기존 25에서 35로 올려서, 움직이는 타겟을 더 빠릿하게 쫓게 만듦
                steer_power = max(-35.0, min(35.0, yaw_error * self.KP_YAW))

                # 🚀 가속력 튜닝 (주인이 멀어지면 더 빨리 쫓아가도록 가속치 상향)
                extra_speed = max(0.0, min(25.0, (dist_cm - 70.0) * 0.2)) 
                target_base = self.BASE_PWM + extra_speed
                
                # 소프트 스타트로 전력 안정화
                self.smooth_base_pwm = (self.smooth_base_pwm * 0.7) + (target_base * 0.3)

                # 모터에 최종 명령 전달 (멈추는 로직 아예 삭제!)
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
                    self.get_logger().error(f'시리얼 통신 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UwbImuFollowerMaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        # 안전 종료 (로봇 정지)
        if node.ser and node.ser.is_open:
            node.ser.write(b"0,0\n") 
            time.sleep(0.1)
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
