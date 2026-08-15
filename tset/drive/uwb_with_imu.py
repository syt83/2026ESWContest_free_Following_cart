import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time

class UwbImuFollowerArduino(Node):
    def __init__(self):
        super().__init__('uwb_imu_follower_node')
        self.subscription = self.create_subscription(Point, '/uwb/data', self.uwb_callback, 10)
        
        # 아두이노 포트 설정
        self.arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
        self.ser = None
        self.init_arduino_serial()

        # ==========================================
        # [튜닝 파라미터]
        # ==========================================
        self.STOP_DIST = 70.0          # 완벽히 멈추는 거리 (cm)
        self.START_DIST = 75.0         # 다시 출발하는 여유 거리 (cm)
        
        self.BASE_PWM = 35             # 기본 직진 속도
        self.MIN_PWM = 35              # 모터 최소 기동 PWM
        self.MAX_PWM = 60              # 모터 최대 PWM (전력 쏠림 방지)
        
        self.SPIN_PWM = 22             # 360도 스캔 회전 속도 (부드럽게)
        self.LOCAL_SPIN_PWM = 22       # 도리도리(Local Scan) 회전 속도
        
        # 💡 [핵심] IMU 각도 제어 게인 (P-Controller)
        # 클수록 각도 오차를 강하게 꺾어 고치고, 작으면 부드럽게 고칩니다. (1.0~1.5 권장)
        self.KP_YAW = 1.2              

        # 상태 머신 변수
        self.mode = 'SCAN_360'         
        self.fail_count = 0            
        self.min_search_dist = 9999.0  
        self.best_yaw = 0.0            
        self.target_yaw = 0.0          # 💡 TRACK 모드에서 유지할 목표 IMU 각도
        self.pause_start_time = 0.0    

        self.current_yaw = 0.0         # IMU 적분 각도 (Yaw)
        self.last_msg_time = time.time()
        
        # 도리도리(Local Scan) 변수
        self.local_scan_phase = 'LEFT'
        self.center_yaw = 0.0
        
        # 센서 데이터 및 주기 관리
        self.dist_history = []         
        self.last_dist = 9999.0
        self.check_time = time.time()
        self.check_interval = 0.12     # 0.12초마다 각도 및 거리 판단

        self.is_stopped = False        
        self.last_send_time = time.time()
        self.send_interval = 0.1       

        # 모터 소프트 스타트(전력 급반등 방지) 변수
        self.smooth_base_pwm = self.BASE_PWM

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(2)  
            self.get_logger().info('✅ IMU 나침반 기반 일직선 제어 모듈 가동!')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    def uwb_callback(self, msg: Point):
        raw_dist_cm = msg.x
        robot_gyro_rate = msg.z  # deg/s

        # 1. 유령 데이터(노이즈) 철벽 방어
        if raw_dist_cm <= 20.0:
            return  

        now = time.time()
        dt = now - self.last_msg_time
        self.last_msg_time = now

        # 2. IMU 자이로 적분을 통한 실시간 절대 각도(Yaw) 적재
        self.current_yaw += robot_gyro_rate * dt

        # 3. 거리 이동 평균 필터 (2개 샘플 - 빠른 반응)
        self.dist_history.append(raw_dist_cm)
        if len(self.dist_history) > 2:  
            self.dist_history.pop(0)
        dist_cm = sum(self.dist_history) / len(self.dist_history)

        # 4. 정지선 / 재출발 제어
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
        # 🔍 [모드 1] SCAN_360: 360도 탐색으로 최적 IMU 각도 매핑
        # ==========================================
        if self.mode == 'SCAN_360':
            self.send_motor_command(self.SPIN_PWM, -self.SPIN_PWM) 
            if dist_cm < self.min_search_dist:
                self.min_search_dist = dist_cm
                self.best_yaw = self.current_yaw
            
            if abs(self.current_yaw) >= 360.0:
                self.mode = 'TURN_TO_BEST'
                self.get_logger().info(f'🎯 360도 스캔 완료! 주인의 IMU 각도: {self.best_yaw:.1f}도')

        # ==========================================
        # 👀 [모드 1.5] SCAN_LOCAL: 좌우 45도 도리도리 스캔
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
                    self.get_logger().info(f'🎯 도리도리 성공! 목표 각도 업데이트: {self.best_yaw:.1f}도')
                else:
                    self.get_logger().warn('🚨 도리도리 실패! 360도 전체 스캔 진입!')
                    self.mode = 'SCAN_360'
                    self.current_yaw = 0.0
                    self.min_search_dist = 9999.0

        # ==========================================
        # 🔄 [모드 2] TURN_TO_BEST: 탐색된 각도로 정밀 정렬
        # ==========================================
        elif self.mode == 'TURN_TO_BEST':
            yaw_error = self.best_yaw - self.current_yaw
            if abs(yaw_error) < 10.0:  # 10도 이내 정렬 시 정지
                self.send_motor_command(0, 0)
                self.mode = 'PAUSE'
                self.pause_start_time = time.time()
                self.target_yaw = self.best_yaw  # 💡 주인을 향한 목표 각도 최종 확정!
            else:
                turn_speed = 30
                if yaw_error > 0: self.send_motor_command(-turn_speed, turn_speed) 
                else: self.send_motor_command(turn_speed, -turn_speed) 

        # ==========================================
        # ⏳ [모드 3] PAUSE: 0.3초 대기
        # ==========================================
        elif self.mode == 'PAUSE':
            self.send_motor_command(0, 0) 
            if now - self.pause_start_time >= 0.3:
                self.mode = 'TRACK'
                self.fail_count = 0
                self.last_dist = dist_cm
                self.dist_history.clear() 

        # ==========================================
        # 🚀 [모드 4] TRACK: IMU 각도 고정 P-제어 (S자 주행 완전 제거!)
        # ==========================================
        elif self.mode == 'TRACK':
            if now - self.check_time >= self.check_interval:
                dist_diff = dist_cm - self.last_dist
                
                # 1. 거리가 계속 줄어들고 있다면 -> 정방향이므로 현재 각도를 계속 새 목표 각도로 고정!
                if dist_diff < -1.5:
                    self.fail_count = 0
                    self.target_yaw = self.current_yaw  # 💡 올바른 직진 궤도 실시간 보정
                
                # 2. 거리가 3.5cm 이상 훅 멀어지면 -> 헛발질 카운트 증가
                elif dist_diff > 3.5:
                    self.fail_count += 1
                
                # 3. 헛발질 3회 누적 시 -> 찍어서 꺾지 않고 즉시 도리도리(SCAN_LOCAL) 탐색으로 진입
                if self.fail_count >= 3:
                    self.get_logger().warn('👀 타겟 각도 이탈! IMU 도리도리 재탐색!')
                    self.mode = 'SCAN_LOCAL'
                    self.local_scan_phase = 'LEFT'
                    self.center_yaw = self.current_yaw
                    self.min_search_dist = 9999.0
                    self.best_yaw = self.current_yaw
                    self.fail_count = 0
                else:
                    # 🔥 [IMU P-제어 핵심]
                    # 목표 각도(target_yaw)와 현재 로봇 각도(current_yaw)의 오차 계산
                    yaw_error = self.target_yaw - self.current_yaw
                    
                    # 각도 오차에 비례하여 조향 제어 (S자 없이 똑바로 직진)
                    steer_power = yaw_error * self.KP_YAW
                    
                    # 과도한 꺾임 방지 (최대 조향 폭 제한)
                    steer_power = max(-25.0, min(25.0, steer_power))

                    # distance 기반 속도 계산 & 소프트 스타트(전력 안정화)
                    extra_speed = (dist_cm - 70.0) * 0.15
                    extra_speed = max(0.0, min(15.0, extra_speed)) 
                    
                    target_base = self.BASE_PWM + extra_speed
                    self.smooth_base_pwm = (self.smooth_base_pwm * 0.7) + (target_base * 0.3)

                    # 왼쪽/오른쪽 모터에 IMU 피드백 차등 반영
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
                    
                    if left_speed == 0 and right_speed == 0:
                        cmd = "0,0\n"
                    else:
                        cmd = f"{l_val},{r_val}\n"
                    
                    self.ser.write(cmd.encode('utf-8'))
                    self.ser.flush()
                except Exception as e:
                    self.get_logger().error(f'시리얼 통신 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UwbImuFollowerArduino()
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
