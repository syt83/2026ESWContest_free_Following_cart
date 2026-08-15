import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time

class UwbFollowerArduino(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')
        self.subscription = self.create_subscription(Point, '/uwb/data', self.uwb_callback, 10)
        
        # 아두이노 포트 설정
        self.arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
        self.ser = None
        self.init_arduino_serial()

        # ==========================================
        # [튜닝 파라미터]
        # ==========================================
        self.STOP_DIST = 70.0          # 완벽히 멈추는 거리
        self.START_DIST = 85.0         # 다시 출발하는 거리 (여유 구간)
        
        self.BASE_PWM = 35             # 기본 직진 속도
        self.SWING_PWM = 20            # 방향 수정할 때 살짝 칠 힘 (직진 위주)
        
        self.MIN_PWM = 35              
        self.MAX_PWM = 65              
        
        self.SPIN_PWM = 25             # 360도 회전 시 속도
        self.LOCAL_SPIN_PWM = 30       # 도리도리(좌우 탐색) 시 속도 (좀 더 빠릿하게)

        # 상태 머신 변수
        self.mode = 'SCAN_360'         # 초기 시작은 360도 스캔
        self.fail_count = 0            
        self.min_search_dist = 9999.0  
        self.best_yaw = 0.0            
        self.pause_start_time = 0.0    

        self.current_yaw = 0.0         
        self.last_msg_time = time.time()
        
        # 💡 도리도리(Local Scan)용 변수
        self.local_scan_phase = 'LEFT'
        self.center_yaw = 0.0
        
        # 센서 데이터 필터링용 큐
        self.dist_history = []         

        self.last_dist = 9999.0
        self.steer_dir = 1             
        self.check_time = time.time()
        self.check_interval = 0.15      # 눈치채는 주기 (0.2초)

        self.is_stopped = False        
        
        self.last_send_time = time.time()
        self.send_interval = 0.1       

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            time.sleep(2)  
            self.get_logger().info('✅ 아두이노 연동 완료 - 직진 우선 + 도리도리 알고리즘 적용')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    def uwb_callback(self, msg: Point):
        raw_dist_cm = msg.x
        robot_gyro_rate = msg.z  

        if raw_dist_cm <= 0.0:
            return  

        # 1. 이동 평균 필터 (거리 튀는 현상 억제)
        self.dist_history.append(raw_dist_cm)
        if len(self.dist_history) > 4:  
            self.dist_history.pop(0)
        dist_cm = sum(self.dist_history) / len(self.dist_history)

        now = time.time()
        dt = now - self.last_msg_time
        self.last_msg_time = now
        self.current_yaw += robot_gyro_rate * dt

        # 2. 정지선 딜레마 방지
        if self.is_stopped:
            if dist_cm > self.START_DIST:
                self.is_stopped = False 
            else:
                self.send_motor_command(0, 0)
                return
        else:
            if dist_cm < self.STOP_DIST:
                self.is_stopped = True  
                self.send_motor_command(0, 0)
                self.last_dist = dist_cm
                self.mode = 'TRACK'
                self.get_logger().info(f'🛑 안정권 도달 정지 (현재 거리: {dist_cm:.0f}cm)')
                return

        # ==========================================
        # 🔍 [모드 1] SCAN_360: 최후의 수단 (주인이 뒤로 갔을 때만)
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
        # 👀 [모드 1.5] SCAN_LOCAL: 좌우 45도 도리도리 탐색
        # ==========================================
        elif self.mode == 'SCAN_LOCAL':
            # 도리도리 중 가장 거리가 짧았던 각도 기록
            if dist_cm < self.min_search_dist:
                self.min_search_dist = dist_cm
                self.best_yaw = self.current_yaw

            if self.local_scan_phase == 'LEFT':
                self.send_motor_command(-self.LOCAL_SPIN_PWM, self.LOCAL_SPIN_PWM) # 왼쪽 턴
                if self.current_yaw >= self.center_yaw + 45.0:
                    self.local_scan_phase = 'RIGHT'
            
            elif self.local_scan_phase == 'RIGHT':
                self.send_motor_command(self.LOCAL_SPIN_PWM, -self.LOCAL_SPIN_PWM) # 오른쪽 턴 (총 90도 스윙)
                if self.current_yaw <= self.center_yaw - 45.0:
                    self.local_scan_phase = 'DONE'
            
            elif self.local_scan_phase == 'DONE':
                # 도리도리 완료 후 타겟 판별
                # 놓치기 직전 거리보다 30cm 이상 멀어지지 않았다면 앞에 있는 것으로 간주!
                if self.min_search_dist <= self.last_dist + 30.0:
                    self.mode = 'TURN_TO_BEST'
                    self.get_logger().info(f'🎯 도리도리 성공! 방향 찾음 ({self.best_yaw:.1f}도)')
                else:
                    self.get_logger().warn('🚨 도리도리 실패! 주인이 뒤에 있다! 360도 스캔!')
                    self.mode = 'SCAN_360'
                    self.current_yaw = 0.0
                    self.min_search_dist = 9999.0

        # ==========================================
        # 🔄 [모드 2] TURN_TO_BEST: 최적의 각도로 몸 돌리기
        # ==========================================
        elif self.mode == 'TURN_TO_BEST':
            error = self.best_yaw - self.current_yaw
            if abs(error) < 15.0:
                self.send_motor_command(0, 0)
                self.mode = 'PAUSE'
                self.pause_start_time = time.time()
            else:
                turn_speed = 35 
                if error > 0: self.send_motor_command(-turn_speed, turn_speed) 
                else: self.send_motor_command(turn_speed, -turn_speed) 

        # ==========================================
        # ⏳ [모드 3] PAUSE: 0.5초 동안 대기
        # ==========================================
        elif self.mode == 'PAUSE':
            self.send_motor_command(0, 0) 
            if now - self.pause_start_time >= 0.5:
                self.mode = 'TRACK'
                self.fail_count = 0
                self.last_dist = dist_cm
                self.dist_history.clear() 

        # ==========================================
        # 🚀 [모드 4] TRACK: 직진 우선(눈치껏 수정) 알고리즘
        # ==========================================
        elif self.mode == 'TRACK':
            if now - self.check_time >= self.check_interval:
                dist_diff = dist_cm - self.last_dist
                
                if dist_diff < -1.0:
                    self.fail_count = 0  
                    steer_power = 0  # 완벽 직진
                elif dist_diff > 2.0:
                    self.steer_dir *= -1
                    self.fail_count += 1
                    steer_power = self.SWING_PWM * self.steer_dir # 반대로 찔러보기
                else:
                    steer_power = 0 if self.fail_count == 0 else (self.SWING_PWM * self.steer_dir)
                
                # 💡 눈치 게임 실패 시 -> 360도가 아니라 '도리도리(SCAN_LOCAL)'로 먼저 넘어감
                if self.fail_count >= 5:
                    self.get_logger().warn('👀 타겟 흐릿함! 제자리 좌우(도리도리) 탐색 시작!')
                    self.mode = 'SCAN_LOCAL'
                    self.local_scan_phase = 'LEFT'
                    self.center_yaw = self.current_yaw
                    self.min_search_dist = 9999.0
                    self.best_yaw = self.current_yaw
                    self.fail_count = 0
                else:
                    # 무단 변속 (거리가 멀수록 부드럽게 가속)
                    extra_speed = (dist_cm - 70.0) * 0.2
                    extra_speed = max(0.0, min(20.0, extra_speed)) 
                    current_base = self.BASE_PWM + extra_speed

                    # IMU를 이용해 직진 궤도 꽉 잡아주기
                    damping = robot_gyro_rate * 0.4
                    final_steer = steer_power - damping

                    left_motor = current_base + final_steer
                    right_motor = current_base - final_steer

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