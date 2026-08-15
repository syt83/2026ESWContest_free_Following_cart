import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time
import threading  # 💡 백그라운드 감시용 스레드 라이브러리 추가
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

# 라이다 제어 핀 번호
LIDAR_M_CTR_PIN = 18

class UwbImuFollowerMaster(Node):
    def __init__(self):
        super().__init__('uwb_imu_follower_node')
        self.subscription = self.create_subscription(Point, '/uwb/data', self.uwb_callback, 10)
        
        # 🔌 아두이노 포트 설정
        self.arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
        self.ser = None
        self.init_arduino_serial()

        # ==========================================
        # 🔑 [추가] 시스템 상태 (ON/OFF) 제어 변수
        # ==========================================
        self.system_active = False  # 처음 켜졌을 때는 무조건 STANDBY 상태
        
        # GPIO 초기화 및 라이다 강제 정지
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LIDAR_M_CTR_PIN, GPIO.OUT)
        GPIO.output(LIDAR_M_CTR_PIN, GPIO.LOW)
        
        # 💡 [핵심] NFC 카드만 하루 종일 감시하는 백그라운드 요원(스레드) 투입
        self.nfc_thread = threading.Thread(target=self.nfc_monitor_loop, daemon=True)
        self.nfc_thread.start()
        
        self.get_logger().info('💤 시스템 대기 중 (STANDBY) - 카드를 태그하여 로봇을 깨우세요.')

        # ==========================================
        # ⚙️ [튜닝 파라미터] 및 상태 변수들 (기존과 동일)
        # ==========================================
        self.STOP_DIST = 120.0          
        self.START_DIST = 125.0         
        self.BASE_PWM = 30             
        self.MIN_PWM = 30              
        self.MAX_PWM = 55              
        self.SPIN_PWM = 22             
        self.LOCAL_SPIN_PWM = 22       
        self.KP_YAW = 3.0              

        self.mode = 'SCAN_360'         
        self.fail_count = 0            
        self.steer_dir = 1             
        self.min_search_dist = 9999.0  
        self.best_yaw = 0.0            
        self.target_yaw = 0.0          
        self.pause_start_time = 0.0    
        self.is_uturning = False       
        self.current_yaw = 0.0         
        self.last_msg_time = time.time()
        self.local_scan_phase = 'LEFT'
        self.center_yaw = 0.0
        self.dist_history = []         
        self.last_dist = 9999.0
        self.check_time = time.time()
        self.check_interval = 0.12     
        self.is_stopped = False        
        self.last_send_time = time.time()
        self.send_interval = 0.1       
        self.smooth_base_pwm = self.BASE_PWM

    # ==========================================
    # 🕵️‍♂️ [추가] 백그라운드 NFC 감시 로직 (ON/OFF 토글)
    # ==========================================
    def nfc_monitor_loop(self):
        reader = SimpleMFRC522()
        
        while rclpy.ok():
            try:
                # 여기서 카드가 올 때까지 대기하지만, 메인 ROS 2 통신은 방해하지 않음
                card_id, text = reader.read()
                
                # 🟢 OFF 상태일 때 카드를 댔다면? -> 깨우기!
                if not self.system_active:
                    self.get_logger().info('====================================')
                    self.get_logger().info(f'🔓 인증 완료! (ID: {card_id})')
                    self.get_logger().info('⚡ 라이다 가동! 주행 준비 중...')
                    self.get_logger().info('====================================')
                    
                    GPIO.output(LIDAR_M_CTR_PIN, GPIO.HIGH)
                    time.sleep(3)  # 라이다 정상 회전까지 대기
                    
                    self.system_active = True
                    self.get_logger().info('🚀 주행 알고리즘 활성화 완료!')
                    
                # 🔴 ON 상태일 때 카드를 댔다면? -> 재우기!
                else:
                    self.get_logger().info('====================================')
                    self.get_logger().info(f'🔒 카드 재인식! (ID: {card_id})')
                    self.get_logger().info('💤 시스템 대기 모드로 진입합니다.')
                    self.get_logger().info('====================================')
                    
                    self.system_active = False
                    
                    # 1. 즉시 모터 멈춤
                    self.send_motor_command(0, 0)
                    # 2. 라이다 모터 끄기 (절전)
                    GPIO.output(LIDAR_M_CTR_PIN, GPIO.LOW)
                
                # 카드를 꾹 대고 있을 때 중복 인식되는 것을 막기 위한 딜레이
                time.sleep(2)
                
            except Exception as e:
                self.get_logger().error(f'NFC 통신 에러: {e}')
                time.sleep(1)

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(2)  
            self.get_logger().info('✅ 아두이노 연결 완료!')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    def normalize_angle(self, angle):
        while angle > 180.0: angle -= 360.0
        while angle < -180.0: angle += 360.0
        return angle

    def uwb_callback(self, msg: Point):
        # 💡 [핵심 안전장치] 대기(OFF) 상태면 센서 값이 들어와도 무시하고 바로 종료!
        if not self.system_active:
            return

        raw_dist_cm = msg.x
        robot_gyro_rate = msg.z  

        if raw_dist_cm <= 20.0:
            return  

        now = time.time()
        dt = now - self.last_msg_time
        self.last_msg_time = now

        self.current_yaw += robot_gyro_rate * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)

        self.dist_history.append(raw_dist_cm)
        if len(self.dist_history) > 3:  
            self.dist_history.pop(0)
        dist_cm = sum(self.dist_history) / len(self.dist_history)

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

        # SCAN_360 모드
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

        # TURN_TO_BEST 모드
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

        # PAUSE 모드
        elif self.mode == 'PAUSE':
            self.send_motor_command(0, 0) 
            if now - self.pause_start_time >= 0.3:
                self.mode = 'TRACK'
                self.fail_count = 0
                self.last_dist = dist_cm
                self.dist_history.clear() 

        # TRACK 모드
        elif self.mode == 'TRACK':
            if self.is_uturning:
                yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
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

            if now - self.check_time >= self.check_interval:
                dist_diff = dist_cm - self.last_dist
                
                if dist_diff < 0.0:
                    self.fail_count = 0
                    self.target_yaw = self.normalize_angle(self.current_yaw + (5.0 * self.steer_dir))
                else:
                    self.fail_count += 1
                    
                    if self.fail_count == 3 or self.fail_count == 5:
                        self.steer_dir *= -1           
                        self.target_yaw = self.normalize_angle(self.current_yaw + (30.0 * self.steer_dir))
                        self.get_logger().info('🔄 궤도 수정 (조향 반전)')
                    
                    elif self.fail_count >= 7:
                        self.get_logger().warn('🚨 타겟 후방 감지! 180도 절대 U턴 모드 진입!')
                        self.is_uturning = True  
                        self.target_yaw = self.normalize_angle(self.current_yaw + 180.0)
                        return 

            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            steer_power = max(-35.0, min(35.0, yaw_error * self.KP_YAW))

            extra_speed = max(0.0, min(25.0, (dist_cm - 70.0) * 0.2)) 
            target_base = self.BASE_PWM + extra_speed
            
            self.smooth_base_pwm = (self.smooth_base_pwm * 0.7) + (target_base * 0.3)

            left_motor = self.smooth_base_pwm - steer_power
            right_motor = self.smooth_base_pwm + steer_power

            self.send_motor_command(left_motor, right_motor)
                
            self.last_dist = dist_cm
            self.check_time = now

    def send_motor_command(self, left_speed, right_speed):
        # 💡 [핵심 안전장치 2] 대기 모드일 때는 무조건 정지(0, 0) 명령만 아두이노로 보낼 수 있음
        if not self.system_active and (left_speed != 0 or right_speed != 0):
            return

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
    # 프로그램이 켜지면 노드는 바로 작동을 시작하지만, 
    # 내부에 세팅된 상태 변수(system_active)가 False라서 아무 행동도 하지 않고 숨을 죽입니다.
    node = UwbImuFollowerMaster()
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        # 종료 시 하드웨어 안전 정리
        if node.ser and node.ser.is_open:
            node.ser.write(b"0,0\n") 
            time.sleep(0.1)
            node.ser.close()
            
        GPIO.output(LIDAR_M_CTR_PIN, GPIO.LOW)
        GPIO.cleanup()
        print("\n✅ 하드웨어 제어권 안전 반환 및 종료")
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()