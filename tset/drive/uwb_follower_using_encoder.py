import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time
import math
import threading

class UwbFollowerArduino(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')
        self.subscription = self.create_subscription(Point, '/uwb/data', self.uwb_callback, 10)
        
        self.arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
        self.ser = None
        self.init_arduino_serial()

        # ==========================================
        # ⚙️ 주행 및 추종 파라미터
        # ==========================================
        self.STOP_DIST = 70.0          
        self.START_DIST = 85.0         
        
        self.BASE_PWM = 35             
        self.MIN_PWM = 35              
        self.MAX_PWM = 65              
        
        # 💡 [엔코더 튜닝] 로봇 휠 크기와 모터 사양에 따라 달라지는 상수 (임의의 초기값)
        self.TICKS_PER_CM = 20.0       # 바퀴가 1cm 굴러갈 때 올라가는 엔코더 틱 수 (예상치)
        self.WHEEL_BASE_CM = 25.0      # 로봇의 왼쪽 바퀴와 오른쪽 바퀴 사이의 거리 (cm)

        self.MAX_SWING = 25.0          

        # ==========================================
        # 🔄 상태 및 엔코더 변수
        # ==========================================
        self.mode = 'INIT'         
        self.start_time = time.time()
        
        self.last_dist = 9999.0
        self.is_stopped = False        

        # 💡 [핵심] 아두이노에서 읽어온 누적 엔코더 값
        self.enc_l = 0
        self.enc_r = 0
        self.prev_enc_l = 0
        self.prev_enc_r = 0

        # 로봇이 계산한 주인의 "추정 상대 좌표 (x, y)"
        self.target_x = 0.0  # 정면 방향
        self.target_y = 0.0  # 좌우 방향

        self.last_send_time = time.time()
        self.send_interval = 0.1       

        # 아두이노에서 오는 엔코더 데이터를 백그라운드에서 계속 읽는 스레드 시작
        self.read_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
        if self.ser and self.ser.is_open:
            self.read_thread.start()

    def init_arduino_serial(self):
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
            time.sleep(2)  
            self.get_logger().info('✅ 아두이노 연동 완료! [엔코더 기반 스마트 오도메트리] 가동!')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')

    def apply_min_pwm(self, speed):
        if abs(speed) < 1.0: return 0
        return max(self.MIN_PWM, speed) if speed > 0 else min(-self.MIN_PWM, speed)

    # ==========================================
    # 🎧 백그라운드 스레드: 아두이노가 보내는 "E,좌측,우측" 읽기
    # ==========================================
    def serial_read_thread(self):
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith("E,"):
                        parts = line.split(',')
                        if len(parts) == 3:
                            self.enc_l = int(parts[1])
                            self.enc_r = int(parts[2])
            except Exception as e:
                pass
            time.sleep(0.01)

    def uwb_callback(self, msg: Point):
        dist_cm = msg.x  # 필터 없이 즉각적인 UWB 거리 사용
        if dist_cm <= 0.0: return  

        now = time.time()

        # 정지/출발 로직
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
        # ⏳ INIT (전원 켜고 1.5초 대기하며 안정화)
        # ==========================================
        if self.mode == 'INIT':
            self.send_motor_command(0, 0)
            if now - self.start_time > 1.5: 
                # 시작할 때 주인이 정확히 내 정면(x)에 있다고 가정하고 추적 시작!
                self.target_x = dist_cm
                self.target_y = 0.0
                self.prev_enc_l = self.enc_l
                self.prev_enc_r = self.enc_r
                self.last_dist = dist_cm
                self.mode = 'TRACK'
                self.get_logger().info(f'🚀 타겟 추정 좌표 [{self.target_x:.1f}, {self.target_y:.1f}] 로 추적 개시!')

        # ==========================================
        # 🚀 TRACK (오도메트리 기반 수학적 추종)
        # ==========================================
        elif self.mode == 'TRACK':
            
            # 1. 방금 0.1초 동안 내 바퀴가 얼마나 굴렀는지 계산
            delta_ticks_l = self.enc_l - self.prev_enc_l
            delta_ticks_r = self.enc_r - self.prev_enc_r
            self.prev_enc_l = self.enc_l
            self.prev_enc_r = self.enc_r

            dist_l_cm = delta_ticks_l / self.TICKS_PER_CM
            dist_r_cm = delta_ticks_r / self.TICKS_PER_CM
            
            # 로봇 몸통이 이동한 중심 거리 (cm)와 회전한 각도 (라디안)
            delta_dist = (dist_l_cm + dist_r_cm) / 2.0
            delta_theta = (dist_r_cm - dist_l_cm) / self.WHEEL_BASE_CM

            # 2. 로봇의 이동량을 타겟 좌표계에 반영 (상대 좌표 업데이트)
            # 내가 앞으로 가면 타겟은 나에게 다가오고, 내가 회전하면 타겟은 반대로 도는 원리
            new_target_x = self.target_x - delta_dist
            new_target_y = self.target_y
            
            # 내가 회전한 만큼 타겟의 좌표를 회전행렬로 꺾어줌
            rot_x = new_target_x * math.cos(-delta_theta) - new_target_y * math.sin(-delta_theta)
            rot_y = new_target_x * math.sin(-delta_theta) + new_target_y * math.cos(-delta_theta)
            
            self.target_x = rot_x
            self.target_y = rot_y

            # 3. 💡 [마법의 융합] UWB 거리와 내가 추정한 타겟 거리를 비교하여 교정!
            # 계산상 타겟과의 거리는 이만큼이어야 한다.
            est_dist = math.sqrt(self.target_x**2 + self.target_y**2)
            
            # 그런데 실제 UWB 센서가 재어보니 'dist_cm' 이었다!
            # 오차를 타겟 좌표에 반영하여 타겟의 위치를 스스로 수정 (Complementary Filter)
            if est_dist > 0.1:
                correction_ratio = dist_cm / est_dist
                # 맹신하지 않고, 30%만 실제 UWB 값을 반영하여 스무스하게 보정
                self.target_x = self.target_x * 0.7 + (self.target_x * correction_ratio) * 0.3
                self.target_y = self.target_y * 0.7 + (self.target_y * correction_ratio) * 0.3

            # 4. 이제 타겟이 내 정면 기준으로 몇 도(각도) 틀어져 있는지 완벽히 파악함!
            target_angle_rad = math.atan2(self.target_y, self.target_x)
            
            # 5. 모터 제어: 타겟이 있는 각도만큼 핸들을 꺾는다 (비례 제어)
            # 각도(라디안)에 P_GAIN(약 15.0)을 곱해 스티어링 파워를 만듦
            steer_power = target_angle_rad * 15.0
            steer_power = max(-self.MAX_SWING, min(self.MAX_SWING, steer_power))

            # 가속도 로직
            extra_speed = (dist_cm - 70.0) * 0.3
            current_base = self.BASE_PWM + max(0.0, min(30.0, extra_speed))

            left_motor = current_base - steer_power
            right_motor = current_base + steer_power

            self.send_motor_command(left_motor, right_motor)
            self.last_dist = dist_cm

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
