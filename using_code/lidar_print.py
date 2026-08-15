# 라이다를 이용해서 장애물을 회피하는 코드
# 1. 가상환경 비활성화
#deactivate
# 2. ROS 2 기본 환경 로드
#source /opt/ros/jazzy/setup.bash
# 3. 라이다 패키지가 빌드된 워크스페이스 환경 로드 (핵심!)
#source ~/dev_ws/install/setup.bash
# 4. 라이다 포트 권한 부여
#sudo chmod 666 /dev/ttyUSB0
# 5. 라이다 실행
#ros2 launch ydlidar_ros2_driver ydlidar_launch.py
# 다른 터미널로 python3 lidar_print.py 을 켜서 거리별 라이다 하기
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import serial
import time
import math

class CartContinuousDualPD(Node):
    def __init__(self):
        super().__init__('cart_continuous_dual_pd')
        
        # 1. 아두이노 시리얼 연결
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)
            self.get_logger().info("✅ 아두이노 시리얼 연결 성공!")
        except Exception as e:
            self.get_logger().error(f"❌ 시리얼 연결 실패: {e}")
            self.ser = None

        # 2. 라이다 구독
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 📌 2중 PD 제어 게인 (테스트 속도 30 맞춤)
        self.Kp_dist = 60.0     # 거리 비례 게인
        self.Kd_dist = 12.0     # 거리 미분 게인
        self.Kp_angle = 70.0    # 각도 비례 게인
        self.Kd_angle = 15.0    # 각도 미분 게인

        self.prev_Ed = 0.0
        self.prev_Ea = 0.0
        self.last_time = time.time()

        # 🎯 테스트 주행 및 감지 설정
        self.BASE_SPEED = 30          # 🟢 테스트용 저속 설정 (30)
        self.MIN_SPEED = -30          # 안쪽 바퀴 역회전 허용 범위
        self.MAX_SPEED = 80           # 바깥쪽 바퀴 최대 속도
        
        self.SAFE_DISTANCE = 0.80     # 🟢 장애물 감지 기준 거리 (80cm)
        self.CRITICAL_DISTANCE = 0.20 # 비상 정지 거리 (20cm)
        self.MAX_FIELD_ANGLE = 60.0   # 감지 영역 반각 (총 120도)

        # 🔄 조향 방향 반전 설정 (-1.0: 반대 방향 회피, 1.0: 정방향)
        self.DIR_INVERT = -1.0

        # 회피 방향 고정 및 복귀 필터 변수
        self.locked_dir = 0.0
        self.clear_counter = 0
        self.HOLD_CYCLES = 5

    def send_speeds(self, left_speed, right_speed):
        if self.ser and self.ser.is_open:
            cmd = f"{int(left_speed)},{int(right_speed)}\n"
            self.ser.write(cmd.encode('utf-8'))

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) == 0:
            return

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.05

        max_threat = 0.0
        critical_flag = False
        
        target_dist = 0.80
        target_angle = 0.0

        left_free_dists = []
        right_free_dists = []

        # ==========================================
        # 연속 데이터 수집 및 최우선 장애물 추출
        # ==========================================
        for i, r in enumerate(ranges):
            if 0.05 < r < 3.5:
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad) + 180.0  # 180도 반전 보정
                
                while angle_deg > 180: angle_deg -= 360
                while angle_deg < -180: angle_deg += 360

                abs_angle = abs(angle_deg)

                # 측면 개활지 탐색 (좌/우 여유 공간)
                if 20.0 < angle_deg <= 90.0:
                    left_free_dists.append(r)
                elif -90.0 <= angle_deg < -20.0:
                    right_free_dists.append(r)

                # 감지 영역 (80cm 이내, ±60도 이내)
                if r < self.SAFE_DISTANCE and abs_angle <= self.MAX_FIELD_ANGLE:
                    if r < self.CRITICAL_DISTANCE:
                        critical_flag = True

                    # 연속 에러 구성요소 계산
                    ed_i = (self.SAFE_DISTANCE - r) / self.SAFE_DISTANCE
                    ea_i = (self.MAX_FIELD_ANGLE - abs_angle) / self.MAX_FIELD_ANGLE
                    
                    # 위협도 = 거리 에러 * 각도 에러
                    threat_i = ed_i * ea_i
                    
                    if threat_i > max_threat:
                        max_threat = threat_i
                        target_dist = r
                        target_angle = angle_deg

        # ==========================================
        # 1. 비상 정지
        # ==========================================
        if critical_flag:
            self.send_speeds(0, 0)
            self.get_logger().warn(f"🚨 [비상정지] 장애물 극초근접 ({target_dist:.2f}m / {target_angle:+.1f}°)")
            self.prev_Ed = 0.0
            self.prev_Ea = 0.0
            self.locked_dir = 0.0
            return

        # ==========================================
        # 2. 연속 에러(Ed, Ea) 및 회피 방향 연산 (+/- 부호 반전 적용)
        # ==========================================
        if max_threat > 0.0:
            self.clear_counter = 0

            # 연속 수학 함수에 의한 거리/각도 에러
            Ed = (self.SAFE_DISTANCE - target_dist) / self.SAFE_DISTANCE
            Ea = (self.MAX_FIELD_ANGLE - abs(target_angle)) / self.MAX_FIELD_ANGLE

            # 🛠️ 회피 방향 정의 (부호 반전 로직 반영)
            if self.locked_dir == 0.0:
                if target_angle > 2.0:
                    # 장애물이 좌측(+각도) -> 우회전으로 피함
                    self.locked_dir = 1.0 * self.DIR_INVERT
                elif target_angle < -2.0:
                    # 장애물이 우측(-각도) -> 좌회전으로 피함
                    self.locked_dir = -1.0 * self.DIR_INVERT
                else:
                    # 정확히 정중앙(0도) -> 좌/우 측면 공간 최댓값 비교 후 넓은 쪽 선택
                    d_left_max = max(left_free_dists) if left_free_dists else 3.5
                    d_right_max = max(right_free_dists) if right_free_dists else 3.5
                    base_dir = 1.0 if d_right_max >= d_left_max else -1.0
                    self.locked_dir = base_dir * self.DIR_INVERT

        else:
            # 장애물이 감지 영역 벗어남 -> 직진 복귀
            self.clear_counter += 1
            if self.clear_counter < self.HOLD_CYCLES:
                Ed = self.prev_Ed * 0.70
                Ea = self.prev_Ea * 0.70
            else:
                Ed = 0.0
                Ea = 0.0
                self.locked_dir = 0.0

        # ==========================================
        # 3. 2중 PD 제어 연산 (Distance PD & Angle PD)
        # ==========================================
        dEd = (Ed - self.prev_Ed) / dt
        dEa = (Ea - self.prev_Ea) / dt

        pd_dist = (self.Kp_dist * Ed) + (self.Kd_dist * dEd)
        pd_angle = (self.Kp_angle * Ea) + (self.Kd_angle * dEa)

        # 최종 조향량 계산
        total_steering = (pd_dist * Ea + pd_angle * Ed) * self.locked_dir

        self.prev_Ed = Ed
        self.prev_Ea = Ea
        self.last_time = current_time

        # ==========================================
        # 4. 차동 모터 PWM 출력 분배
        # ==========================================
        left_speed = self.BASE_SPEED + total_steering
        right_speed = self.BASE_SPEED - total_steering

        left_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, left_speed))
        right_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, right_speed))

        self.send_speeds(left_speed, right_speed)

        # ==========================================
        # 5. 실시간 상태 및 모니터링 텔레메트리 출력
        # ==========================================
        # 실제 차량 회전 방향 계산
        if left_speed > right_speed:
            actual_turn = "RIGHT(우회전)"
        elif right_speed > left_speed:
            actual_turn = "LEFT(좌회전)"
        else:
            actual_turn = "STRAIGHT(직진)"

        if max_threat > 0.0:
            obs_info = f"Dist:{target_dist:.2f}m | Angle:{target_angle:+.1f}°"
        else:
            obs_info = "Clear (장애물 없음)"

        self.get_logger().info(
            f"[{actual_turn}] {obs_info} || Ed:{Ed:.2f} | Ea:{Ea:.2f} | Steer:{total_steering:+.1f} | PWM(L/R):{int(left_speed)}/{int(right_speed)}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CartContinuousDualPD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b"0,0\n")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()