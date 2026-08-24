import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, String


class LidarAvoidance(Node):

    def __init__(self):

        super().__init__('lidar_avoidance')

        # =========================================================
        # 거리 설정
        # =========================================================

        # 전방 장애물 회피 시작거리
        self.AVOID_START_DISTANCE = 1.70

        # 이 거리 이하부터 강한 회피
        self.FULL_AVOID_DISTANCE = 0.45

        # =========================================================
        # Emergency
        # =========================================================

        # 정면 30cm 이하 -> emergency
        self.EMERGENCY_DISTANCE = 0.30

        # emergency는 정면 ±25도만
        self.EMERGENCY_HALF_ANGLE = 25.0

        # blind zone 때문에 장애물이 사라져도
        # emergency 상태를 유지할 시간
        self.EMERGENCY_HOLD_TIME = 3.00

        self.last_emergency_detect_time = -999.0

        # =========================================================
        # 전방
        # =========================================================

        self.FRONT_HALF_ANGLE = 25.0

        # =========================================================
        # 회피 Steering
        # =========================================================

        self.MAX_STEERING = 48.0

        # PASSING 상태에서 유지하는 기본 조향
        self.PASSING_STEERING = 18.0

        # 측면 장애물이 가까우면 추가 조향
        self.PASSING_MAX_STEERING = 30.0

        # =========================================================
        # 방향 변경 방지
        # =========================================================

        self.DIRECTION_SWITCH_MARGIN = 0.15

        # =========================================================
        # 장애물 통과 판단
        # =========================================================

        # 장애물이 측면에 있다고 판단하는 거리
        self.SIDE_DETECT_DISTANCE = 1.20

        # 측면에서 이 이상 떨어지면 clear 후보
        self.SIDE_CLEAR_DISTANCE = 0.80

        # 후측면도 이 이상이면 clear 후보
        self.REAR_SIDE_CLEAR_DISTANCE = 0.90

        # PASSING 상태 최소 유지시간
        self.PASS_MIN_TIME = 0.60

        # 측면/후측면 clear가 연속으로 유지될 시간
        self.PASS_CLEAR_TIME = 0.50

        # 센서 구조상 obstacle_seen을 못 잡았을 때
        # 모든 관련 영역이 완전히 clear한 경우에만 사용하는 보조시간
        self.PASS_FALLBACK_TIME = 2.50

        # =========================================================
        # State
        # =========================================================

        # TRACK
        # AVOID
        # PASSING
        # RECOVER

        self.avoid_state = 'TRACK'

        # -1 = LEFT
        #  0 = NONE
        # +1 = RIGHT
        self.avoid_direction = 0

        self.avoid_start_time = 0.0
        self.pass_start_time = 0.0
        self.pass_clear_start = None

        # PASSING 중 실제 장애물을 측면/후측면에서
        # 한 번이라도 확인했는지
        self.obstacle_seen_during_pass = False

        # RECOVER 시작시간
        self.recover_start_time = 0.0

        # RECOVER 상태 유지
        self.RECOVER_TIME = 1.00

        # =========================================================
        # Scan
        # =========================================================

        self.SCAN_MAX_DISTANCE = 2.5

        self.SECTOR_HALF_WIDTH = 18.0

        # =========================================================
        # Publishers
        # =========================================================

        self.steering_pub = self.create_publisher(
            Float32,
            '/avoidance/steering',
            10
        )

        self.threat_pub = self.create_publisher(
            Float32,
            '/avoidance/threat',
            10
        )

        self.emergency_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )

        self.front_clearance_pub = self.create_publisher(
            Float32,
            '/avoidance/front_clearance',
            10
        )

        self.rear_clearance_pub = self.create_publisher(
            Float32,
            '/avoidance/rear_clearance',
            10
        )

        # 현재 상태 확인용
        self.state_pub = self.create_publisher(
            String,
            '/avoidance/state',
            10
        )

        # 좌우 공간 확인용
        self.left_score_pub = self.create_publisher(
            Float32,
            '/avoidance/left_score',
            10
        )

        self.right_score_pub = self.create_publisher(
            Float32,
            '/avoidance/right_score',
            10
        )

        # PASSING 중 장애물이 남아있는 측면
        self.side_clearance_pub = self.create_publisher(
            Float32,
            '/avoidance/obstacle_side_clearance',
            10
        )

        self.rear_side_clearance_pub = self.create_publisher(
            Float32,
            '/avoidance/obstacle_rear_side_clearance',
            10
        )

        # =========================================================
        # Subscriber
        # =========================================================

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'LiDAR avoidance started | '
            'TRACK -> AVOID -> PASSING -> RECOVER'
        )

    # =============================================================
    # Utility
    # =============================================================

    def clamp(self, value, low, high):

        return max(
            low,
            min(
                high,
                value
            )
        )

    def normalize_angle_deg(self, angle):

        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle

    # =============================================================
    # Range
    # =============================================================

    def valid_range(self, r, msg):

        if not math.isfinite(r):
            return False

        if r <= 0.0:
            return False

        if r < msg.range_min:
            return False

        if r > msg.range_max:
            return False

        return True

    # =============================================================
    # Percentile
    # =============================================================

    def percentile(self, values, fraction):

        if not values:
            return self.SCAN_MAX_DISTANCE

        values = sorted(values)

        index = int(
            (len(values) - 1)
            *
            fraction
        )

        index = max(
            0,
            min(
                len(values) - 1,
                index
            )
        )

        return values[index]

    # =============================================================
    # Sector
    # =============================================================

    def get_sector_values(
        self,
        msg,
        center_deg,
        half_width_deg
    ):

        values = []

        angle = msg.angle_min

        for r in msg.ranges:

            deg = math.degrees(angle)

            deg = self.normalize_angle_deg(
                deg
            )

            diff = self.normalize_angle_deg(
                deg - center_deg
            )

            if abs(diff) <= half_width_deg:

                if self.valid_range(
                    r,
                    msg
                ):

                    values.append(
                        min(
                            r,
                            self.SCAN_MAX_DISTANCE
                        )
                    )

            angle += msg.angle_increment

        return values

    def sector_clearance(
        self,
        msg,
        center_deg,
        half_width_deg
    ):

        values = self.get_sector_values(
            msg,
            center_deg,
            half_width_deg
        )

        if not values:
            return self.SCAN_MAX_DISTANCE

        return self.percentile(
            values,
            0.25
        )

    # =============================================================
    # Front clearance
    # =============================================================

    def calculate_front_clearance(
        self,
        msg
    ):

        values = self.get_sector_values(
            msg,
            0.0,
            self.FRONT_HALF_ANGLE
        )

        if not values:
            return self.SCAN_MAX_DISTANCE

        # 작은 장애물도 비교적 일찍 반응
        return self.percentile(
            values,
            0.10
        )

    # =============================================================
    # Emergency
    # =============================================================

    def emergency_front_min(
        self,
        msg
    ):

        values = self.get_sector_values(
            msg,
            0.0,
            self.EMERGENCY_HALF_ANGLE
        )

        if not values:
            return None

        return min(values)

    def calculate_emergency(
        self,
        msg
    ):

        now = time.monotonic()

        minimum = self.emergency_front_min(
            msg
        )

        if (
            minimum is not None
            and
            minimum <= self.EMERGENCY_DISTANCE
        ):

            self.last_emergency_detect_time = now

            return True

        if (
            now
            -
            self.last_emergency_detect_time
            <
            self.EMERGENCY_HOLD_TIME
        ):

            return True

        return False

    # =============================================================
    # 좌우 안전공간 계산
    # =============================================================

    def calculate_side_scores(
        self,
        msg
    ):

        # ---------------------------------------------------------
        # LEFT
        # ---------------------------------------------------------

        left_30 = self.sector_clearance(
            msg,
            30.0,
            18.0
        )

        left_60 = self.sector_clearance(
            msg,
            60.0,
            18.0
        )

        left_90 = self.sector_clearance(
            msg,
            90.0,
            18.0
        )

        left_score = (
            0.50 * left_30
            +
            0.30 * left_60
            +
            0.20 * left_90
        )

        # ---------------------------------------------------------
        # RIGHT
        # ---------------------------------------------------------

        right_30 = self.sector_clearance(
            msg,
            -30.0,
            18.0
        )

        right_60 = self.sector_clearance(
            msg,
            -60.0,
            18.0
        )

        right_90 = self.sector_clearance(
            msg,
            -90.0,
            18.0
        )

        right_score = (
            0.50 * right_30
            +
            0.30 * right_60
            +
            0.20 * right_90
        )

        return (
            left_score,
            right_score
        )

    # =============================================================
    # 회피 시작
    # =============================================================

    def start_avoidance(
        self,
        left_score,
        right_score,
        now
    ):

        # 더 넓은 방향 선택
        if right_score > left_score:

            self.avoid_direction = 1

            direction_text = 'RIGHT'

        else:

            self.avoid_direction = -1

            direction_text = 'LEFT'

        self.avoid_state = 'AVOID'

        self.avoid_start_time = now

        self.pass_clear_start = None

        self.obstacle_seen_during_pass = False

        self.get_logger().info(
            f'AVOID START -> {direction_text} | '
            f'L={left_score:.2f} R={right_score:.2f}'
        )

    # =============================================================
    # AVOID steering
    # =============================================================

    def calculate_avoid_steering(
        self,
        front_clearance
    ):

        proximity = (
            self.AVOID_START_DISTANCE
            -
            front_clearance
        ) / (
            self.AVOID_START_DISTANCE
            -
            self.FULL_AVOID_DISTANCE
        )

        proximity = self.clamp(
            proximity,
            0.0,
            1.0
        )

        # 예전의 부드러운 회피 방식
        steering_ratio = (
            proximity ** 0.55
        )

        steering_magnitude = (
            self.MAX_STEERING
            *
            steering_ratio
        )

        return self.clamp(
            self.avoid_direction
            *
            steering_magnitude,
            -self.MAX_STEERING,
            self.MAX_STEERING
        )

    # =============================================================
    # PASSING 중 장애물이 어느 측면에 남는가?
    #
    # RIGHT로 피하면 장애물은 LEFT에 남음.
    # LEFT로 피하면 장애물은 RIGHT에 남음.
    # =============================================================

    def calculate_obstacle_side_clearance(
        self,
        msg
    ):

        if self.avoid_direction == 1:

            # 오른쪽으로 피하는 중
            # 장애물은 왼쪽 측면/후측면에 남음

            side = self.sector_clearance(
                msg,
                90.0,
                28.0
            )

            rear_side = self.sector_clearance(
                msg,
                135.0,
                25.0
            )

        else:

            # 왼쪽으로 피하는 중
            # 장애물은 오른쪽에 남음

            side = self.sector_clearance(
                msg,
                -90.0,
                28.0
            )

            rear_side = self.sector_clearance(
                msg,
                -135.0,
                25.0
            )

        return (
            side,
            rear_side
        )

    # =============================================================
    # PASSING 중 진행방향 쪽도 확인
    #
    # 예:
    # RIGHT로 돌고 있는데 우측 벽이 너무 가까우면
    # 계속 강하게 오른쪽으로 꺾으면 안 됨.
    # =============================================================

    def calculate_path_clearance(
        self,
        msg
    ):

        if self.avoid_direction == 1:

            return self.sector_clearance(
                msg,
                -55.0,
                25.0
            )

        return self.sector_clearance(
            msg,
            55.0,
            25.0
        )

    # =============================================================
    # PASSING steering
    # =============================================================

    def calculate_passing_steering(
        self,
        msg,
        obstacle_side
    ):

        # 기본적으로 선택한 회피방향을 유지
        magnitude = self.PASSING_STEERING

        # ---------------------------------------------------------
        # 장애물이 측면에 가까울수록
        # 원래 회피 방향으로 조금 더 밀어냄
        # ---------------------------------------------------------

        if obstacle_side < 0.80:

            proximity = (
                0.80
                -
                obstacle_side
            ) / 0.50

            proximity = self.clamp(
                proximity,
                0.0,
                1.0
            )

            magnitude += (
                12.0
                *
                proximity
            )

        magnitude = min(
            magnitude,
            self.PASSING_MAX_STEERING
        )

        # ---------------------------------------------------------
        # 진행하려는 방향 자체가 막혀있으면
        # 너무 강한 조향을 제한
        # ---------------------------------------------------------

        path_clearance = (
            self.calculate_path_clearance(
                msg
            )
        )

        if path_clearance < 0.45:

            magnitude *= 0.35

        elif path_clearance < 0.65:

            magnitude *= 0.65

        return (
            self.avoid_direction
            *
            magnitude
        )

    # =============================================================
    # Threat
    # =============================================================

    def calculate_threat(
        self,
        front_clearance
    ):

        if (
            front_clearance
            >=
            self.AVOID_START_DISTANCE
        ):

            return 0.0

        if (
            front_clearance
            <=
            self.EMERGENCY_DISTANCE
        ):

            return 1.0

        threat = (
            self.AVOID_START_DISTANCE
            -
            front_clearance
        ) / (
            self.AVOID_START_DISTANCE
            -
            self.EMERGENCY_DISTANCE
        )

        return self.clamp(
            threat,
            0.0,
            1.0
        )

    # =============================================================
    # State machine
    # =============================================================

    def update_avoidance(
        self,
        msg,
        front_clearance
    ):

        now = time.monotonic()

        left_score, right_score = (
            self.calculate_side_scores(
                msg
            )
        )

        obstacle_side = self.SCAN_MAX_DISTANCE
        obstacle_rear_side = self.SCAN_MAX_DISTANCE

        steering = 0.0

        # =========================================================
        # TRACK
        # =========================================================

        if self.avoid_state == 'TRACK':

            if (
                front_clearance
                <
                self.AVOID_START_DISTANCE
            ):

                self.start_avoidance(
                    left_score,
                    right_score,
                    now
                )

                steering = (
                    self.calculate_avoid_steering(
                        front_clearance
                    )
                )

            else:

                steering = 0.0

        # =========================================================
        # AVOID
        # =========================================================

        elif self.avoid_state == 'AVOID':

            steering = (
                self.calculate_avoid_steering(
                    front_clearance
                )
            )

            # -----------------------------------------------------
            # 정면 장애물이 충분히 빠졌다고 판단되면
            #
            # 여기서 TRACK으로 돌아가지 않고
            # PASSING으로 이동
            # -----------------------------------------------------

            if front_clearance > 1.05:

                self.avoid_state = 'PASSING'

                self.pass_start_time = now

                self.pass_clear_start = None

                self.obstacle_seen_during_pass = False

                self.get_logger().info(
                    'AVOID -> PASSING'
                )

        # =========================================================
        # PASSING
        # =========================================================

        elif self.avoid_state == 'PASSING':

            (
                obstacle_side,
                obstacle_rear_side
            ) = (
                self.calculate_obstacle_side_clearance(
                    msg
                )
            )

            # -----------------------------------------------------
            # 장애물이 실제로 측면/후측면에 들어오는지 확인
            # -----------------------------------------------------

            if (
                obstacle_side
                <
                self.SIDE_DETECT_DISTANCE
                or
                obstacle_rear_side
                <
                self.SIDE_DETECT_DISTANCE
            ):

                self.obstacle_seen_during_pass = True

            # -----------------------------------------------------
            # 장애물이 옆에 있는 동안 선택한 회피 방향 유지
            # -----------------------------------------------------

            steering = (
                self.calculate_passing_steering(
                    msg,
                    obstacle_side
                )
            )

            pass_elapsed = (
                now
                -
                self.pass_start_time
            )

            # -----------------------------------------------------
            # 장애물이 측면/후측면에서 충분히 멀어졌는지
            # -----------------------------------------------------

            side_clear = (
                obstacle_side
                >=
                self.SIDE_CLEAR_DISTANCE
            )

            rear_side_clear = (
                obstacle_rear_side
                >=
                self.REAR_SIDE_CLEAR_DISTANCE
            )

            front_clear = (
                front_clearance
                >
                1.00
            )

            all_clear = (
                side_clear
                and
                rear_side_clear
                and
                front_clear
            )

            # -----------------------------------------------------
            # 최소 PASSING 시간
            # -----------------------------------------------------

            if pass_elapsed < self.PASS_MIN_TIME:

                self.pass_clear_start = None

            else:

                # -------------------------------------------------
                # 정상적인 경우:
                # 장애물을 실제 옆/후측면에서 본 뒤
                # 사라져야 통과 완료
                # -------------------------------------------------

                normal_clear = (
                    self.obstacle_seen_during_pass
                    and
                    all_clear
                )

                # -------------------------------------------------
                # 보조조건:
                # 센서 기하 때문에 측면 obstacle_seen을
                # 못 잡았더라도 2.5초 이상 지났고
                # 모든 관련 방향이 충분히 clear할 때만 허용
                # -------------------------------------------------

                fallback_clear = (
                    pass_elapsed
                    >
                    self.PASS_FALLBACK_TIME
                    and
                    all_clear
                )

                if (
                    normal_clear
                    or
                    fallback_clear
                ):

                    if self.pass_clear_start is None:

                        self.pass_clear_start = now

                    elif (
                        now
                        -
                        self.pass_clear_start
                        >=
                        self.PASS_CLEAR_TIME
                    ):

                        self.avoid_state = 'RECOVER'

                        self.recover_start_time = now

                        self.pass_clear_start = None

                        steering = 0.0

                        self.get_logger().info(
                            'PASSING -> RECOVER'
                        )

                else:

                    self.pass_clear_start = None

        # =========================================================
        # RECOVER
        # =========================================================

        elif self.avoid_state == 'RECOVER':

            steering = 0.0

            # -----------------------------------------------------
            # 복귀 중 새로운 전방 장애물
            # -----------------------------------------------------

            if (
                front_clearance
                <
                self.AVOID_START_DISTANCE
            ):

                self.avoid_state = 'TRACK'

                self.avoid_direction = 0

                self.start_avoidance(
                    left_score,
                    right_score,
                    now
                )

                steering = (
                    self.calculate_avoid_steering(
                        front_clearance
                    )
                )

            elif (
                now
                -
                self.recover_start_time
                >=
                self.RECOVER_TIME
            ):

                self.avoid_state = 'TRACK'

                self.avoid_direction = 0

                self.obstacle_seen_during_pass = False

                self.get_logger().info(
                    'RECOVER -> TRACK'
                )

        return (
            steering,
            left_score,
            right_score,
            obstacle_side,
            obstacle_rear_side
        )

    # =============================================================
    # Scan callback
    # =============================================================

    def scan_callback(
        self,
        msg
    ):

        # =========================================================
        # Front
        # =========================================================

        front_clearance = (
            self.calculate_front_clearance(
                msg
            )
        )

        # =========================================================
        # Rear
        # =========================================================

        rear_clearance = (
            self.sector_clearance(
                msg,
                180.0,
                25.0
            )
        )

        # =========================================================
        # Emergency
        # =========================================================

        emergency = (
            self.calculate_emergency(
                msg
            )
        )

        # =========================================================
        # Avoidance
        # =========================================================

        (
            steering,
            left_score,
            right_score,
            obstacle_side,
            obstacle_rear_side
        ) = self.update_avoidance(
            msg,
            front_clearance
        )

        # =========================================================
        # Threat
        # =========================================================

        threat = (
            self.calculate_threat(
                front_clearance
            )
        )

        # =========================================================
        # Publish
        # =========================================================

        m = Float32()
        m.data = float(steering)
        self.steering_pub.publish(m)

        m = Float32()
        m.data = float(threat)
        self.threat_pub.publish(m)

        m = Bool()
        m.data = bool(emergency)
        self.emergency_pub.publish(m)

        m = Float32()
        m.data = float(front_clearance)
        self.front_clearance_pub.publish(m)

        m = Float32()
        m.data = float(rear_clearance)
        self.rear_clearance_pub.publish(m)

        m = String()
        m.data = self.avoid_state
        self.state_pub.publish(m)

        m = Float32()
        m.data = float(left_score)
        self.left_score_pub.publish(m)

        m = Float32()
        m.data = float(right_score)
        self.right_score_pub.publish(m)

        m = Float32()
        m.data = float(obstacle_side)
        self.side_clearance_pub.publish(m)

        m = Float32()
        m.data = float(obstacle_rear_side)
        self.rear_side_clearance_pub.publish(m)


def main(args=None):

    rclpy.init(args=args)

    node = LidarAvoidance()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()