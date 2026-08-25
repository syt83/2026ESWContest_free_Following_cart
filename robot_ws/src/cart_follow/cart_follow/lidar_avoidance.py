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

        # 테스트:
        # UWB는 사용자 1.30m에서 정지
        # 일반 장애물 회피는 1.20m부터
        self.AVOID_START_DISTANCE = 1.20

        self.FULL_AVOID_DISTANCE = 0.45

        # =========================================================
        # Emergency
        # =========================================================

        self.EMERGENCY_DISTANCE = 0.30

        self.EMERGENCY_HALF_ANGLE = 25.0

        self.EMERGENCY_HOLD_TIME = 3.00

        self.last_emergency_detect_time = -999.0

        # =========================================================
        # Front
        # =========================================================

        self.FRONT_HALF_ANGLE = 25.0

        # =========================================================
        # Steering
        # =========================================================

        self.MAX_STEERING = 48.0

        self.PASSING_STEERING = 18.0

        self.PASSING_MAX_STEERING = 30.0

        # =========================================================
        # Direction
        # =========================================================

        self.DIRECTION_SWITCH_MARGIN = 0.15

        # =========================================================
        # PASSING
        # =========================================================

        self.SIDE_DETECT_DISTANCE = 1.20

        self.SIDE_CLEAR_DISTANCE = 0.80

        self.REAR_SIDE_CLEAR_DISTANCE = 0.90

        self.PASS_MIN_TIME = 0.60

        self.PASS_CLEAR_TIME = 0.50

        self.PASS_FALLBACK_TIME = 2.50

        # =========================================================
        # State
        # =========================================================

        self.avoid_state = 'TRACK'

        # -1 = LEFT
        #  0 = NONE
        # +1 = RIGHT
        self.avoid_direction = 0

        self.avoid_start_time = 0.0

        self.pass_start_time = 0.0

        self.pass_clear_start = None

        self.obstacle_seen_during_pass = False

        self.recover_start_time = 0.0

        self.RECOVER_TIME = 1.00

        # =========================================================
        # NEW
        #
        # Emergency 중 일반 LiDAR 회피 비활성화
        #
        # True일 때:
        #
        # steering = 0
        # threat = 0
        # AVOID/PASSING 진행 X
        #
        # 하지만:
        #
        # emergency 계산 O
        # front clearance O
        # rear clearance O
        #
        # 즉 안전센서는 계속 살아있음.
        # =========================================================

        self.avoidance_disabled = False

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

        self.state_pub = self.create_publisher(
            String,
            '/avoidance/state',
            10
        )

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
        # Subscribers
        # =========================================================

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # NEW:
        # follow_controller에서 Emergency 동안
        # 일반 회피 기능을 ON/OFF
        self.create_subscription(
            Bool,
            '/avoidance/disable',
            self.disable_callback,
            10
        )

        self.get_logger().info(
            'LiDAR avoidance started | '
            'AVOID=1.20m | '
            'Emergency=0.30m | '
            'Emergency reset protection ON'
        )

    # =============================================================
    # Utility
    # =============================================================

    def clamp(
        self,
        value,
        low,
        high
    ):

        return max(
            low,
            min(
                high,
                value
            )
        )

    def normalize_angle_deg(
        self,
        angle
    ):

        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle

    # =============================================================
    # NEW
    # LiDAR avoidance reset
    # =============================================================

    def reset_avoidance_state(self):

        self.avoid_state = 'TRACK'

        self.avoid_direction = 0

        self.avoid_start_time = 0.0

        self.pass_start_time = 0.0

        self.pass_clear_start = None

        self.obstacle_seen_during_pass = False

        self.recover_start_time = 0.0

    # =============================================================
    # NEW
    # Emergency disable callback
    # =============================================================

    def disable_callback(
        self,
        msg
    ):

        new_disabled = bool(
            msg.data
        )

        # ---------------------------------------------------------
        # 일반 회피 OFF
        # ---------------------------------------------------------

        if (
            new_disabled
            and
            not self.avoidance_disabled
        ):

            self.avoidance_disabled = True

            self.reset_avoidance_state()

            self.get_logger().warn(
                'NORMAL AVOIDANCE DISABLED | '
                'state reset'
            )

        # ---------------------------------------------------------
        # 일반 회피 다시 ON
        # ---------------------------------------------------------

        elif (
            not new_disabled
            and
            self.avoidance_disabled
        ):

            # 이전 상태를 절대로 이어받지 않음
            self.reset_avoidance_state()

            self.avoidance_disabled = False

            self.get_logger().info(
                'NORMAL AVOIDANCE ENABLED | '
                'fresh TRACK'
            )

    # =============================================================
    # Range
    # =============================================================

    def valid_range(
        self,
        r,
        msg
    ):

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

    def percentile(
        self,
        values,
        fraction
    ):

        if not values:
            return self.SCAN_MAX_DISTANCE

        values = sorted(
            values
        )

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

            deg = math.degrees(
                angle
            )

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

        return self.percentile(
            values,
            0.10
        )

    # =============================================================
    # Emergency
    #
    # 주의:
    # avoidance_disabled와 상관없이 항상 실행
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

        return min(
            values
        )

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
    # Side scores
    # =============================================================

    def calculate_side_scores(
        self,
        msg
    ):

        # LEFT

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

        # RIGHT

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
    # Start avoidance
    # =============================================================

    def start_avoidance(
        self,
        left_score,
        right_score,
        now
    ):

        # 오른쪽 공간이 더 넓음
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
            f'L={left_score:.2f} '
            f'R={right_score:.2f}'
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
    # PASSING obstacle side
    # =============================================================

    def calculate_obstacle_side_clearance(
        self,
        msg
    ):

        if self.avoid_direction == 1:

            # RIGHT 회피
            # 장애물은 LEFT에 남음

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

            # LEFT 회피
            # 장애물은 RIGHT에 남음

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
    # PASSING path clearance
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

        magnitude = (
            self.PASSING_STEERING
        )

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
    # Avoidance state machine
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

        obstacle_side = (
            self.SCAN_MAX_DISTANCE
        )

        obstacle_rear_side = (
            self.SCAN_MAX_DISTANCE
        )

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

            if front_clearance > 1.05:

                self.avoid_state = (
                    'PASSING'
                )

                self.pass_start_time = (
                    now
                )

                self.pass_clear_start = None

                self.obstacle_seen_during_pass = (
                    False
                )

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

            if (
                obstacle_side
                <
                self.SIDE_DETECT_DISTANCE
                or
                obstacle_rear_side
                <
                self.SIDE_DETECT_DISTANCE
            ):

                self.obstacle_seen_during_pass = (
                    True
                )

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

            if (
                pass_elapsed
                <
                self.PASS_MIN_TIME
            ):

                self.pass_clear_start = None

            else:

                normal_clear = (
                    self.obstacle_seen_during_pass
                    and
                    all_clear
                )

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

                        self.pass_clear_start = (
                            now
                        )

                    elif (
                        now
                        -
                        self.pass_clear_start
                        >=
                        self.PASS_CLEAR_TIME
                    ):

                        self.avoid_state = (
                            'RECOVER'
                        )

                        self.recover_start_time = (
                            now
                        )

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

            if (
                front_clearance
                <
                self.AVOID_START_DISTANCE
            ):

                self.avoid_state = (
                    'TRACK'
                )

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

                self.avoid_state = (
                    'TRACK'
                )

                self.avoid_direction = 0

                self.obstacle_seen_during_pass = (
                    False
                )

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
        # SAFETY DATA
        #
        # 이 부분은 일반 회피 disable 상태에서도
        # 반드시 계속 계산한다.
        # =========================================================

        front_clearance = (
            self.calculate_front_clearance(
                msg
            )
        )

        rear_clearance = (
            self.sector_clearance(
                msg,
                180.0,
                25.0
            )
        )

        emergency = (
            self.calculate_emergency(
                msg
            )
        )

        # =========================================================
        # 일반 회피
        # =========================================================

        if self.avoidance_disabled:

            # Emergency/후진 중에는
            # 예전 LEFT/RIGHT 회피를 절대 유지하지 않는다.

            self.reset_avoidance_state()

            steering = 0.0
            threat = 0.0

            left_score = (
                self.SCAN_MAX_DISTANCE
            )

            right_score = (
                self.SCAN_MAX_DISTANCE
            )

            obstacle_side = (
                self.SCAN_MAX_DISTANCE
            )

            obstacle_rear_side = (
                self.SCAN_MAX_DISTANCE
            )

        else:

            (
                steering,
                left_score,
                right_score,
                obstacle_side,
                obstacle_rear_side
            ) = (
                self.update_avoidance(
                    msg,
                    front_clearance
                )
            )

            threat = (
                self.calculate_threat(
                    front_clearance
                )
            )

        # =========================================================
        # Publish
        # =========================================================

        m = Float32()

        m.data = float(
            steering
        )

        self.steering_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = Float32()

        m.data = float(
            threat
        )

        self.threat_pub.publish(
            m
        )

        # ---------------------------------------------------------
        # Emergency는 disable과 무관하게 publish
        # ---------------------------------------------------------

        m = Bool()

        m.data = bool(
            emergency
        )

        self.emergency_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = Float32()

        m.data = float(
            front_clearance
        )

        self.front_clearance_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = Float32()

        m.data = float(
            rear_clearance
        )

        self.rear_clearance_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = String()

        if self.avoidance_disabled:

            m.data = 'DISABLED'

        else:

            m.data = self.avoid_state

        self.state_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = Float32()

        m.data = float(
            left_score
        )

        self.left_score_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = Float32()

        m.data = float(
            right_score
        )

        self.right_score_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = Float32()

        m.data = float(
            obstacle_side
        )

        self.side_clearance_pub.publish(
            m
        )

        # ---------------------------------------------------------

        m = Float32()

        m.data = float(
            obstacle_rear_side
        )

        self.rear_side_clearance_pub.publish(
            m
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = LidarAvoidance()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()