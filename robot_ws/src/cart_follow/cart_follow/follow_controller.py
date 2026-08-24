import time
import statistics
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    Float32,
    String,
    Bool,
    Int32MultiArray
)


class FollowController(Node):

    def __init__(self):

        super().__init__('follow_controller')

        # =========================================================
        # 사용자 거리
        # =========================================================

        self.STOP_DISTANCE = 80.0
        self.RESTART_DISTANCE = 90.0

        self.SLOW_DISTANCE = 130.0
        self.NORMAL_DISTANCE = 220.0

        self.MAX_TRACK_DISTANCE = 600.0

        # =========================================================
        # Speed
        # =========================================================

        self.SLOW_SPEED = 38.0
        self.NORMAL_SPEED = 65.0
        self.MAX_SPEED = 85.0

        # =========================================================
        # UWB
        # =========================================================

        self.K_UWB = 0.28

        self.MAX_UWB_TURN = 22.0

        self.UWB_SIGN = 1.0

        # =========================================================
        # UWB Filter
        # =========================================================

        self.UWB_MEDIAN_SIZE = 5

        self.uwb_error_buffer = deque(
            maxlen=self.UWB_MEDIAN_SIZE
        )

        self.UWB_ALPHA = 0.4

        self.UWB_DEADBAND = 5.0

        self.UWB_MAX_ERROR_STEP = 8.0

        self.ema_direction_error = 0.0

        self.filtered_direction_error = 0.0

        # =========================================================
        # LiDAR
        # =========================================================

        self.AVOID_GAIN = 1.0

        self.MAX_LIDAR_TURN = 45.0

        self.MAX_TOTAL_TURN = 55.0

        # =========================================================
        # 회피 중 UWB 영향
        # =========================================================

        self.UWB_SCALE_TRACK = 1.00

        self.UWB_SCALE_AVOID = 0.20

        self.UWB_SCALE_PASSING = 0.15

        # RECOVER 시작 시
        self.UWB_SCALE_RECOVER_START = 0.15

        # UWB 복귀시간
        self.UWB_RECOVER_TIME = 1.00

        # =========================================================
        # Emergency
        # =========================================================

        self.EMERGENCY_STOP_TIME = 1.00

        self.REVERSE_SPEED = -45.0

        self.REVERSE_TIME = 2.00

        # =========================================================
        # 후방 안전
        # =========================================================

        # 후방 80cm 확보
        self.REAR_SAFE_DISTANCE = 0.80

        # 후진 중 30cm 이하 즉시 정지
        self.REAR_STOP_DISTANCE = 0.30

        # =========================================================
        # Emergency 재무장
        # =========================================================

        self.EMERGENCY_CLEAR_TIME = 0.30

        # =========================================================
        # Timeout
        # =========================================================

        self.UWB_TIMEOUT = 0.70
        self.LIDAR_TIMEOUT = 0.50

        # =========================================================
        # UWB state
        # =========================================================

        self.raw_direction_error = 0.0

        self.user_distance = 9999.0

        self.direction = 'LOST'

        self.last_uwb = 0.0

        # =========================================================
        # LiDAR state
        # =========================================================

        self.lidar_steering = 0.0

        self.lidar_threat = 0.0

        self.emergency = False

        self.front_clearance = 0.0

        self.rear_clearance = 0.0

        self.last_lidar = 0.0

        # 회피 상태
        self.avoidance_state = 'TRACK'

        self.previous_avoidance_state = 'TRACK'

        self.recover_start_time = 0.0

        # =========================================================
        # User stop
        # =========================================================

        self.stopped_for_user = False

        # =========================================================
        # Emergency state
        # =========================================================

        self.emergency_state = 'TRACK'

        self.emergency_start_time = 0.0

        self.reverse_start_time = 0.0

        self.emergency_armed = True

        self.emergency_clear_start = None

        # =========================================================
        # Subscribers
        # =========================================================

        self.create_subscription(
            Float32,
            '/uwb/direction_error',
            self.cb_direction_error,
            10
        )

        self.create_subscription(
            Float32,
            '/uwb/user_distance',
            self.cb_user_distance,
            10
        )

        self.create_subscription(
            String,
            '/uwb/direction',
            self.cb_direction,
            10
        )

        self.create_subscription(
            Float32,
            '/avoidance/steering',
            self.cb_lidar_steering,
            10
        )

        self.create_subscription(
            Float32,
            '/avoidance/threat',
            self.cb_lidar_threat,
            10
        )

        self.create_subscription(
            Bool,
            '/emergency_stop',
            self.cb_emergency,
            10
        )

        self.create_subscription(
            Float32,
            '/avoidance/front_clearance',
            self.cb_front_clearance,
            10
        )

        self.create_subscription(
            Float32,
            '/avoidance/rear_clearance',
            self.cb_rear_clearance,
            10
        )

        self.create_subscription(
            String,
            '/avoidance/state',
            self.cb_avoidance_state,
            10
        )

        # =========================================================
        # Publisher
        # =========================================================

        self.motor_pub = self.create_publisher(
            Int32MultiArray,
            '/motor_cmd_pwm',
            10
        )

        self.filtered_error_pub = self.create_publisher(
            Float32,
            '/uwb/filtered_direction_error',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Follow controller started | '
            'obstacle passing protection ON'
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

    # =============================================================
    # UWB
    # =============================================================

    def cb_direction_error(
        self,
        msg
    ):

        raw = float(msg.data)

        self.raw_direction_error = raw

        self.uwb_error_buffer.append(
            raw
        )

        median_error = statistics.median(
            self.uwb_error_buffer
        )

        self.ema_direction_error = (
            self.UWB_ALPHA
            *
            median_error
            +
            (
                1.0
                -
                self.UWB_ALPHA
            )
            *
            self.ema_direction_error
        )

        if (
            abs(
                self.ema_direction_error
            )
            <
            self.UWB_DEADBAND
        ):

            target = 0.0

        else:

            target = (
                self.ema_direction_error
            )

        delta = (
            target
            -
            self.filtered_direction_error
        )

        delta = self.clamp(
            delta,
            -self.UWB_MAX_ERROR_STEP,
            self.UWB_MAX_ERROR_STEP
        )

        self.filtered_direction_error += (
            delta
        )

        self.last_uwb = time.monotonic()

        m = Float32()

        m.data = float(
            self.filtered_direction_error
        )

        self.filtered_error_pub.publish(
            m
        )

    def cb_user_distance(
        self,
        msg
    ):

        self.user_distance = float(
            msg.data
        )

        self.last_uwb = time.monotonic()

    def cb_direction(
        self,
        msg
    ):

        self.direction = str(
            msg.data
        )

        self.last_uwb = time.monotonic()

    # =============================================================
    # LiDAR
    # =============================================================

    def cb_lidar_steering(
        self,
        msg
    ):

        self.lidar_steering = float(
            msg.data
        )

        self.last_lidar = time.monotonic()

    def cb_lidar_threat(
        self,
        msg
    ):

        self.lidar_threat = float(
            msg.data
        )

        self.last_lidar = time.monotonic()

    def cb_emergency(
        self,
        msg
    ):

        self.emergency = bool(
            msg.data
        )

        self.last_lidar = time.monotonic()

    def cb_front_clearance(
        self,
        msg
    ):

        self.front_clearance = float(
            msg.data
        )

        self.last_lidar = time.monotonic()

    def cb_rear_clearance(
        self,
        msg
    ):

        self.rear_clearance = float(
            msg.data
        )

        self.last_lidar = time.monotonic()

    def cb_avoidance_state(
        self,
        msg
    ):

        new_state = str(
            msg.data
        )

        if (
            new_state !=
            self.avoidance_state
        ):

            self.previous_avoidance_state = (
                self.avoidance_state
            )

            self.avoidance_state = (
                new_state
            )

            if new_state == 'RECOVER':

                self.recover_start_time = (
                    time.monotonic()
                )

        self.last_lidar = time.monotonic()

    # =============================================================
    # Motor
    # =============================================================

    def publish_motor(
        self,
        left,
        right
    ):

        msg = Int32MultiArray()

        msg.data = [
            int(
                self.clamp(
                    left,
                    -255,
                    255
                )
            ),
            int(
                self.clamp(
                    right,
                    -255,
                    255
                )
            )
        ]

        self.motor_pub.publish(
            msg
        )

    # =============================================================
    # Speed
    # =============================================================

    def calculate_speed(
        self
    ):

        d = self.user_distance

        if d <= self.STOP_DISTANCE:

            return 0.0

        if d <= self.SLOW_DISTANCE:

            ratio = (
                d
                -
                self.STOP_DISTANCE
            ) / (
                self.SLOW_DISTANCE
                -
                self.STOP_DISTANCE
            )

            ratio = self.clamp(
                ratio,
                0.0,
                1.0
            )

            return (
                self.SLOW_SPEED
                *
                ratio
            )

        if d <= self.NORMAL_DISTANCE:

            ratio = (
                d
                -
                self.SLOW_DISTANCE
            ) / (
                self.NORMAL_DISTANCE
                -
                self.SLOW_DISTANCE
            )

            ratio = self.clamp(
                ratio,
                0.0,
                1.0
            )

            return (
                self.SLOW_SPEED
                +
                (
                    self.NORMAL_SPEED
                    -
                    self.SLOW_SPEED
                )
                *
                ratio
            )

        return self.MAX_SPEED

    # =============================================================
    # UWB scale
    # =============================================================

    def calculate_uwb_scale(
        self,
        now
    ):

        if self.avoidance_state == 'AVOID':

            return self.UWB_SCALE_AVOID

        if self.avoidance_state == 'PASSING':

            return self.UWB_SCALE_PASSING

        if self.avoidance_state == 'RECOVER':

            elapsed = (
                now
                -
                self.recover_start_time
            )

            ratio = self.clamp(
                elapsed
                /
                self.UWB_RECOVER_TIME,
                0.0,
                1.0
            )

            return (
                self.UWB_SCALE_RECOVER_START
                +
                (
                    1.0
                    -
                    self.UWB_SCALE_RECOVER_START
                )
                *
                ratio
            )

        return self.UWB_SCALE_TRACK

    # =============================================================
    # UWB turn
    # =============================================================

    def calculate_uwb_turn(
        self,
        now
    ):

        turn = (
            self.UWB_SIGN
            *
            self.K_UWB
            *
            self.filtered_direction_error
        )

        turn = self.clamp(
            turn,
            -self.MAX_UWB_TURN,
            self.MAX_UWB_TURN
        )

        scale = self.calculate_uwb_scale(
            now
        )

        return (
            turn
            *
            scale
        )

    # =============================================================
    # Emergency
    # =============================================================

    def start_emergency(
        self,
        now
    ):

        self.emergency_armed = False

        self.emergency_clear_start = None

        self.emergency_state = 'STOP'

        self.emergency_start_time = now

        self.stopped_for_user = False

        self.publish_motor(
            0,
            0
        )

        self.get_logger().warn(
            'EMERGENCY | STOP'
        )

    def run_emergency_stop(
        self,
        now
    ):

        if (
            now
            -
            self.emergency_start_time
            <
            self.EMERGENCY_STOP_TIME
        ):

            self.publish_motor(
                0,
                0
            )

            return

        if (
            self.rear_clearance
            >=
            self.REAR_SAFE_DISTANCE
        ):

            self.emergency_state = 'REVERSE'

            self.reverse_start_time = now

            self.publish_motor(
                self.REVERSE_SPEED,
                self.REVERSE_SPEED
            )

            self.get_logger().warn(
                f'REVERSE | rear={self.rear_clearance:.2f}'
            )

        else:

            self.emergency_state = 'WAIT_REAR'

            self.publish_motor(
                0,
                0
            )

    def run_wait_rear(
        self,
        now
    ):

        self.publish_motor(
            0,
            0
        )

        if (
            self.rear_clearance
            >=
            self.REAR_SAFE_DISTANCE
        ):

            self.emergency_state = 'REVERSE'

            self.reverse_start_time = now

    def run_reverse(
        self,
        now
    ):

        if (
            self.rear_clearance
            <=
            self.REAR_STOP_DISTANCE
        ):

            self.publish_motor(
                0,
                0
            )

            self.emergency_state = 'WAIT_CLEAR'

            return

        if (
            now
            -
            self.reverse_start_time
            <
            self.REVERSE_TIME
        ):

            self.publish_motor(
                self.REVERSE_SPEED,
                self.REVERSE_SPEED
            )

            return

        self.publish_motor(
            0,
            0
        )

        self.emergency_state = 'WAIT_CLEAR'

        self.uwb_error_buffer.clear()

        self.ema_direction_error = 0.0

        self.filtered_direction_error = 0.0

    def run_wait_clear(
        self,
        now
    ):

        self.publish_motor(
            0,
            0
        )

        if self.emergency:

            self.emergency_clear_start = None

            return

        if self.emergency_clear_start is None:

            self.emergency_clear_start = now

            return

        if (
            now
            -
            self.emergency_clear_start
            <
            self.EMERGENCY_CLEAR_TIME
        ):

            return

        self.emergency_armed = True

        self.emergency_state = 'TRACK'

        self.emergency_clear_start = None

    # =============================================================
    # Main control
    # =============================================================

    def control_loop(
        self
    ):

        now = time.monotonic()

        # =========================================================
        # Emergency state machine
        # =========================================================

        if self.emergency_state == 'STOP':

            self.run_emergency_stop(now)

            return

        if self.emergency_state == 'WAIT_REAR':

            self.run_wait_rear(now)

            return

        if self.emergency_state == 'REVERSE':

            self.run_reverse(now)

            return

        if self.emergency_state == 'WAIT_CLEAR':

            self.run_wait_clear(now)

            return

        # =========================================================
        # Sensor timeout
        # =========================================================

        if (
            now
            -
            self.last_lidar
            >
            self.LIDAR_TIMEOUT
        ):

            self.publish_motor(
                0,
                0
            )

            return

        if (
            self.emergency
            and
            self.emergency_armed
        ):

            self.start_emergency(
                now
            )

            return

        if (
            now
            -
            self.last_uwb
            >
            self.UWB_TIMEOUT
        ):

            self.publish_motor(
                0,
                0
            )

            return

        if self.direction == 'LOST':

            self.publish_motor(
                0,
                0
            )

            return

        if (
            self.user_distance
            >
            self.MAX_TRACK_DISTANCE
        ):

            self.publish_motor(
                0,
                0
            )

            return

        # =========================================================
        # 사용자 거리
        # =========================================================

        if self.stopped_for_user:

            if (
                self.user_distance
                <
                self.RESTART_DISTANCE
            ):

                self.publish_motor(
                    0,
                    0
                )

                return

            self.stopped_for_user = False

        else:

            if (
                self.user_distance
                <=
                self.STOP_DISTANCE
            ):

                self.stopped_for_user = True

                self.publish_motor(
                    0,
                    0
                )

                return

        # =========================================================
        # Speed
        # =========================================================

        base_speed = self.calculate_speed()

        # =========================================================
        # UWB
        # =========================================================

        uwb_turn = self.calculate_uwb_turn(
            now
        )

        # =========================================================
        # LiDAR
        # =========================================================

        lidar_turn = (
            self.AVOID_GAIN
            *
            self.lidar_steering
        )

        lidar_turn = self.clamp(
            lidar_turn,
            -self.MAX_LIDAR_TURN,
            self.MAX_LIDAR_TURN
        )

        # =========================================================
        # Total
        # =========================================================

        total_turn = (
            uwb_turn
            +
            lidar_turn
        )

        total_turn = self.clamp(
            total_turn,
            -self.MAX_TOTAL_TURN,
            self.MAX_TOTAL_TURN
        )

        # =========================================================
        # Threat speed reduction
        # =========================================================

        threat = self.clamp(
            self.lidar_threat,
            0.0,
            1.0
        )

        speed_scale = (
            1.0
            -
            0.45
            *
            threat
        )

        speed_scale = self.clamp(
            speed_scale,
            0.55,
            1.0
        )

        base_speed *= speed_scale

        # =========================================================
        # PASSING 중 너무 빨리 지나가지 않게
        # =========================================================

        if self.avoidance_state == 'PASSING':

            base_speed *= 0.82

        # =========================================================
        # Motor differential
        # =========================================================

        left = (
            base_speed
            +
            total_turn
        )

        right = (
            base_speed
            -
            total_turn
        )

        left = max(
            -20.0,
            left
        )

        right = max(
            -20.0,
            right
        )

        self.publish_motor(
            left,
            right
        )


def main(args=None):

    rclpy.init(args=args)

    node = FollowController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        try:

            if rclpy.ok():

                node.publish_motor(
                    0,
                    0
                )

        except Exception:

            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()