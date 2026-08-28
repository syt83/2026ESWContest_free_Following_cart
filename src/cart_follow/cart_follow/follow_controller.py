import time
import statistics
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from std_msgs.msg import (
    Float32,
    String,
    Bool,
    Int32MultiArray
)


class FollowController(Node):

    def __init__(self):

        super().__init__(
            'follow_controller'
        )

        # =========================================================
        # 사용자 거리
        # 단위: cm
        # =========================================================

        # 130cm 이하 -> 정지
        self.STOP_DISTANCE = 130.0

        # 145cm 이상 -> 다시 추종
        self.RESTART_DISTANCE = 145.0

        self.SLOW_DISTANCE = 170.0
        self.NORMAL_DISTANCE = 250.0

        self.MAX_TRACK_DISTANCE = 600.0

        # =========================================================
        # Forward speed
        # =========================================================

        self.SLOW_SPEED = 38.0
        self.NORMAL_SPEED = 65.0
        self.MAX_SPEED = 85.0

        # =========================================================
        # UWB steering
        # =========================================================

        self.K_UWB = 0.35

        self.MAX_UWB_TURN = 22.0

        # + = RIGHT
        # - = LEFT
        self.UWB_SIGN = 1.0

        # =========================================================
        # UWB filter
        # =========================================================

        self.UWB_MEDIAN_SIZE = 5

        self.uwb_error_buffer = deque(
            maxlen=self.UWB_MEDIAN_SIZE
        )

        self.UWB_ALPHA = 0.40

        self.UWB_DEADBAND = 5.0

        self.UWB_MAX_ERROR_STEP = 8.0

        self.ema_direction_error = 0.0

        self.filtered_direction_error = 0.0

        # =========================================================
        # Startup stabilization
        #
        # 처음 센서값이 안정되기 전에 출발해서
        # 좌우로 흔들리는 현상을 방지
        # =========================================================

        # 최소 대기시간
        self.STARTUP_STABLE_TIME = 1.00

        # 같은 방향 몇 번 연속 확인할지
        self.STARTUP_DIRECTION_COUNT = 3

        self.startup_begin_time = (
            time.monotonic()
        )

        self.startup_direction = None

        self.startup_direction_count = 0

        self.startup_ready = False

        self.startup_ready_logged = False

        # =========================================================
        # IMU
        #
        # MPU6050 gyro Z를 이용한 yaw damping
        #
        # 기존 실측 convention:
        #
        # LEFT rotation  -> gyro_z positive
        # RIGHT rotation -> gyro_z negative
        #
        # 모터 steering convention:
        #
        # +turn -> RIGHT
        # -turn -> LEFT
        #
        # 따라서 gyro_z에 + gain을 곱하면
        # 현재 회전의 반대방향으로 damping이 걸림.
        # =========================================================

        self.K_IMU = 18.0

        # IMU가 너무 강하게 개입하지 않도록 제한
        self.MAX_IMU_TURN = 12.0

        # 작은 gyro noise 무시
        # imu_node에서도 deadband가 있지만 한번 더 보호
        self.IMU_YAW_DEADBAND = 0.015

        # IMU 데이터 timeout
        self.IMU_TIMEOUT = 0.30

        self.yaw_rate = 0.0

        self.last_imu = 0.0

        # =========================================================
        # LiDAR
        # =========================================================

        self.AVOID_GAIN = 1.0

        self.MAX_LIDAR_TURN = 45.0

        self.MAX_TOTAL_TURN = 55.0

        # =========================================================
        # 회피 상태별 UWB 영향
        # =========================================================

        self.UWB_SCALE_TRACK = 1.00

        self.UWB_SCALE_AVOID = 0.20

        self.UWB_SCALE_PASSING = 0.15

        self.UWB_SCALE_RECOVER_START = 0.15

        self.UWB_RECOVER_TIME = 1.00

        # =========================================================
        # Emergency
        # =========================================================

        self.EMERGENCY_STOP_TIME = 1.00

        self.REVERSE_SPEED = -45.0

        self.REVERSE_TIME = 2.00

        # =========================================================
        # Emergency 후 UWB 우선 복귀
        # =========================================================

        self.POST_REVERSE_UWB_TIME = 1.50

        self.POST_REVERSE_MAX_SPEED = 35.0

        self.post_reverse_uwb_until = 0.0

        self.post_reverse_recovery_active = False

        # =========================================================
        # Rear safety
        # =========================================================

        self.REAR_SAFE_DISTANCE = 0.80

        self.REAR_STOP_DISTANCE = 0.30

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

        # =========================================================
        # Avoidance state
        # =========================================================

        self.avoidance_state = 'TRACK'

        self.previous_avoidance_state = (
            'TRACK'
        )

        self.recover_start_time = 0.0

        self.avoidance_disabled = False

        # =========================================================
        # User stop
        # =========================================================

        self.stopped_for_user = False

        # =========================================================
        # Emergency state
        #
        # TRACK
        # STOP
        # WAIT_REAR
        # REVERSE
        # WAIT_CLEAR
        # =========================================================

        self.emergency_state = 'TRACK'

        self.emergency_start_time = 0.0

        self.reverse_start_time = 0.0

        self.emergency_armed = True

        self.emergency_clear_start = None

        # =========================================================
        # UWB subscriptions
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

        # =========================================================
        # IMU subscription
        # =========================================================

        self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.cb_imu,
            10
        )

        # =========================================================
        # LiDAR subscriptions
        # =========================================================

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
        # Publishers
        # =========================================================

        self.motor_pub = self.create_publisher(
            Int32MultiArray,
            '/motor_cmd_pwm',
            10
        )

        self.filtered_error_pub = (
            self.create_publisher(
                Float32,
                '/uwb/filtered_direction_error',
                10
            )
        )

        # IMU 보정 확인용
        self.imu_turn_pub = (
            self.create_publisher(
                Float32,
                '/control/imu_turn',
                10
            )
        )

        self.total_turn_pub = (
            self.create_publisher(
                Float32,
                '/control/total_turn',
                10
            )
        )

        # LiDAR 일반회피 ON/OFF
        self.avoidance_disable_pub = (
            self.create_publisher(
                Bool,
                '/avoidance/disable',
                10
            )
        )

        # =========================================================
        # 20Hz controller
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Follow controller started | '
            'STARTUP WAIT ON | '
            'IMU yaw damping ON | '
            'STOP=130cm | '
            'POST REVERSE UWB=1.5s'
        )

    # =============================================================
    # Clamp
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
    # LiDAR normal avoidance ON / OFF
    # =============================================================

    def set_avoidance_disabled(
        self,
        disabled
    ):

        disabled = bool(
            disabled
        )

        if (
            disabled
            ==
            self.avoidance_disabled
        ):
            return

        self.avoidance_disabled = disabled

        msg = Bool()

        msg.data = disabled

        self.avoidance_disable_pub.publish(
            msg
        )

        if disabled:

            self.get_logger().warn(
                'NORMAL LIDAR AVOIDANCE -> DISABLED'
            )

        else:

            self.get_logger().info(
                'NORMAL LIDAR AVOIDANCE -> ENABLED'
            )

    # =============================================================
    # Steering reset
    # =============================================================

    def clear_old_steering(
        self
    ):

        # UWB
        self.uwb_error_buffer.clear()

        self.ema_direction_error = 0.0

        self.filtered_direction_error = 0.0

        # LiDAR
        self.lidar_steering = 0.0

        self.lidar_threat = 0.0

        self.avoidance_state = 'TRACK'

        self.previous_avoidance_state = (
            'TRACK'
        )

        self.recover_start_time = 0.0

        # IMU는 실제 센서값이기 때문에
        # 값을 장시간 보존할 필요 없음
        self.yaw_rate = 0.0

    # =============================================================
    # UWB direction error
    # =============================================================

    def cb_direction_error(
        self,
        msg
    ):

        raw = float(
            msg.data
        )

        self.raw_direction_error = raw

        # ---------------------------------------------------------
        # Median
        # ---------------------------------------------------------

        self.uwb_error_buffer.append(
            raw
        )

        median_error = statistics.median(
            self.uwb_error_buffer
        )

        # ---------------------------------------------------------
        # EMA
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Deadband
        # ---------------------------------------------------------

        if (
            abs(
                self.ema_direction_error
            )
            <
            self.UWB_DEADBAND
        ):

            target_error = 0.0

        else:

            target_error = (
                self.ema_direction_error
            )

        # ---------------------------------------------------------
        # Rate limit
        # ---------------------------------------------------------

        delta = (
            target_error
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

        self.last_uwb = (
            time.monotonic()
        )

        # ---------------------------------------------------------
        # Debug
        # ---------------------------------------------------------

        out = Float32()

        out.data = float(
            self.filtered_direction_error
        )

        self.filtered_error_pub.publish(
            out
        )

    # =============================================================
    # User distance
    # =============================================================

    def cb_user_distance(
        self,
        msg
    ):

        self.user_distance = float(
            msg.data
        )

        self.last_uwb = (
            time.monotonic()
        )

    # =============================================================
    # UWB direction
    # =============================================================

    def cb_direction(
        self,
        msg
    ):

        new_direction = str(
            msg.data
        )

        # =========================================================
        # Startup 안정화용 연속 direction 확인
        # =========================================================

        if new_direction in (
            'LEFT',
            'CENTER',
            'RIGHT'
        ):

            if (
                new_direction
                ==
                self.startup_direction
            ):

                self.startup_direction_count += 1

            else:

                self.startup_direction = (
                    new_direction
                )

                self.startup_direction_count = 1

        else:

            self.startup_direction = None

            self.startup_direction_count = 0

        # =========================================================
        # CENTER 들어오면 과거 UWB steering 즉시 제거
        # =========================================================

        if (
            new_direction == 'CENTER'
            and
            self.direction != 'CENTER'
        ):

            self.ema_direction_error = 0.0

            self.filtered_direction_error = 0.0

            self.uwb_error_buffer.clear()

        self.direction = new_direction

        self.last_uwb = (
            time.monotonic()
        )

    # =============================================================
    # IMU callback
    # =============================================================

    def cb_imu(
        self,
        msg
    ):

        self.yaw_rate = float(
            msg.angular_velocity.z
        )

        self.last_imu = (
            time.monotonic()
        )

    # =============================================================
    # IMU damping
    # =============================================================

    def calculate_imu_turn(
        self,
        now
    ):

        # ---------------------------------------------------------
        # IMU stale
        # ---------------------------------------------------------

        if (
            now
            -
            self.last_imu
            >
            self.IMU_TIMEOUT
        ):

            return 0.0

        yaw = self.yaw_rate

        # ---------------------------------------------------------
        # Deadband
        # ---------------------------------------------------------

        if (
            abs(yaw)
            <
            self.IMU_YAW_DEADBAND
        ):

            return 0.0

        # =========================================================
        # 방향 convention
        #
        # gyro_z positive = LEFT 실제회전
        #
        # +turn = RIGHT 명령
        #
        # 따라서 positive gyro에 positive correction을 줘서
        # LEFT 회전을 억제한다.
        #
        # gyro_z negative = RIGHT 실제회전
        # -> negative correction = LEFT
        # =========================================================

        imu_turn = (
            self.K_IMU
            *
            yaw
        )

        return self.clamp(
            imu_turn,
            -self.MAX_IMU_TURN,
            self.MAX_IMU_TURN
        )

    # =============================================================
    # LiDAR callbacks
    # =============================================================

    def cb_lidar_steering(
        self,
        msg
    ):

        self.lidar_steering = float(
            msg.data
        )

        self.last_lidar = (
            time.monotonic()
        )

    def cb_lidar_threat(
        self,
        msg
    ):

        self.lidar_threat = float(
            msg.data
        )

        self.last_lidar = (
            time.monotonic()
        )

    def cb_emergency(
        self,
        msg
    ):

        self.emergency = bool(
            msg.data
        )

        self.last_lidar = (
            time.monotonic()
        )

    def cb_front_clearance(
        self,
        msg
    ):

        self.front_clearance = float(
            msg.data
        )

        self.last_lidar = (
            time.monotonic()
        )

    def cb_rear_clearance(
        self,
        msg
    ):

        self.rear_clearance = float(
            msg.data
        )

        self.last_lidar = (
            time.monotonic()
        )

    def cb_avoidance_state(
        self,
        msg
    ):

        new_state = str(
            msg.data
        )

        if (
            new_state
            !=
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

        self.last_lidar = (
            time.monotonic()
        )

    # =============================================================
    # Motor publisher
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

        # ---------------------------------------------------------
        # Stop
        # ---------------------------------------------------------

        if d <= self.STOP_DISTANCE:

            return 0.0

        # ---------------------------------------------------------
        # Slow zone
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Normal zone
        # ---------------------------------------------------------

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

        # 후진 직후에는 UWB 100%
        if self.post_reverse_recovery_active:

            return 1.0

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
    # UWB steering
    # =============================================================

    def calculate_uwb_turn(
        self,
        now
    ):

        if self.direction == 'LOST':

            return 0.0

        if self.direction == 'CENTER':

            return 0.0

        error = (
            self.filtered_direction_error
        )

        # ---------------------------------------------------------
        # 반대 방향 filter 잔류값 방지
        # ---------------------------------------------------------

        if (
            self.direction == 'RIGHT'
            and
            error <= 0.0
        ):

            return 0.0

        if (
            self.direction == 'LEFT'
            and
            error >= 0.0
        ):

            return 0.0

        turn = (
            self.UWB_SIGN
            *
            self.K_UWB
            *
            error
        )

        turn = self.clamp(
            turn,
            -self.MAX_UWB_TURN,
            self.MAX_UWB_TURN
        )

        scale = (
            self.calculate_uwb_scale(
                now
            )
        )

        return (
            turn
            *
            scale
        )

    # =============================================================
    # Startup check
    # =============================================================

    def startup_check(
        self,
        now
    ):

        if self.startup_ready:

            return True

        elapsed = (
            now
            -
            self.startup_begin_time
        )

        direction_ok = (
            self.startup_direction_count
            >=
            self.STARTUP_DIRECTION_COUNT
        )

        uwb_ok = (
            self.direction
            !=
            'LOST'
            and
            now
            -
            self.last_uwb
            <=
            self.UWB_TIMEOUT
        )

        imu_ok = (
            now
            -
            self.last_imu
            <=
            self.IMU_TIMEOUT
        )

        time_ok = (
            elapsed
            >=
            self.STARTUP_STABLE_TIME
        )

        if (
            direction_ok
            and
            uwb_ok
            and
            imu_ok
            and
            time_ok
        ):

            self.startup_ready = True

            # 시작 중 들어온 과거 steering 삭제
            self.uwb_error_buffer.clear()

            self.ema_direction_error = 0.0

            self.filtered_direction_error = 0.0

            if not self.startup_ready_logged:

                self.get_logger().info(
                    'STARTUP STABLE | '
                    f'DIR={self.direction} | '
                    'FOLLOW ENABLED'
                )

                self.startup_ready_logged = True

            return True

        return False

    # =============================================================
    # Emergency start
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

        # 이전 post-reverse 제거
        self.post_reverse_recovery_active = (
            False
        )

        self.post_reverse_uwb_until = 0.0

        # 일반 LiDAR avoidance OFF
        self.set_avoidance_disabled(
            True
        )

        self.clear_old_steering()

        self.publish_motor(
            0,
            0
        )

        self.get_logger().warn(
            'EMERGENCY | STOP'
        )

    # =============================================================
    # Emergency STOP
    # =============================================================

    def run_emergency_stop(
        self,
        now
    ):

        elapsed = (
            now
            -
            self.emergency_start_time
        )

        if (
            elapsed
            <
            self.EMERGENCY_STOP_TIME
        ):

            self.publish_motor(
                0,
                0
            )

            return

        # ---------------------------------------------------------
        # 뒤가 안전함
        # ---------------------------------------------------------

        if (
            self.rear_clearance
            >=
            self.REAR_SAFE_DISTANCE
        ):

            self.emergency_state = (
                'REVERSE'
            )

            self.reverse_start_time = now

            self.publish_motor(
                self.REVERSE_SPEED,
                self.REVERSE_SPEED
            )

            self.get_logger().warn(
                'EMERGENCY | REVERSE'
            )

        # ---------------------------------------------------------
        # 뒤쪽 막힘
        # ---------------------------------------------------------

        else:

            self.emergency_state = (
                'WAIT_REAR'
            )

            self.publish_motor(
                0,
                0
            )

            self.get_logger().warn(
                'EMERGENCY | WAIT_REAR'
            )

    # =============================================================
    # Wait rear
    # =============================================================

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

            self.emergency_state = (
                'REVERSE'
            )

            self.reverse_start_time = now

            self.get_logger().warn(
                'EMERGENCY | REVERSE'
            )

    # =============================================================
    # Finish reverse
    # =============================================================

    def finish_reverse(
        self
    ):

        self.publish_motor(
            0,
            0
        )

        self.emergency_state = (
            'WAIT_CLEAR'
        )

        self.emergency_clear_start = None

        self.clear_old_steering()

        self.get_logger().warn(
            'REVERSE FINISHED | '
            'waiting emergency clear'
        )

    # =============================================================
    # Reverse
    # =============================================================

    def run_reverse(
        self,
        now
    ):

        # ---------------------------------------------------------
        # 후방 장애물이 너무 가까워짐
        # ---------------------------------------------------------

        if (
            self.rear_clearance
            <=
            self.REAR_STOP_DISTANCE
        ):

            self.finish_reverse()

            self.get_logger().warn(
                'REVERSE STOPPED | '
                'rear obstacle'
            )

            return

        elapsed = (
            now
            -
            self.reverse_start_time
        )

        if (
            elapsed
            <
            self.REVERSE_TIME
        ):

            # 직선 후진
            # IMU steering을 적용하지 않는다.
            self.publish_motor(
                self.REVERSE_SPEED,
                self.REVERSE_SPEED
            )

            return

        self.finish_reverse()

    # =============================================================
    # Wait Emergency clear
    # =============================================================

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

        if (
            self.emergency_clear_start
            is None
        ):

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

        # =========================================================
        # Emergency 완전 종료
        # =========================================================

        self.clear_old_steering()

        self.emergency_armed = True

        self.emergency_state = 'TRACK'

        self.emergency_clear_start = None

        # ---------------------------------------------------------
        # 후진 후 UWB 우선 시작
        # ---------------------------------------------------------

        self.post_reverse_recovery_active = (
            True
        )

        self.post_reverse_uwb_until = (
            now
            +
            self.POST_REVERSE_UWB_TIME
        )

        # LiDAR 일반회피는 아직 OFF
        self.set_avoidance_disabled(
            True
        )

        self.get_logger().warn(
            'EMERGENCY CLEARED | '
            'POST-REVERSE UWB PRIORITY START'
        )

    # =============================================================
    # Post reverse recovery
    # =============================================================

    def update_post_reverse_recovery(
        self,
        now
    ):

        if not self.post_reverse_recovery_active:

            return False

        if (
            now
            <
            self.post_reverse_uwb_until
        ):

            return True

        # ---------------------------------------------------------
        # Recovery 완료
        # ---------------------------------------------------------

        self.post_reverse_recovery_active = (
            False
        )

        self.post_reverse_uwb_until = 0.0

        self.lidar_steering = 0.0

        self.lidar_threat = 0.0

        self.set_avoidance_disabled(
            False
        )

        self.get_logger().info(
            'POST-REVERSE UWB PRIORITY END | '
            'LiDAR avoidance enabled'
        )

        return False

    # =============================================================
    # Main control
    # =============================================================

    def control_loop(
        self
    ):

        now = time.monotonic()

        # =========================================================
        # 1. Emergency state machine
        # =========================================================

        if self.emergency_state == 'STOP':

            self.run_emergency_stop(
                now
            )

            return

        if self.emergency_state == 'WAIT_REAR':

            self.run_wait_rear(
                now
            )

            return

        if self.emergency_state == 'REVERSE':

            self.run_reverse(
                now
            )

            return

        if self.emergency_state == 'WAIT_CLEAR':

            self.run_wait_clear(
                now
            )

            return

        # =========================================================
        # 2. LiDAR timeout
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

        # =========================================================
        # 3. Emergency
        # =========================================================

        if (
            self.emergency
            and
            self.emergency_armed
        ):

            self.start_emergency(
                now
            )

            return

        # =========================================================
        # 4. UWB timeout
        # =========================================================

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

        # =========================================================
        # 5. IMU timeout
        #
        # IMU를 실제 제어에 쓰므로
        # IMU가 죽으면 정상주행도 정지
        # =========================================================

        if (
            now
            -
            self.last_imu
            >
            self.IMU_TIMEOUT
        ):

            self.publish_motor(
                0,
                0
            )

            return

        # =========================================================
        # 6. Startup stabilization
        # =========================================================

        if not self.startup_check(
            now
        ):

            self.publish_motor(
                0,
                0
            )

            return

        # =========================================================
        # 7. LOST
        # =========================================================

        if self.direction == 'LOST':

            self.publish_motor(
                0,
                0
            )

            return

        # =========================================================
        # 8. 너무 멀면 정지
        # =========================================================

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
        # 9. Post reverse recovery 상태
        # =========================================================

        post_reverse_active = (
            self.update_post_reverse_recovery(
                now
            )
        )

        # =========================================================
        # 10. 사용자 거리 STOP latch
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
        # 11. Base speed
        # =========================================================

        base_speed = (
            self.calculate_speed()
        )

        # 후진 직후에는 저속
        if post_reverse_active:

            base_speed = min(
                base_speed,
                self.POST_REVERSE_MAX_SPEED
            )

        # =========================================================
        # 12. UWB steering
        # =========================================================

        uwb_turn = (
            self.calculate_uwb_turn(
                now
            )
        )

        # =========================================================
        # 13. LiDAR steering
        # =========================================================

        if post_reverse_active:

            # 후진 직후에는 일반 LiDAR steering 무시
            lidar_turn = 0.0

        else:

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
        # 14. IMU yaw damping
        # =========================================================

        imu_turn = (
            self.calculate_imu_turn(
                now
            )
        )

        # =========================================================
        # 15. Final steering
        #
        # UWB:
        # 사람 방향
        #
        # LiDAR:
        # 장애물 회피
        #
        # IMU:
        # 현재 회전 과속 억제
        # =========================================================

        total_turn = (
            uwb_turn
            +
            lidar_turn
            +
            imu_turn
        )

        total_turn = self.clamp(
            total_turn,
            -self.MAX_TOTAL_TURN,
            self.MAX_TOTAL_TURN
        )

        # ---------------------------------------------------------
        # Debug topic
        # ---------------------------------------------------------

        msg = Float32()

        msg.data = float(
            imu_turn
        )

        self.imu_turn_pub.publish(
            msg
        )

        msg = Float32()

        msg.data = float(
            total_turn
        )

        self.total_turn_pub.publish(
            msg
        )

        # =========================================================
        # 16. Threat based speed
        # =========================================================

        if post_reverse_active:

            threat = 0.0

        else:

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

        base_speed *= (
            speed_scale
        )

        # =========================================================
        # 17. PASSING slowdown
        # =========================================================

        if (
            not post_reverse_active
            and
            self.avoidance_state
            ==
            'PASSING'
        ):

            base_speed *= 0.82

        # =========================================================
        # 18. Differential drive
        #
        # positive turn = RIGHT
        #
        # RIGHT:
        # left motor faster
        # right motor slower
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

        # 정상주행 과도한 역회전 제한
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

    rclpy.init(
        args=args
    )

    node = FollowController()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        try:

            if rclpy.ok():

                node.publish_motor(
                    0,
                    0
                )

                node.set_avoidance_disabled(
                    False
                )

        except Exception:

            pass

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()