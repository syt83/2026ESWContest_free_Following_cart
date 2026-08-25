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

        super().__init__(
            'follow_controller'
        )

        # =========================================================
        # 사용자 거리
        # 단위: cm
        # =========================================================

        # 130cm 이하 -> 정지
        self.STOP_DISTANCE = 130.0

        # 145cm 이상 -> 다시 추종 시작
        self.RESTART_DISTANCE = 145.0

        # 감속 영역
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

        # + error -> RIGHT
        # - error -> LEFT
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

        # Emergency 후 1초 정지
        self.EMERGENCY_STOP_TIME = 1.00

        # 직선 후진 PWM
        self.REVERSE_SPEED = -45.0

        # 후진 시간
        self.REVERSE_TIME = 2.00

        # =========================================================
        # NEW
        # Emergency 후 UWB 우선 복귀
        # =========================================================

        # 후진이 끝나고 Emergency가 해제된 뒤
        # 이 시간 동안 LiDAR 일반 steering을 사용하지 않는다.
        self.POST_REVERSE_UWB_TIME = 1.50

        # 복귀 중 최고 속도
        self.POST_REVERSE_MAX_SPEED = 35.0

        # 이 시각까지 UWB 우선
        self.post_reverse_uwb_until = 0.0

        # 현재 post-reverse recovery 중인지
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

        self.previous_avoidance_state = 'TRACK'

        self.recover_start_time = 0.0

        # 현재 LiDAR 일반회피 disable 명령 상태
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

        # LiDAR 일반 회피 ON/OFF
        self.avoidance_disable_pub = (
            self.create_publisher(
                Bool,
                '/avoidance/disable',
                10
            )
        )

        # =========================================================
        # Timer
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Follow controller started | '
            'STOP=130cm | '
            'POST REVERSE UWB PRIORITY=1.5s'
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

        # 같은 명령을 매 루프 계속 publish하지 않음
        if (
            disabled
            ==
            self.avoidance_disabled
        ):
            return

        self.avoidance_disabled = (
            disabled
        )

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
    # 과거 steering 완전 초기화
    # =============================================================

    def clear_old_steering(
        self
    ):

        # UWB filter
        self.uwb_error_buffer.clear()

        self.ema_direction_error = 0.0

        self.filtered_direction_error = 0.0

        # LiDAR cached command
        self.lidar_steering = 0.0

        self.lidar_threat = 0.0

        # 회피 상태
        self.avoidance_state = 'TRACK'

        self.previous_avoidance_state = (
            'TRACK'
        )

        self.recover_start_time = 0.0

    # =============================================================
    # UWB error
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

        self.last_uwb = time.monotonic()

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
    # UWB distance
    # =============================================================

    def cb_user_distance(
        self,
        msg
    ):

        self.user_distance = float(
            msg.data
        )

        self.last_uwb = time.monotonic()

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

        # CENTER 진입 시 이전 조향값 제거
        if (
            new_direction == 'CENTER'
            and
            self.direction != 'CENTER'
        ):

            self.ema_direction_error = 0.0

            self.filtered_direction_error = 0.0

            self.uwb_error_buffer.clear()

        self.direction = (
            new_direction
        )

        self.last_uwb = time.monotonic()

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

        self.last_lidar = time.monotonic()

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
    # Forward speed
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
        # Slow
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
        # Normal
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

        # Post-reverse에서는 LiDAR state 영향 없이
        # UWB 100% 사용
        if self.post_reverse_recovery_active:

            return 1.0

        if self.avoidance_state == 'AVOID':

            return (
                self.UWB_SCALE_AVOID
            )

        if self.avoidance_state == 'PASSING':

            return (
                self.UWB_SCALE_PASSING
            )

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

        # LOST
        if self.direction == 'LOST':

            return 0.0

        # CENTER
        if self.direction == 'CENTER':

            return 0.0

        error = (
            self.filtered_direction_error
        )

        # ---------------------------------------------------------
        # RIGHT인데 과거 음수 error가 남아있다면
        # 반대조향 금지
        # ---------------------------------------------------------

        if (
            self.direction == 'RIGHT'
            and
            error <= 0.0
        ):

            return 0.0

        # ---------------------------------------------------------
        # LEFT인데 과거 양수 error가 남아있다면
        # 반대조향 금지
        # ---------------------------------------------------------

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

        scale = self.calculate_uwb_scale(
            now
        )

        return (
            turn
            *
            scale
        )

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

        # 이전 post-reverse 상태 제거
        self.post_reverse_recovery_active = (
            False
        )

        self.post_reverse_uwb_until = 0.0

        # =========================================================
        # Emergency 중 일반 LiDAR 회피 중지
        # Emergency 감지는 lidar_avoidance 안에서 계속 동작
        # =========================================================

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
    # Emergency stop
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
        # 뒤 공간 충분
        # ---------------------------------------------------------

        if (
            self.rear_clearance
            >=
            self.REAR_SAFE_DISTANCE
        ):

            self.emergency_state = (
                'REVERSE'
            )

            self.reverse_start_time = (
                now
            )

            self.publish_motor(
                self.REVERSE_SPEED,
                self.REVERSE_SPEED
            )

            self.get_logger().warn(
                'EMERGENCY | REVERSE'
            )

        # ---------------------------------------------------------
        # 뒤 공간 부족
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

            self.reverse_start_time = (
                now
            )

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

        # =========================================================
        # 후진 전 모든 방향 기억 제거
        # =========================================================

        self.clear_old_steering()

        # 아직 일반 LiDAR 회피는 OFF
        # Emergency False 확인 후 UWB recovery 시작

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
        # 후방 장애물 너무 가까움
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

        # ---------------------------------------------------------
        # 후진 중
        # ---------------------------------------------------------

        if (
            elapsed
            <
            self.REVERSE_TIME
        ):

            self.publish_motor(
                self.REVERSE_SPEED,
                self.REVERSE_SPEED
            )

            return

        # ---------------------------------------------------------
        # 후진 완료
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # 아직 Emergency
        # ---------------------------------------------------------

        if self.emergency:

            self.emergency_clear_start = None

            return

        # ---------------------------------------------------------
        # Emergency False 시작
        # ---------------------------------------------------------

        if (
            self.emergency_clear_start
            is None
        ):

            self.emergency_clear_start = (
                now
            )

            return

        # ---------------------------------------------------------
        # 일정시간 False 확인
        # ---------------------------------------------------------

        if (
            now
            -
            self.emergency_clear_start
            <
            self.EMERGENCY_CLEAR_TIME
        ):

            return

        # =========================================================
        # Emergency 완전 해제
        #
        # 여기서 일반 LiDAR 회피를 바로 켜지 않는다.
        #
        # UWB에게 먼저 1.5초 우선권을 줌.
        # =========================================================

        self.clear_old_steering()

        self.emergency_armed = True

        self.emergency_state = (
            'TRACK'
        )

        self.emergency_clear_start = None

        # UWB recovery 시작
        self.post_reverse_recovery_active = (
            True
        )

        self.post_reverse_uwb_until = (
            now
            +
            self.POST_REVERSE_UWB_TIME
        )

        # LiDAR 일반 회피는 계속 OFF
        self.set_avoidance_disabled(
            True
        )

        self.get_logger().warn(
            'EMERGENCY CLEARED | '
            'POST-REVERSE UWB PRIORITY START | '
            f'{self.POST_REVERSE_UWB_TIME:.1f}s'
        )

    # =============================================================
    # Post reverse recovery update
    # =============================================================

    def update_post_reverse_recovery(
        self,
        now
    ):

        if not self.post_reverse_recovery_active:

            return False

        # 아직 recovery 시간
        if (
            now
            <
            self.post_reverse_uwb_until
        ):

            return True

        # =========================================================
        # Recovery 완료
        # =========================================================

        self.post_reverse_recovery_active = (
            False
        )

        self.post_reverse_uwb_until = 0.0

        # 과거 command 다시 한번 삭제
        self.lidar_steering = 0.0
        self.lidar_threat = 0.0

        # 일반 LiDAR 회피 재활성화
        self.set_avoidance_disabled(
            False
        )

        self.get_logger().info(
            'POST-REVERSE UWB PRIORITY END | '
            'normal LiDAR avoidance enabled'
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
        # Emergency state machine
        # 최우선
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
        # LiDAR timeout
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
        # Emergency
        #
        # Post-reverse recovery 중에도 Emergency는 살아있음.
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
        # Post reverse recovery 상태 확인
        # =========================================================

        post_reverse_active = (
            self.update_post_reverse_recovery(
                now
            )
        )

        # =========================================================
        # UWB timeout
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
        # LOST
        # =========================================================

        if self.direction == 'LOST':

            self.publish_motor(
                0,
                0
            )

            return

        # =========================================================
        # 너무 멀면 정지
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
        # 사용자 거리 stop latch
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

            self.stopped_for_user = (
                False
            )

        else:

            if (
                self.user_distance
                <=
                self.STOP_DISTANCE
            ):

                self.stopped_for_user = (
                    True
                )

                self.publish_motor(
                    0,
                    0
                )

                return

        # =========================================================
        # Base speed
        # =========================================================

        base_speed = (
            self.calculate_speed()
        )

        # =========================================================
        # NEW
        # 후진 직후 UWB 복귀 중에는 저속
        # =========================================================

        if post_reverse_active:

            base_speed = min(
                base_speed,
                self.POST_REVERSE_MAX_SPEED
            )

        # =========================================================
        # UWB steering
        # =========================================================

        uwb_turn = (
            self.calculate_uwb_turn(
                now
            )
        )

        # =========================================================
        # LiDAR steering
        # =========================================================

        if post_reverse_active:

            # =====================================================
            # 중요:
            # Emergency 후 복귀 중에는
            # 일반 LiDAR LEFT/RIGHT 조향 완전히 무시.
            #
            # Emergency 감지는 위에서 계속 사용 중.
            # =====================================================

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
        # Total steering
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
        # Threat speed scaling
        #
        # post-reverse에서는 LiDAR threat도 일반 회피용으로
        # 적용하지 않는다.
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
        # PASSING slowdown
        #
        # post-reverse에서는 PASSING 자체가 disable되어 있으므로
        # 적용하지 않음
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
        # Differential drive
        #
        # + total_turn = RIGHT
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

        # 정상주행에서 과도한 역회전 방지
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

                # 종료 시 LiDAR 일반 회피를 정상 상태로 되돌림
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