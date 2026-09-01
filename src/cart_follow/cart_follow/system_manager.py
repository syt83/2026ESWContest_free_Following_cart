import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool, Int32MultiArray, String

import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522


# =============================================================
# System configuration
# =============================================================

NFC_UID = '765575780619'


# =============================================================
# Status LED
#
# GPIO.BOARD physical pin numbers
#
# RED    : Pin 11 = GPIO17
# YELLOW : Pin 13 = GPIO27
# GREEN  : Pin 15 = GPIO22
# =============================================================

LED_RED_PIN = 11
LED_YELLOW_PIN = 13
LED_GREEN_PIN = 15


# =============================================================
# LED timing
# =============================================================

# 전원/프로그램 시작 시
# 세 LED 모두 켜는 시간
BOOT_LED_TEST_TIME = 0.50

# WAIT:
# 0.5초 ON + 0.5초 OFF
WAIT_BLINK_PERIOD = 1.00

# STARTING / STOPPING:
# 0.15초 ON + 0.15초 OFF
START_BLINK_PERIOD = 0.30

# Emergency STOP:
# 빠른 빨강 점멸
EMERGENCY_BLINK_PERIOD = 0.20

# LED 상태 업데이트 주기
LED_UPDATE_INTERVAL = 0.05


# =============================================================
# Sensor
# =============================================================

SENSOR_FRESH_TIME = 1.0
SENSOR_READY_TIMEOUT = 20.0


# =============================================================
# NFC
# =============================================================

CARD_REMOVE_TIME = 1.0
NFC_POLL_INTERVAL = 0.10
NFC_RELEASE_CONFIRM_TIME = 0.50
NFC_SHUTDOWN_HOLD_TIME = 3.0
SHUTDOWN_BLINK_PERIOD = 0.30


class SystemManager(Node):

    def __init__(self):

        super().__init__('system_manager')

        # =========================================================
        # Main state
        #
        # WAIT
        # STARTING
        # ACTIVE
        # STOPPING
        # =========================================================

        self.active = False
        self.busy = False
        self.shutting_down = False

        self.system_phase = 'WAIT'

        self.state_lock = threading.Lock()
        self.child_lock = threading.Lock()
        self.led_lock = threading.Lock()

        self.shutdown_event = threading.Event()


        # =========================================================
        # Child processes
        # =========================================================

        self.children = {}


        # =========================================================
        # Sensor readiness
        # =========================================================

        self.last_uwb = 0.0
        self.last_lidar = 0.0
        self.last_imu = 0.0

        self.uwb_direction = 'LOST'


        self.create_subscription(
            String,
            '/uwb/direction',
            self.cb_uwb_direction,
            10
        )


        self.create_subscription(
            LaserScan,
            '/scan',
            self.cb_scan,
            qos_profile_sensor_data
        )


        self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.cb_imu,
            10
        )


        # =========================================================
        # Motor safety publishers
        # =========================================================

        self.hard_stop_pub = self.create_publisher(
            Bool,
            '/motor_hard_stop',
            10
        )


        self.motor_pub = self.create_publisher(
            Int32MultiArray,
            '/motor_cmd_pwm',
            10
        )


        # =========================================================
        # Motor state
        #
        # LED 표시용
        # =========================================================

        self.last_motor_left = 0
        self.last_motor_right = 0


        self.create_subscription(
            Int32MultiArray,
            '/motor_cmd_pwm',
            self.cb_motor_cmd,
            10
        )


        # =========================================================
        # Emergency state
        #
        # Emergency + motor=0
        # -> RED fast blink
        #
        # Emergency + motor != 0
        # -> GREEN
        # =========================================================

        self.emergency_active = False


        self.create_subscription(
            Bool,
            '/emergency_stop',
            self.cb_emergency,
            10
        )


        # =========================================================
        # OLED removed
        # =========================================================

        self.get_logger().info(
            'OLED REMOVED | I2C reserved for MPU6050'
        )


        # =========================================================
        # GPIO / NFC
        # =========================================================

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)


        self.nfc = SimpleMFRC522()


        # =========================================================
        # LED GPIO
        # =========================================================

        GPIO.setup(
            LED_RED_PIN,
            GPIO.OUT,
            initial=GPIO.LOW
        )

        GPIO.setup(
            LED_YELLOW_PIN,
            GPIO.OUT,
            initial=GPIO.LOW
        )

        GPIO.setup(
            LED_GREEN_PIN,
            GPIO.OUT,
            initial=GPIO.LOW
        )


        # 마지막 실제 출력 상태
        self.last_led_output = None


        # =========================================================
        # Power-on LED self test
        #
        # 처음 0.5초:
        #
        # RED + YELLOW + GREEN 모두 ON
        # =========================================================

        self.boot_led_until = (
            time.monotonic()
            +
            BOOT_LED_TEST_TIME
        )


        self.write_leds(
            True,
            True,
            True
        )


        # =========================================================
        # LED timer
        # =========================================================

        self.led_timer = self.create_timer(
            LED_UPDATE_INTERVAL,
            self.update_status_led
        )


        # =========================================================
        # NFC
        # =========================================================

        self.card_latched = False
        self.last_card_seen = 0.0
        self.wait_card_start = None
        self.wait_card_last_seen = 0.0
        self.wait_card_long_fired = False


        self.nfc_timer = self.create_timer(
            NFC_POLL_INTERVAL,
            self.poll_nfc
        )


        # =========================================================
        # Initial safe state
        #
        # 전원이 들어와 system_manager가 실행됐다고 해서
        # 로봇이 자동 주행하지 않는다.
        #
        # 모터는 HARD STOP.
        #
        # 카드 입력 전까지 WAIT.
        # =========================================================

        self.publish_hard_stop(
            True
        )

        self.publish_zero()


        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'SYSTEM MANAGER STARTED'
        )

        self.get_logger().info(
            'STATE = WAIT'
        )

        self.get_logger().info(
            'YELLOW SLOW BLINK = WAITING FOR NFC'
        )

        self.get_logger().info(
            'SCAN NFC CARD'
        )

        self.get_logger().info(
            '========================================'
        )


    # =============================================================
    # Phase
    # =============================================================

    def set_phase(
        self,
        phase
    ):

        phase = str(
            phase
        ).upper()


        with self.state_lock:

            old_phase = self.system_phase

            self.system_phase = phase


        if old_phase != phase:

            try:

                self.get_logger().info(
                    f'SYSTEM PHASE -> {phase}'
                )

            except Exception:

                pass


    # =============================================================
    # LED low-level
    # =============================================================

    def write_leds(
        self,
        red,
        yellow,
        green
    ):

        values_bool = (
            bool(red),
            bool(yellow),
            bool(green)
        )


        with self.led_lock:

            if (
                self.last_led_output
                ==
                values_bool
            ):

                return


            values_gpio = (
                GPIO.HIGH
                if red
                else GPIO.LOW,

                GPIO.HIGH
                if yellow
                else GPIO.LOW,

                GPIO.HIGH
                if green
                else GPIO.LOW
            )


            try:

                GPIO.output(
                    (
                        LED_RED_PIN,
                        LED_YELLOW_PIN,
                        LED_GREEN_PIN
                    ),
                    values_gpio
                )


                self.last_led_output = (
                    values_bool
                )


            except Exception:

                pass


    # =============================================================
    # Blink helper
    # =============================================================

    def blink_on(
        self,
        now,
        period
    ):

        if period <= 0.0:

            return True


        phase = (
            now
            %
            period
        )


        return (
            phase
            <
            (
                period
                /
                2.0
            )
        )


    # =============================================================
    # Main LED state machine
    # =============================================================

    def update_status_led(
        self
    ):

        if self.shutting_down:

            return


        now = time.monotonic()


        # =========================================================
        # BOOT self test
        #
        # RED + YELLOW + GREEN
        # =========================================================

        if (
            now
            <
            self.boot_led_until
        ):

            self.write_leds(
                True,
                True,
                True
            )

            return


        with self.state_lock:

            phase = self.system_phase

            active = self.active


        # =========================================================
        # WAIT
        #
        # YELLOW slow blink
        # =========================================================

        if phase == 'WAIT':

            on = self.blink_on(
                now,
                WAIT_BLINK_PERIOD
            )


            self.write_leds(
                False,
                on,
                False
            )

            return


        # =========================================================
        # STARTING
        #
        # YELLOW fast blink
        # =========================================================

        if phase == 'STARTING':

            on = self.blink_on(
                now,
                START_BLINK_PERIOD
            )


            self.write_leds(
                False,
                on,
                False
            )

            return


        # =========================================================
        # STOPPING
        #
        # YELLOW fast blink
        #
        # 카드를 다시 댔다는 것을 즉시 알 수 있음
        # =========================================================

        if phase == 'STOPPING':

            on = self.blink_on(
                now,
                START_BLINK_PERIOD
            )


            self.write_leds(
                False,
                on,
                False
            )

            return


        if phase == 'SHUTDOWN':
            on = self.blink_on(
                now,
                SHUTDOWN_BLINK_PERIOD
            )

            self.write_leds(
                on,
                on,
                on
            )

            return

        # =========================================================
        # ACTIVE
        # =========================================================

        if (
            phase == 'ACTIVE'
            and
            active
        ):

            moving = not (
                self.last_motor_left == 0
                and
                self.last_motor_right == 0
            )


            # =====================================================
            # Emergency
            # =====================================================

            if self.emergency_active:

                # -----------------------------------------------
                # Emergency reverse / escape
                # -----------------------------------------------

                if moving:

                    self.write_leds(
                        False,
                        False,
                        True
                    )

                    return


                # -----------------------------------------------
                # Emergency STOP
                #
                # RED fast blink
                # -----------------------------------------------

                on = self.blink_on(
                    now,
                    EMERGENCY_BLINK_PERIOD
                )


                self.write_leds(
                    on,
                    False,
                    False
                )

                return


            # =====================================================
            # Normal ACTIVE
            # =====================================================

            if moving:

                # GREEN steady
                self.write_leds(
                    False,
                    False,
                    True
                )


            else:

                # RED steady
                self.write_leds(
                    True,
                    False,
                    False
                )


            return


        # =========================================================
        # Fallback
        #
        # 예상치 못한 상태라면 안전하게 YELLOW
        # =========================================================

        self.write_leds(
            False,
            True,
            False
        )


    # =============================================================
    # Sensor callbacks
    # =============================================================

    def cb_uwb_direction(
        self,
        msg
    ):

        direction = str(
            msg.data
        ).strip().upper()


        self.uwb_direction = (
            direction
        )


        if direction in (
            'LEFT',
            'CENTER',
            'RIGHT'
        ):

            self.last_uwb = (
                time.monotonic()
            )


    def cb_scan(
        self,
        msg
    ):

        self.last_lidar = (
            time.monotonic()
        )


    def cb_imu(
        self,
        msg
    ):

        self.last_imu = (
            time.monotonic()
        )


    # =============================================================
    # Motor callback
    # =============================================================

    def cb_motor_cmd(
        self,
        msg
    ):

        if len(msg.data) < 2:

            return


        self.last_motor_left = int(
            msg.data[0]
        )

        self.last_motor_right = int(
            msg.data[1]
        )


    # =============================================================
    # Emergency callback
    # =============================================================

    def cb_emergency(
        self,
        msg
    ):

        self.emergency_active = bool(
            msg.data
        )


    # =============================================================
    # Motor safety
    # =============================================================

    def publish_hard_stop(
        self,
        enabled
    ):

        msg = Bool()

        msg.data = bool(
            enabled
        )


        self.hard_stop_pub.publish(
            msg
        )


    def publish_zero(
        self
    ):

        msg = Int32MultiArray()

        msg.data = [
            0,
            0
        ]


        self.motor_pub.publish(
            msg
        )


    # =============================================================
    # Force stop
    # =============================================================

    def force_stop(
        self,
        repeats=5
    ):

        for _ in range(repeats):

            if (
                self.shutting_down
                and
                not rclpy.ok()
            ):

                break


            try:

                self.publish_hard_stop(
                    True
                )

                self.publish_zero()


            except Exception:

                pass


            time.sleep(
                0.05
            )


    # =============================================================
    # Release hard stop
    # =============================================================

    def release_hard_stop(
        self,
        repeats=5
    ):

        # 해제 직전에도 반드시 0 명령

        for _ in range(repeats):

            self.publish_zero()

            self.publish_hard_stop(
                False
            )

            time.sleep(
                0.05
            )


    # =============================================================
    # Child process start
    # =============================================================

    def start_child(
        self,
        name,
        command
    ):

        if self.shutdown_event.is_set():

            raise RuntimeError(
                'system manager is shutting down'
            )


        with self.child_lock:

            old = self.children.get(
                name
            )


            if (
                old is not None
                and
                old.poll() is None
            ):

                return


            self.get_logger().info(
                f'START -> {name}'
            )


            process = subprocess.Popen(
                command,
                start_new_session=True
            )


            self.children[
                name
            ] = process


    # =============================================================
    # Child alive
    # =============================================================

    def child_alive(
        self,
        name
    ):

        with self.child_lock:

            process = self.children.get(
                name
            )


        return (
            process is not None
            and
            process.poll() is None
        )


    def require_child_alive(
        self,
        name
    ):

        if not self.child_alive(
            name
        ):

            raise RuntimeError(
                f'{name} failed during startup'
            )


    # =============================================================
    # Stop child
    # =============================================================

    def stop_child(
        self,
        name
    ):

        with self.child_lock:

            process = self.children.get(
                name
            )


        if process is None:

            return


        if process.poll() is not None:

            with self.child_lock:

                self.children.pop(
                    name,
                    None
                )

            return


        try:

            self.get_logger().info(
                f'STOP -> {name}'
            )

        except Exception:

            pass


        # =========================================================
        # 1. SIGINT
        # =========================================================

        try:

            os.killpg(
                os.getpgid(
                    process.pid
                ),
                signal.SIGINT
            )

        except ProcessLookupError:

            pass


        try:

            process.wait(
                timeout=3.0
            )


        except subprocess.TimeoutExpired:

            # =====================================================
            # 2. SIGTERM
            # =====================================================

            try:

                os.killpg(
                    os.getpgid(
                        process.pid
                    ),
                    signal.SIGTERM
                )

            except ProcessLookupError:

                pass


            try:

                process.wait(
                    timeout=2.0
                )


            except subprocess.TimeoutExpired:

                # ================================================
                # 3. SIGKILL
                # ================================================

                try:

                    os.killpg(
                        os.getpgid(
                            process.pid
                        ),
                        signal.SIGKILL
                    )

                except ProcessLookupError:

                    pass


                try:

                    process.wait(
                        timeout=1.0
                    )

                except subprocess.TimeoutExpired:

                    pass


        with self.child_lock:

            self.children.pop(
                name,
                None
            )


    # =============================================================
    # Stop all
    # =============================================================

    def stop_all_children(
        self
    ):

        # Controller 먼저
        # motor_serial 마지막

        stop_order = (
            'follow_controller',
            'imu_node',
            'uwb_tracker',
            'lidar_avoidance',
            'ydlidar',
            'motor_serial'
        )


        for name in stop_order:

            self.stop_child(
                name
            )


        with self.child_lock:

            leftovers = list(
                self.children.keys()
            )


        for name in leftovers:

            self.stop_child(
                name
            )


    # =============================================================
    # Abortable sleep
    # =============================================================

    def sleep_checked(
        self,
        seconds,
        child_names=()
    ):

        end_time = (
            time.monotonic()
            +
            seconds
        )


        while (
            time.monotonic()
            <
            end_time
        ):

            if self.shutdown_event.is_set():

                raise RuntimeError(
                    'system manager shutdown requested'
                )


            for name in child_names:

                self.require_child_alive(
                    name
                )


            time.sleep(
                0.05
            )


    # =============================================================
    # Sensor readiness
    # =============================================================

    def sensor_status(
        self
    ):

        now = time.monotonic()


        uwb_ok = (
            self.uwb_direction
            in
            (
                'LEFT',
                'CENTER',
                'RIGHT'
            )
            and
            now
            -
            self.last_uwb
            <=
            SENSOR_FRESH_TIME
        )


        lidar_ok = (
            now
            -
            self.last_lidar
            <=
            SENSOR_FRESH_TIME
        )


        imu_ok = (
            now
            -
            self.last_imu
            <=
            SENSOR_FRESH_TIME
        )


        return (
            uwb_ok,
            lidar_ok,
            imu_ok
        )


    # =============================================================
    # Wait for sensors
    # =============================================================

    def wait_for_sensors(
        self
    ):

        start_time = (
            time.monotonic()
        )

        next_log_time = 0.0


        while True:

            if self.shutdown_event.is_set():

                raise RuntimeError(
                    'system manager shutdown requested'
                )


            for name in (
                'motor_serial',
                'ydlidar',
                'lidar_avoidance',
                'uwb_tracker',
                'imu_node'
            ):

                self.require_child_alive(
                    name
                )


            (
                uwb_ok,
                lidar_ok,
                imu_ok
            ) = self.sensor_status()


            if (
                uwb_ok
                and
                lidar_ok
                and
                imu_ok
            ):

                self.get_logger().info(
                    'SENSORS READY | '
                    'UWB=OK | '
                    'IMU=OK | '
                    'LIDAR=OK'
                )

                return


            now = time.monotonic()


            if (
                now
                >=
                next_log_time
            ):

                self.get_logger().info(
                    'WAIT SENSOR | '
                    f'UWB={"OK" if uwb_ok else "--"} | '
                    f'IMU={"OK" if imu_ok else "--"} | '
                    f'LIDAR={"OK" if lidar_ok else "--"}'
                )

                next_log_time = (
                    now
                    +
                    1.0
                )


            if (
                now
                -
                start_time
                >
                SENSOR_READY_TIMEOUT
            ):

                raise RuntimeError(
                    'sensor ready timeout'
                )


            time.sleep(
                0.05
            )


    # =============================================================
    # WAIT -> ACTIVE
    # =============================================================

    def start_system(
        self
    ):

        try:

            # =====================================================
            # STARTING
            #
            # YELLOW fast blink
            # =====================================================

            self.set_phase(
                'STARTING'
            )


            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'SYSTEM STARTING'
            )

            self.get_logger().info(
                'YELLOW FAST BLINK'
            )

            self.get_logger().info(
                '========================================'
            )


            # =====================================================
            # Motor serial
            # =====================================================

            self.start_child(
                'motor_serial',
                [
                    'ros2',
                    'run',
                    'cart_follow',
                    'motor_serial'
                ]
            )


            self.sleep_checked(
                2.5,
                (
                    'motor_serial',
                )
            )


            # motor_serial이 올라온 뒤
            # hard stop 재확인

            self.force_stop()


            # =====================================================
            # YDLIDAR
            # =====================================================

            self.start_child(
                'ydlidar',
                [
                    'ros2',
                    'launch',
                    'ydlidar_ros2_driver',
                    'ydlidar_launch.py'
                ]
            )


            self.sleep_checked(
                3.0,
                (
                    'motor_serial',
                    'ydlidar'
                )
            )


            # =====================================================
            # LiDAR avoidance
            # =====================================================

            self.start_child(
                'lidar_avoidance',
                [
                    'ros2',
                    'run',
                    'cart_follow',
                    'lidar_avoidance'
                ]
            )


            self.sleep_checked(
                1.0,
                (
                    'motor_serial',
                    'ydlidar',
                    'lidar_avoidance'
                )
            )


            # =====================================================
            # UWB
            # =====================================================

            self.start_child(
                'uwb_tracker',
                [
                    'ros2',
                    'run',
                    'cart_follow',
                    'uwb_tracker'
                ]
            )


            self.sleep_checked(
                3.0,
                (
                    'motor_serial',
                    'ydlidar',
                    'lidar_avoidance',
                    'uwb_tracker'
                )
            )


            # =====================================================
            # IMU
            #
            # Motor hard stop 상태에서 calibration
            # =====================================================

            self.start_child(
                'imu_node',
                [
                    'ros2',
                    'run',
                    'cart_follow',
                    'imu_node'
                ]
            )


            self.sleep_checked(
                0.5,
                (
                    'motor_serial',
                    'ydlidar',
                    'lidar_avoidance',
                    'uwb_tracker',
                    'imu_node'
                )
            )


            # =====================================================
            # Sensor ready
            # =====================================================

            self.wait_for_sensors()


            # =====================================================
            # Follow controller
            # =====================================================

            self.start_child(
                'follow_controller',
                [
                    'ros2',
                    'run',
                    'cart_follow',
                    'follow_controller'
                ]
            )


            self.sleep_checked(
                1.5,
                (
                    'motor_serial',
                    'ydlidar',
                    'lidar_avoidance',
                    'uwb_tracker',
                    'imu_node',
                    'follow_controller'
                )
            )


            # =====================================================
            # Final sensor check
            # =====================================================

            (
                uwb_ok,
                lidar_ok,
                imu_ok
            ) = self.sensor_status()


            if not uwb_ok:

                raise RuntimeError(
                    'UWB became unavailable before ACTIVE'
                )


            if not lidar_ok:

                raise RuntimeError(
                    'LiDAR became unavailable before ACTIVE'
                )


            if not imu_ok:

                raise RuntimeError(
                    'IMU became unavailable before ACTIVE'
                )


            # =====================================================
            # Release motors
            # =====================================================

            self.publish_zero()

            self.release_hard_stop()


            # =====================================================
            # ACTIVE
            # =====================================================

            with self.state_lock:

                self.active = True


            self.emergency_active = False

            self.last_motor_left = 0
            self.last_motor_right = 0


            self.set_phase(
                'ACTIVE'
            )


            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'SYSTEM = ACTIVE'
            )

            self.get_logger().info(
                'ROBOT FOLLOW ENABLED'
            )

            self.get_logger().info(
                'GREEN = MOVING | RED = NORMAL STOP'
            )

            self.get_logger().info(
                '========================================'
            )


        except Exception as e:

            if not self.shutting_down:

                self.get_logger().error(
                    f'SYSTEM START FAILED: {e}'
                )


            self.force_stop()


            with self.state_lock:

                self.active = False


            self.stop_all_children()


            self.set_phase(
                'WAIT'
            )


            if not self.shutting_down:

                self.get_logger().warn(
                    'SYSTEM RETURNED TO WAIT'
                )

                self.get_logger().info(
                    'SCAN NFC CARD'
                )


        finally:

            with self.state_lock:

                self.busy = False


    # =============================================================
    # ACTIVE -> WAIT
    # =============================================================

    def stop_system(
        self
    ):

        try:

            # =====================================================
            # 카드가 정상 인식됐다는 것을
            # LED로 바로 보여준다.
            #
            # RED였더라도 즉시
            # YELLOW FAST BLINK로 변경.
            # =====================================================

            self.set_phase(
                'STOPPING'
            )


            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'SYSTEM STOPPING'
            )

            self.get_logger().info(
                'YELLOW FAST BLINK'
            )

            self.get_logger().info(
                '========================================'
            )


            # =====================================================
            # 실제 모터를 가장 먼저 정지
            # =====================================================

            self.force_stop()


            with self.state_lock:

                self.active = False


            self.emergency_active = False

            self.last_motor_left = 0
            self.last_motor_right = 0


            # =====================================================
            # Child shutdown
            # =====================================================

            self.stop_all_children()


            # =====================================================
            # WAIT
            #
            # YELLOW slow blink
            # =====================================================

            self.set_phase(
                'WAIT'
            )


            if not self.shutting_down:

                self.get_logger().warn(
                    'SYSTEM RETURNED TO WAIT'
                )

                self.get_logger().info(
                    'YELLOW SLOW BLINK'
                )

                self.get_logger().info(
                    'SCAN NFC CARD'
                )


        finally:

            with self.state_lock:

                self.busy = False


    # =============================================================
    # NFC
    # =============================================================

    def reset_wait_card_hold(self):
        self.wait_card_start = None
        self.wait_card_last_seen = 0.0
        self.wait_card_long_fired = False

    def read_present_card_uid(self):
        reader = self.nfc.READER

        status, _ = reader.MFRC522_Request(
            reader.PICC_REQALL
        )

        if status != reader.MI_OK:
            return None

        status, uid = reader.MFRC522_Anticoll()

        if status != reader.MI_OK:
            return None

        return self.nfc.uid_to_num(uid)

    def shutdown_host(self):
        try:
            self.set_phase(
                'SHUTDOWN'
            )

            self.get_logger().warn(
                'NFC HOLD 3 SEC -> RASPBERRY PI SHUTDOWN'
            )

            self.force_stop()

            with self.state_lock:
                self.active = False

            self.emergency_active = False
            self.last_motor_left = 0
            self.last_motor_right = 0

            self.stop_all_children()
            self.force_stop()

            time.sleep(
                1.0
            )

            subprocess.run(
                [
                    'sudo',
                    '-n',
                    '/usr/sbin/shutdown',
                    '-h',
                    'now'
                ],
                check=True
            )

        except Exception as e:
            self.get_logger().error(
                f'SHUTDOWN FAILED: {e}'
            )

            with self.state_lock:
                self.active = False
                self.busy = False

            self.set_phase(
                'WAIT'
            )

    def poll_nfc(
        self
    ):

        if self.shutting_down:
            return

        now = time.monotonic()

        try:
            uid = self.read_present_card_uid()

        except Exception as e:
            self.get_logger().error(
                f'NFC read error: {e}'
            )
            return

        if uid:
            self.last_card_seen = now

            if self.card_latched:
                return

            uid_text = str(
                uid
            )

            if uid_text != NFC_UID:
                self.get_logger().warn(
                    f'NFC REJECTED | UID={uid_text}'
                )
                self.card_latched = True
                self.reset_wait_card_hold()
                return

            with self.state_lock:
                active = self.active
                busy = self.busy
                phase = self.system_phase

            if busy:
                self.card_latched = True
                self.reset_wait_card_hold()
                return

            if active:
                self.get_logger().info(
                    'NFC ACCEPTED | ACTIVE -> WAIT'
                )

                self.card_latched = True
                self.reset_wait_card_hold()

                with self.state_lock:
                    if self.busy:
                        return
                    self.busy = True

                self.set_phase(
                    'STOPPING'
                )

                worker = threading.Thread(
                    target=self.stop_system,
                    daemon=True
                )
                worker.start()
                return

            if phase != 'WAIT':
                self.card_latched = True
                self.reset_wait_card_hold()
                return

            if self.wait_card_start is None:
                self.wait_card_start = now
                self.wait_card_last_seen = now
                self.wait_card_long_fired = False

                self.get_logger().info(
                    'NFC HOLD START | RELEASE < 3 SEC = START | HOLD >= 3 SEC = SHUTDOWN'
                )
                return

            self.wait_card_last_seen = now

            hold_time = (
                now
                -
                self.wait_card_start
            )

            if (
                hold_time
                >=
                NFC_SHUTDOWN_HOLD_TIME
                and
                not self.wait_card_long_fired
            ):
                self.wait_card_long_fired = True
                self.card_latched = True

                self.get_logger().warn(
                    f'NFC LONG HOLD = {hold_time:.1f}s'
                )

                with self.state_lock:
                    if self.busy:
                        return
                    self.busy = True

                worker = threading.Thread(
                    target=self.shutdown_host,
                    daemon=True
                )
                worker.start()

            return

        if self.card_latched:
            if (
                now
                -
                self.last_card_seen
                >=
                CARD_REMOVE_TIME
            ):
                self.card_latched = False
                self.reset_wait_card_hold()

                self.get_logger().info(
                    'NFC CARD REMOVED | READY'
                )
            return

        if self.wait_card_start is not None:
            missing_time = (
                now
                -
                self.wait_card_last_seen
            )

            if (
                missing_time
                <
                NFC_RELEASE_CONFIRM_TIME
            ):
                return

            held_time = (
                self.wait_card_last_seen
                -
                self.wait_card_start
            )

            long_fired = self.wait_card_long_fired

            self.reset_wait_card_hold()

            if long_fired:
                return

            if (
                held_time
                <
                NFC_SHUTDOWN_HOLD_TIME
            ):
                self.get_logger().info(
                    f'NFC SHORT TAP = {held_time:.1f}s | START SYSTEM'
                )

                with self.state_lock:
                    if self.busy:
                        return

                    if self.active:
                        return

                    if self.system_phase != 'WAIT':
                        return

                    self.busy = True

                self.set_phase(
                    'STARTING'
                )

                worker = threading.Thread(
                    target=self.start_system,
                    daemon=True
                )
                worker.start()

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(
        self
    ):

        if self.shutting_down:

            return super().destroy_node()


        self.shutting_down = True

        self.shutdown_event.set()


        # =========================================================
        # Motor safety first
        # =========================================================

        try:

            self.force_stop()

        except Exception:

            pass


        # =========================================================
        # Stop children
        # =========================================================

        try:

            self.stop_all_children()

        except Exception:

            pass


        # =========================================================
        # LEDs OFF
        #
        # 프로그램이 완전히 종료되면
        # 세 LED 모두 끈다.
        # =========================================================

        try:

            self.write_leds(
                False,
                False,
                False
            )

        except Exception:

            pass


        try:

            GPIO.cleanup()

        except Exception:

            pass


        return super().destroy_node()


# =============================================================
# Main
# =============================================================

def main(
    args=None
):

    rclpy.init(
        args=args
    )


    node = SystemManager()


    try:

        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        pass


    finally:

        try:

            node.destroy_node()

        except Exception:

            pass


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()