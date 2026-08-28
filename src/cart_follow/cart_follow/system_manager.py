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

NFC_UID = '736167527119'

# =============================================================
# Status LEDs - physical pin numbers (GPIO.BOARD)
# RED    : Pin 11 = GPIO17
# YELLOW : Pin 13 = GPIO27
# GREEN  : Pin 15 = GPIO22
#
# LED rule:
#   WAIT / STARTING -> YELLOW
#   ACTIVE + STOP   -> RED
#   ACTIVE + MOTION -> GREEN
#   Emergency pause -> RED
#   Emergency reverse -> GREEN
#
# Only ONE LED is ever commanded ON.
# =============================================================

LED_RED_PIN = 11
LED_YELLOW_PIN = 13
LED_GREEN_PIN = 15

SENSOR_FRESH_TIME = 1.0
SENSOR_READY_TIMEOUT = 20.0

CARD_REMOVE_TIME = 1.0
NFC_POLL_INTERVAL = 0.10


class SystemManager(Node):

    def __init__(self):
        super().__init__('system_manager')

        # =========================================================
        # State
        # =========================================================

        self.active = False
        self.busy = False
        self.shutting_down = False

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

        # Motor command is also observed by system_manager only for LED status.
        # [0, 0] while ACTIVE -> RED
        # any non-zero motion (including emergency reverse) -> GREEN
        self.last_motor_left = 0
        self.last_motor_right = 0

        self.create_subscription(
            Int32MultiArray,
            '/motor_cmd_pwm',
            self.cb_motor_cmd,
            10
        )

        # OLED removed: I2C bus is reserved for MPU6050
        self.get_logger().info('OLED REMOVED | I2C reserved for MPU6050')

        # =========================================================
        # NFC
        # =========================================================

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        # RC522 uses the same BOARD numbering convention.
        self.nfc = SimpleMFRC522()

        # Status LED outputs. GPIO HIGH = maximum software brightness.
        GPIO.setup(LED_RED_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(LED_YELLOW_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(LED_GREEN_PIN, GPIO.OUT, initial=GPIO.LOW)

        self.led_state = None

        # LED phase
        # - power-on before first accepted card: all OFF
        # - card accepted / sensor loading: YELLOW
        # - ACTIVE motion: GREEN
        # - ACTIVE stop or system stopped: RED
        self.starting = False
        self.has_started = False

        self.set_status_led('OFF')

        self.card_latched = False
        self.last_card_seen = 0.0

        self.nfc_timer = self.create_timer(
            NFC_POLL_INTERVAL,
            self.poll_nfc
        )

        # =========================================================
        # Initial safe state
        # =========================================================

        self.publish_hard_stop(True)
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
            'SCAN NFC CARD'
        )
        self.get_logger().info(
            '========================================'
        )

    # =============================================================
    # Status LED
    # =============================================================

    def set_status_led(self, color):
        """
        Exactly one status LED is commanded ON.

        OFF    = power-on before the first accepted NFC card
        RED    = stopped / emergency pause
        YELLOW = card accepted, sensors loading
        GREEN  = moving / emergency reverse
        """
        color = str(color).upper()

        if color not in ('OFF', 'RED', 'YELLOW', 'GREEN'):
            color = 'OFF'

        with self.led_lock:
            if color == self.led_state:
                return

            if color == 'OFF':
                values = (GPIO.LOW, GPIO.LOW, GPIO.LOW)
            elif color == 'RED':
                values = (GPIO.HIGH, GPIO.LOW, GPIO.LOW)
            elif color == 'YELLOW':
                values = (GPIO.LOW, GPIO.HIGH, GPIO.LOW)
            else:  # GREEN
                values = (GPIO.LOW, GPIO.LOW, GPIO.HIGH)

            # One call with three outputs: there is never a requested state
            # in which two LEDs are ON at the same time.
            GPIO.output(
                (LED_RED_PIN, LED_YELLOW_PIN, LED_GREEN_PIN),
                values
            )

            self.led_state = color

        try:
            self.get_logger().info(f'STATUS LED -> {color}')
        except Exception:
            pass

    def update_led_from_motion(self):
        with self.state_lock:
            active = self.active

        if not active:
            if self.starting:
                self.set_status_led('YELLOW')
            elif self.has_started:
                self.set_status_led('RED')
            else:
                self.set_status_led('OFF')
            return

        # ACTIVE state:
        # exact zero command means stopped -> RED.
        # Any motor command, forward/turn/reverse, means motion -> GREEN.
        if self.last_motor_left == 0 and self.last_motor_right == 0:
            self.set_status_led('RED')
        else:
            self.set_status_led('GREEN')

    # =============================================================
    # Sensor callbacks
    # =============================================================

    def cb_uwb_direction(self, msg):
        direction = str(msg.data).strip().upper()
        self.uwb_direction = direction

        # LOST는 ready 데이터로 취급하지 않는다.
        if direction in (
            'LEFT',
            'CENTER',
            'RIGHT'
        ):
            self.last_uwb = time.monotonic()

    def cb_scan(self, msg):
        self.last_lidar = time.monotonic()

    def cb_imu(self, msg):
        # 실제 값 자체보다 fresh message 수신 여부를 readiness로 사용
        self.last_imu = time.monotonic()

    def cb_motor_cmd(self, msg):
        if len(msg.data) < 2:
            return

        self.last_motor_left = int(msg.data[0])
        self.last_motor_right = int(msg.data[1])

        self.update_led_from_motion()

    # =============================================================
    # Motor safety
    # =============================================================

    def publish_hard_stop(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.hard_stop_pub.publish(msg)

    def publish_zero(self):
        msg = Int32MultiArray()
        msg.data = [0, 0]
        self.motor_pub.publish(msg)

    def force_stop(self, repeats=5):
        for _ in range(repeats):
            if self.shutting_down and not rclpy.ok():
                break

            try:
                self.publish_hard_stop(True)
                self.publish_zero()
            except Exception:
                pass

            time.sleep(0.05)

    def release_hard_stop(self, repeats=5):
        # 해제 직전에도 반드시 0 명령을 보낸다.
        for _ in range(repeats):
            self.publish_zero()
            self.publish_hard_stop(False)
            time.sleep(0.05)

    # =============================================================
    # Process management
    # =============================================================

    def start_child(self, name, command):
        if self.shutdown_event.is_set():
            raise RuntimeError('system manager is shutting down')

        with self.child_lock:
            old = self.children.get(name)

            if old is not None and old.poll() is None:
                return

            self.get_logger().info(
                f'START -> {name}'
            )

            process = subprocess.Popen(
                command,
                start_new_session=True
            )

            self.children[name] = process

    def child_alive(self, name):
        with self.child_lock:
            process = self.children.get(name)

        return (
            process is not None
            and
            process.poll() is None
        )

    def require_child_alive(self, name):
        if not self.child_alive(name):
            raise RuntimeError(
                f'{name} failed during startup'
            )

    def stop_child(self, name):
        with self.child_lock:
            process = self.children.get(name)

        if process is None:
            return

        if process.poll() is not None:
            with self.child_lock:
                self.children.pop(name, None)
            return

        try:
            self.get_logger().info(
                f'STOP -> {name}'
            )
        except Exception:
            pass

        # ---------------------------------------------------------
        # 1. SIGINT
        # ---------------------------------------------------------

        try:
            os.killpg(
                os.getpgid(process.pid),
                signal.SIGINT
            )
        except ProcessLookupError:
            pass

        try:
            process.wait(timeout=3.0)

        except subprocess.TimeoutExpired:

            # -----------------------------------------------------
            # 2. SIGTERM
            # -----------------------------------------------------

            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGTERM
                )
            except ProcessLookupError:
                pass

            try:
                process.wait(timeout=2.0)

            except subprocess.TimeoutExpired:

                # -------------------------------------------------
                # 3. SIGKILL
                # -------------------------------------------------

                try:
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGKILL
                    )
                except ProcessLookupError:
                    pass

                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass

        with self.child_lock:
            self.children.pop(name, None)

    def stop_all_children(self):
        # Controller를 가장 먼저 끄고 motor_serial을 마지막에 끈다.
        stop_order = (
            'follow_controller',
            'imu_node',
            'uwb_tracker',
            'lidar_avoidance',
            'ydlidar',
            'motor_serial'
        )

        for name in stop_order:
            self.stop_child(name)

        # 혹시 등록은 됐지만 stop_order에 없는 프로세스가 있으면 정리
        with self.child_lock:
            leftovers = list(self.children.keys())

        for name in leftovers:
            self.stop_child(name)

    # =============================================================
    # Abortable sleep
    # =============================================================

    def sleep_checked(self, seconds, child_names=()):
        end_time = time.monotonic() + seconds

        while time.monotonic() < end_time:
            if self.shutdown_event.is_set():
                raise RuntimeError(
                    'system manager shutdown requested'
                )

            for name in child_names:
                self.require_child_alive(name)

            time.sleep(0.05)

    # =============================================================
    # Sensor readiness
    # =============================================================

    def sensor_status(self):
        now = time.monotonic()

        uwb_ok = (
            self.uwb_direction
            in
            ('LEFT', 'CENTER', 'RIGHT')
            and
            now - self.last_uwb
            <=
            SENSOR_FRESH_TIME
        )

        lidar_ok = (
            now - self.last_lidar
            <=
            SENSOR_FRESH_TIME
        )

        imu_ok = (
            now - self.last_imu
            <=
            SENSOR_FRESH_TIME
        )

        return uwb_ok, lidar_ok, imu_ok

    def wait_for_sensors(self):
        start_time = time.monotonic()
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
                self.require_child_alive(name)

            uwb_ok, lidar_ok, imu_ok = self.sensor_status()

            if uwb_ok and lidar_ok and imu_ok:
                self.get_logger().info(
                    'SENSORS READY | '
                    'UWB=OK | '
                    'IMU=OK | '
                    'LIDAR=OK'
                )
                return

            now = time.monotonic()

            if now >= next_log_time:
                self.get_logger().info(
                    'WAIT SENSOR | '
                    f'UWB={"OK" if uwb_ok else "--"} | '
                    f'IMU={"OK" if imu_ok else "--"} | '
                    f'LIDAR={"OK" if lidar_ok else "--"}'
                )
                next_log_time = now + 1.0

            if (
                now - start_time
                >
                SENSOR_READY_TIMEOUT
            ):
                raise RuntimeError(
                    'sensor ready timeout'
                )

            time.sleep(0.05)

    # =============================================================
    # WAIT -> ACTIVE
    # =============================================================

    def start_system(self):
        try:
            # Accepted card -> loading indication until ACTIVE.
            self.starting = True
            self.set_status_led('YELLOW')

            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info(
                'SYSTEM STARTING'
            )
            self.get_logger().info(
                '========================================'
            )

            # -----------------------------------------------------
            # Motor serial
            # -----------------------------------------------------

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
                ('motor_serial',)
            )

            # motor_serial이 완전히 올라온 뒤 hard stop 재확인
            self.force_stop()

            # -----------------------------------------------------
            # YDLIDAR
            # -----------------------------------------------------

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

            # -----------------------------------------------------
            # LiDAR avoidance
            # -----------------------------------------------------

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

            # -----------------------------------------------------
            # UWB
            # -----------------------------------------------------

            self.start_child(
                'uwb_tracker',
                [
                    'ros2',
                    'run',
                    'cart_follow',
                    'uwb_tracker'
                ]
            )

            # Stella serial open + 초기 데이터 안정화 시간
            self.sleep_checked(
                3.0,
                (
                    'motor_serial',
                    'ydlidar',
                    'lidar_avoidance',
                    'uwb_tracker'
                )
            )

            # -----------------------------------------------------
            # IMU
            #
            # hard stop이 걸린 상태에서 시작하므로 calibration 동안
            # 차체가 움직이지 않는다. OLED는 제거되어 I2C에는 MPU6050만 있다.
            # -----------------------------------------------------

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

            # -----------------------------------------------------
            # UWB + IMU + LiDAR ready
            # -----------------------------------------------------

            self.wait_for_sensors()

            # -----------------------------------------------------
            # Follow controller
            # -----------------------------------------------------

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

            # 센서가 시작 완료 직전까지 살아있는지 최종 확인
            uwb_ok, lidar_ok, imu_ok = self.sensor_status()

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

            self.publish_zero()
            self.release_hard_stop()

            with self.state_lock:
                self.active = True

            self.starting = False
            self.has_started = True

            # ACTIVE but still stopped -> RED. As soon as motor command
            # becomes non-zero, cb_motor_cmd changes it to GREEN.
            self.update_led_from_motion()

            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info(
                'SYSTEM = ACTIVE'
            )
            self.get_logger().info(
                'ROBOT FOLLOW ENABLED | IMU yaw damping available'
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

            self.starting = False
            self.has_started = True
            self.set_status_led('RED')
            self.stop_all_children()

            if not self.shutting_down:
                self.get_logger().warn(
                    'SYSTEM RETURNED TO WAIT'
                )

        finally:
            with self.state_lock:
                self.busy = False

    # =============================================================
    # ACTIVE -> WAIT
    # =============================================================

    def stop_system(self):
        try:
            # Stop transition: RED and remain RED after stop.
            self.starting = False
            self.has_started = True
            self.set_status_led('RED')

            # 가장 먼저 실제 모터를 정지시킨다.
            self.force_stop()

            with self.state_lock:
                self.active = False

            self.stop_all_children()

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
    # NFC
    # =============================================================

    def poll_nfc(self):
        if self.shutting_down:
            return

        now = time.monotonic()

        try:
            uid = self.nfc.read_id_no_block()

        except Exception as e:
            self.get_logger().error(
                f'NFC read error: {e}'
            )
            return

        if uid:
            self.last_card_seen = now

            if self.card_latched:
                return

            self.card_latched = True

            uid_text = str(uid)

            self.get_logger().info(
                f'NFC UID = {uid_text}'
            )

            if uid_text != NFC_UID:
                self.get_logger().warn(
                    'NFC REJECTED'
                )
                return

            self.get_logger().info(
                'NFC ACCEPTED'
            )

            with self.state_lock:
                if self.busy:
                    return

                self.busy = True
                currently_active = self.active

            if currently_active:
                worker = threading.Thread(
                    target=self.stop_system,
                    daemon=True
                )
            else:
                worker = threading.Thread(
                    target=self.start_system,
                    daemon=True
                )

            worker.start()
            return

        # 카드가 충분히 오래 사라진 뒤에만 다음 태그를 허용
        if (
            self.card_latched
            and
            now - self.last_card_seen
            >=
            CARD_REMOVE_TIME
        ):
            self.card_latched = False

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(self):
        if self.shutting_down:
            return super().destroy_node()

        self.shutting_down = True
        self.shutdown_event.set()

        try:
            self.set_status_led('RED')
        except Exception:
            pass

        try:
            self.force_stop()
        except Exception:
            pass


        try:
            self.stop_all_children()
        except Exception:
            pass

        try:
            GPIO.cleanup()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SystemManager()

    try:
        rclpy.spin(node)

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
