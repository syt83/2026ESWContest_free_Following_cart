import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray


# =============================================================
# RC522
# =============================================================

try:
    from mfrc522 import SimpleMFRC522

    RC522_AVAILABLE = True

except ImportError:

    RC522_AVAILABLE = False


class SystemManager(Node):

    def __init__(self):

        super().__init__(
            'system_manager'
        )

        # =========================================================
        # 상태
        # =========================================================

        self.active = False

        self.starting = False

        self.stopping = False

        # =========================================================
        # NFC
        # =========================================================

        # 같은 카드를 계속 올려놨을 때
        # ON/OFF가 반복되는 것 방지
        self.NFC_COOLDOWN = 2.0

        self.last_card_time = 0.0

        self.last_card_id = None

        # =========================================================
        # 등록 카드
        #
        # 처음에는 None으로 두면
        # 모든 카드 허용
        #
        # UID 확인 후 숫자로 넣으면 됨.
        # =========================================================

        self.ALLOWED_CARD_ID = 736167527119

        # 예:
        #
        # self.ALLOWED_CARD_ID = 123456789012
        #

        # =========================================================
        # 실행 중인 프로세스
        # =========================================================

        self.processes = []

        # =========================================================
        # Motor safety publisher
        # =========================================================

        self.hard_stop_pub = self.create_publisher(
            Bool,
            '/motor_hard_stop',
            10
        )

        self.motor_cmd_pub = self.create_publisher(
            Int32MultiArray,
            '/motor_cmd_pwm',
            10
        )

        # =========================================================
        # RC522 초기화
        # =========================================================

        self.reader = None

        if RC522_AVAILABLE:

            try:

                self.reader = SimpleMFRC522()

                self.get_logger().info(
                    'RC522 initialized'
                )

            except Exception as e:

                self.get_logger().error(
                    f'RC522 init failed: {e}'
                )

        else:

            self.get_logger().error(
                'mfrc522 Python library not installed'
            )

        # =========================================================
        # STANDBY 상태로 시작
        # =========================================================

        self.publish_hard_stop(
            True
        )

        self.publish_motor_stop()

        # =========================================================
        # NFC polling
        # =========================================================

        self.timer = self.create_timer(
            0.10,
            self.update
        )

        self.get_logger().info(
            '==================================='
        )

        self.get_logger().info(
            'SYSTEM MANAGER STARTED'
        )

        self.get_logger().info(
            'STATE = STANDBY'
        )

        self.get_logger().info(
            'SCAN NFC CARD'
        )

        self.get_logger().info(
            '==================================='
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

        # 여러 번 보내서
        # subscriber 시작 타이밍 차이 대응
        for _ in range(3):

            self.hard_stop_pub.publish(
                msg
            )

    def publish_motor_stop(
        self
    ):

        msg = Int32MultiArray()

        msg.data = [
            0,
            0
        ]

        for _ in range(3):

            self.motor_cmd_pub.publish(
                msg
            )

    # =============================================================
    # Process start
    # =============================================================

    def start_process(
        self,
        name,
        command
    ):

        self.get_logger().info(
            f'STARTING -> {name}'
        )

        try:

            process = subprocess.Popen(
                command,
                preexec_fn=os.setsid
            )

            self.processes.append(
                (
                    name,
                    process
                )
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f'FAILED TO START {name}: {e}'
            )

            return False

    # =============================================================
    # Robot start
    # =============================================================

    def activate_system(
        self
    ):

        if self.active:
            return

        if self.starting:
            return

        self.starting = True

        self.get_logger().warn(
            '==================================='
        )

        self.get_logger().warn(
            'NFC ACCEPTED'
        )

        self.get_logger().warn(
            'SYSTEM -> ACTIVE'
        )

        self.get_logger().warn(
            '==================================='
        )

        # =========================================================
        # 시작 중에는 모터 잠금 유지
        # =========================================================

        self.publish_hard_stop(
            True
        )

        self.publish_motor_stop()

        # =========================================================
        # 기존 process 정리
        # =========================================================

        self.processes.clear()

        # =========================================================
        # 1. LiDAR driver
        # =========================================================

        if not self.start_process(
            'YDLIDAR DRIVER',
            [
                'ros2',
                'launch',
                'ydlidar_ros2_driver',
                'ydlidar_launch.py'
            ]
        ):

            self.start_failed()

            return

        # LiDAR 회전 및 USB 초기화 대기
        time.sleep(
            2.5
        )

        # =========================================================
        # 2. LiDAR avoidance
        # =========================================================

        if not self.start_process(
            'LIDAR AVOIDANCE',
            [
                'ros2',
                'run',
                'cart_follow',
                'lidar_avoidance'
            ]
        ):

            self.start_failed()

            return

        time.sleep(
            0.5
        )

        # =========================================================
        # 3. UWB
        # =========================================================

        if not self.start_process(
            'UWB TRACKER',
            [
                'ros2',
                'run',
                'cart_follow',
                'uwb_tracker'
            ]
        ):

            self.start_failed()

            return

        # Stella serial open 시 초기화 시간이 있으므로
        # 기존 코드와 맞춰 충분히 대기
        time.sleep(
            2.5
        )

        # =========================================================
        # 4. IMU
        # =========================================================

        if not self.start_process(
            'IMU',
            [
                'ros2',
                'run',
                'cart_follow',
                'imu_node'
            ]
        ):

            self.start_failed()

            return

        # MPU6050 gyro calibration 시간
        time.sleep(
            3.5
        )

        # =========================================================
        # 5. Follow controller
        # =========================================================

        if not self.start_process(
            'FOLLOW CONTROLLER',
            [
                'ros2',
                'run',
                'cart_follow',
                'follow_controller'
            ]
        ):

            self.start_failed()

            return

        time.sleep(
            1.0
        )

        # =========================================================
        # 6. Motor serial
        #
        # 항상 마지막
        # =========================================================

        if not self.start_process(
            'MOTOR SERIAL',
            [
                'ros2',
                'run',
                'cart_follow',
                'motor_serial'
            ]
        ):

            self.start_failed()

            return

        # Arduino serial reset 대기
        time.sleep(
            2.5
        )

        # =========================================================
        # Motor release
        # =========================================================

        self.publish_motor_stop()

        self.publish_hard_stop(
            False
        )

        self.active = True

        self.starting = False

        self.get_logger().info(
            '==================================='
        )

        self.get_logger().info(
            'ROBOT READY'
        )

        self.get_logger().info(
            'STATE = ACTIVE'
        )

        self.get_logger().info(
            '==================================='
        )

    # =============================================================
    # 시작 실패
    # =============================================================

    def start_failed(
        self
    ):

        self.get_logger().error(
            'SYSTEM START FAILED'
        )

        self.publish_hard_stop(
            True
        )

        self.publish_motor_stop()

        self.stop_all_processes()

        self.active = False

        self.starting = False

        self.get_logger().warn(
            'RETURN TO STANDBY'
        )

    # =============================================================
    # Robot deactivate
    # =============================================================

    def deactivate_system(
        self
    ):

        if self.stopping:
            return

        self.stopping = True

        self.get_logger().warn(
            '==================================='
        )

        self.get_logger().warn(
            'SYSTEM -> STANDBY'
        )

        self.get_logger().warn(
            'STOPPING ROBOT'
        )

        self.get_logger().warn(
            '==================================='
        )

        # =========================================================
        # 1. 모터 즉시 정지
        # =========================================================

        self.publish_hard_stop(
            True
        )

        self.publish_motor_stop()

        time.sleep(
            0.5
        )

        # =========================================================
        # 2. 모든 ROS process 종료
        # =========================================================

        self.stop_all_processes()

        self.active = False

        self.starting = False

        self.stopping = False

        self.get_logger().info(
            '==================================='
        )

        self.get_logger().info(
            'STATE = STANDBY'
        )

        self.get_logger().info(
            'SCAN NFC CARD'
        )

        self.get_logger().info(
            '==================================='
        )

    # =============================================================
    # 모든 process 종료
    # =============================================================

    def stop_all_processes(
        self
    ):

        # =========================================================
        # 역순 종료
        #
        # Motor
        # Controller
        # IMU
        # UWB
        # LiDAR avoidance
        # LiDAR driver
        # =========================================================

        for (
            name,
            process
        ) in reversed(
            self.processes
        ):

            try:

                if (
                    process.poll()
                    is None
                ):

                    self.get_logger().info(
                        f'STOPPING -> {name}'
                    )

                    os.killpg(
                        os.getpgid(
                            process.pid
                        ),
                        signal.SIGINT
                    )

                    try:

                        process.wait(
                            timeout=3.0
                        )

                    except subprocess.TimeoutExpired:

                        self.get_logger().warn(
                            f'FORCE STOP -> {name}'
                        )

                        os.killpg(
                            os.getpgid(
                                process.pid
                            ),
                            signal.SIGTERM
                        )

            except Exception as e:

                self.get_logger().warn(
                    f'STOP ERROR {name}: {e}'
                )

        self.processes.clear()

    # =============================================================
    # NFC 처리
    # =============================================================

    def handle_card(
        self,
        card_id
    ):

        now = time.monotonic()

        # =========================================================
        # 동일 카드 debounce
        # =========================================================

        if (
            self.last_card_id
            ==
            card_id
            and
            now
            -
            self.last_card_time
            <
            self.NFC_COOLDOWN
        ):

            return

        self.last_card_id = (
            card_id
        )

        self.last_card_time = now

        self.get_logger().info(
            f'NFC CARD UID = {card_id}'
        )

        # =========================================================
        # 등록 카드 검사
        # =========================================================

        if (
            self.ALLOWED_CARD_ID
            is not None
            and
            card_id
            !=
            self.ALLOWED_CARD_ID
        ):

            self.get_logger().warn(
                'NFC ACCESS DENIED'
            )

            return

        # =========================================================
        # Toggle
        # =========================================================

        if self.active:

            self.deactivate_system()

        else:

            self.activate_system()

    # =============================================================
    # Update
    # =============================================================

    def update(
        self
    ):

        if self.reader is None:
            return

        if self.starting:
            return

        if self.stopping:
            return

        try:

            # SimpleMFRC522 non-blocking read
            card_id = (
                self.reader.read_id_no_block()
            )

            if card_id is None:
                return

            self.handle_card(
                int(card_id)
            )

        except Exception as e:

            self.get_logger().error(
                f'RC522 read error: {e}'
            )

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(
        self
    ):

        self.publish_hard_stop(
            True
        )

        self.publish_motor_stop()

        self.stop_all_processes()

        try:

            import RPi.GPIO as GPIO

            GPIO.cleanup()

        except Exception:

            pass

        super().destroy_node()


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

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
