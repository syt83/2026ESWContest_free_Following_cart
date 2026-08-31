# 실행:
# ros2 run cart_follow motor_serial

import time
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool


# =========================================================
# BASELINE NOTE
# follow_controller의 38/65/85 계열 값을 그대로 Uno(-255~255)에 보낸다.
# 이전 잘 되던 주행감 재현을 위해 이 버전에서는 퍼센트 -> 255 재스케일을 하지 않는다.
# =========================================================

# =========================================================
# Arduino
# =========================================================

UNO_PORT = (
    '/dev/serial/by-id/'
    'usb-Arduino_Srl_Arduino_Uno_'
    '754303331373510131B2-if00'
)

UNO_BAUD = 9600


class MotorSerial(Node):

    def __init__(self):

        super().__init__(
            'motor_serial'
        )

        # =========================================================
        # Serial
        # =========================================================

        self.ser = serial.Serial(
            UNO_PORT,
            UNO_BAUD,
            timeout=0
        )

        time.sleep(2.0)

        self.ser.reset_input_buffer()

        # =========================================================
        # Motor
        # =========================================================

        self.target_left = 0
        self.target_right = 0

        self.current_left = 0
        self.current_right = 0

        # =========================================================
        # Ramp
        # =========================================================

        self.RAMP_STEP = 3

        # =========================================================
        # Timeout
        # =========================================================

        self.last_cmd_time = (
            time.monotonic()
        )

        self.CMD_TIMEOUT = 0.4

        # =========================================================
        # HARD STOP
        # =========================================================

        self.hard_stop = True

        # =========================================================
        # Subscribers
        # =========================================================

        self.create_subscription(
            Int32MultiArray,
            '/motor_cmd_pwm',
            self.motor_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/motor_hard_stop',
            self.hard_stop_callback,
            10
        )

        # =========================================================
        # 20 Hz
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info(
            'Motor serial started | '
            'HARD STOP DEFAULT ON | '
            'RAMP_STEP=3'
        )

    # =============================================================
    # Motor callback
    # =============================================================

    def motor_callback(
        self,
        msg
    ):

        if len(msg.data) < 2:
            return

        if self.hard_stop:
            return

        self.target_left = int(
            self.clamp(
                msg.data[0],
                -255,
                255
            )
        )

        self.target_right = int(
            self.clamp(
                msg.data[1],
                -255,
                255
            )
        )

        self.last_cmd_time = (
            time.monotonic()
        )

    # =============================================================
    # Hard stop
    # =============================================================

    def hard_stop_callback(
        self,
        msg
    ):

        new_state = bool(
            msg.data
        )

        if (
            new_state
            and
            not self.hard_stop
        ):

            self.get_logger().warn(
                'MOTOR HARD STOP ACTIVATED'
            )

        elif (
            not new_state
            and
            self.hard_stop
        ):

            self.get_logger().info(
                'MOTOR HARD STOP RELEASED'
            )

            self.target_left = 0
            self.target_right = 0

            self.current_left = 0
            self.current_right = 0

            self.last_cmd_time = (
                time.monotonic()
            )

        self.hard_stop = new_state

        if self.hard_stop:

            self.target_left = 0
            self.target_right = 0

            self.current_left = 0
            self.current_right = 0

            self.send_motor(
                0,
                0
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
    # Ramp
    # =============================================================

    def move_toward(
        self,
        current,
        target
    ):

        if current < target:

            current += self.RAMP_STEP

            if current > target:
                current = target

        elif current > target:

            current -= self.RAMP_STEP

            if current < target:
                current = target

        return current

    # =============================================================
    # Send motor
    # =============================================================

    def send_motor(
        self,
        left,
        right
    ):

        command = (
            f'{int(left)},'
            f'{int(right)}\n'
        )

        try:

            self.ser.write(
                command.encode()
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Serial write error: {e}'
            )

    # =============================================================
    # Update
    # =============================================================

    def update(self):

        now = time.monotonic()

        # =========================================================
        # Hard stop
        # =========================================================

        if self.hard_stop:

            self.target_left = 0
            self.target_right = 0

            self.current_left = 0
            self.current_right = 0

            self.send_motor(
                0,
                0
            )

            return

        # =========================================================
        # Command timeout
        # =========================================================

        if (
            now
            -
            self.last_cmd_time
            >
            self.CMD_TIMEOUT
        ):

            self.target_left = 0
            self.target_right = 0

        # =========================================================
        # Ramp
        # =========================================================

        self.current_left = self.move_toward(
            self.current_left,
            self.target_left
        )

        self.current_right = self.move_toward(
            self.current_right,
            self.target_right
        )

        # =========================================================
        # Send
        # =========================================================

        self.send_motor(
            self.current_left,
            self.current_right
        )

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(self):

        try:

            if self.ser.is_open:

                self.send_motor(
                    0,
                    0
                )

                time.sleep(
                    0.05
                )

                self.send_motor(
                    0,
                    0
                )

                time.sleep(
                    0.05
                )

                self.ser.close()

        except Exception:

            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = MotorSerial()

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