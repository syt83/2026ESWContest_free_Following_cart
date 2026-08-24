#실행방법 : source /opt/ros/jazzy/setup.bash
#source ~/robot_project/venv/bin/activate
#source ~/robot_project/robot_ws/install/setup.bash
#ros2 run cart_follow uwb_tracker

import re
import time
import statistics
from collections import deque

import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String


LEFT_PORT = (
    "/dev/serial/by-id/"
    "usb-Arduino_Stella_2FF8078C6CD20BAA-if00"
)

RIGHT_PORT = (
    "/dev/serial/by-id/"
    "usb-Arduino_Stella_43211ECF471912FE-if00"
)

BAUD = 115200
FILTER_SIZE = 5

# 기존 실측 보정식
LEFT_A = 0.9629
LEFT_B = -35.36

RIGHT_A = 0.8247
RIGHT_B = -22.04


class UWBTracker(Node):

    def __init__(self):
        super().__init__("uwb_tracker")

        self.pub_left = self.create_publisher(
            Float32, "/uwb/left_distance", 10
        )

        self.pub_right = self.create_publisher(
            Float32, "/uwb/right_distance", 10
        )

        self.pub_error = self.create_publisher(
            Float32, "/uwb/direction_error", 10
        )

        self.pub_distance = self.create_publisher(
            Float32, "/uwb/user_distance", 10
        )

        self.pub_direction = self.create_publisher(
            String, "/uwb/direction", 10
        )

        self.left_serial = serial.Serial(
            LEFT_PORT,
            BAUD,
            timeout=0
        )

        self.right_serial = serial.Serial(
            RIGHT_PORT,
            BAUD,
            timeout=0
        )

        time.sleep(2.0)

        self.left_buffer = deque(maxlen=FILTER_SIZE)
        self.right_buffer = deque(maxlen=FILTER_SIZE)

        self.left_raw = None
        self.right_raw = None

        self.last_left_time = 0.0
        self.last_right_time = 0.0

        self.UWB_TIMEOUT = 1.0
        self.DIRECTION_DEADBAND = 10.0

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info(
            "UWB tracker started"
        )

    def parse_distance(self, line):
        numbers = re.findall(
            r"-?\d+(?:\.\d+)?",
            line
        )

        if not numbers:
            return None

        try:
            return float(numbers[-1])
        except ValueError:
            return None

    def read_latest(self, ser):
        latest = None

        while ser.in_waiting > 0:
            line = ser.readline().decode(
                "utf-8",
                errors="ignore"
            ).strip()

            value = self.parse_distance(line)

            if value is not None:
                latest = value

        return latest

    def correct_left(self, raw):
        return max(
            0.0,
            LEFT_A * raw + LEFT_B
        )

    def correct_right(self, raw):
        return max(
            0.0,
            RIGHT_A * raw + RIGHT_B
        )

    def update(self):
        now = time.monotonic()

        value = self.read_latest(
            self.left_serial
        )

        if value is not None:
            self.left_buffer.append(value)

            self.left_raw = statistics.median(
                self.left_buffer
            )

            self.last_left_time = now

        value = self.read_latest(
            self.right_serial
        )

        if value is not None:
            self.right_buffer.append(value)

            self.right_raw = statistics.median(
                self.right_buffer
            )

            self.last_right_time = now

        if (
            self.left_raw is None
            or self.right_raw is None
            or now - self.last_left_time > self.UWB_TIMEOUT
            or now - self.last_right_time > self.UWB_TIMEOUT
        ):
            msg = String()
            msg.data = "LOST"
            self.pub_direction.publish(msg)

            return

        # 방향은 RAW median 비교
        direction_error = (
            self.left_raw
            -
            self.right_raw
        )

        if direction_error < -self.DIRECTION_DEADBAND:
            direction = "LEFT"

        elif direction_error > self.DIRECTION_DEADBAND:
            direction = "RIGHT"

        else:
            direction = "CENTER"

        # 거리값은 기존 보정식 적용
        left_corrected = self.correct_left(
            self.left_raw
        )

        right_corrected = self.correct_right(
            self.right_raw
        )

        user_distance = (
            left_corrected
            +
            right_corrected
        ) / 2.0

        msg = Float32()
        msg.data = float(left_corrected)
        self.pub_left.publish(msg)

        msg = Float32()
        msg.data = float(right_corrected)
        self.pub_right.publish(msg)

        msg = Float32()
        msg.data = float(direction_error)
        self.pub_error.publish(msg)

        msg = Float32()
        msg.data = float(user_distance)
        self.pub_distance.publish(msg)

        msg2 = String()
        msg2.data = direction
        self.pub_direction.publish(msg2)

        self.get_logger().info(
            f"L={left_corrected:.1f} "
            f"R={right_corrected:.1f} "
            f"D={user_distance:.1f} "
            f"ERR={direction_error:.1f} "
            f"DIR={direction}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = UWBTracker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.left_serial.close()
            node.right_serial.close()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()