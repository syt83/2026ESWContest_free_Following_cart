# 실행:
# ros2 run cart_follow uwb_tracker

import re
import time
import statistics
from collections import deque

import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String


# =============================================================
# Stella serial ports
# =============================================================

LEFT_PORT = (
    "/dev/serial/by-id/"
    "usb-Arduino_Stella_2FF8078C6CD20BAA-if00"
)

RIGHT_PORT = (
    "/dev/serial/by-id/"
    "usb-Arduino_Stella_43211ECF471912FE-if00"
)

BAUD = 115200


# =============================================================
# UWB median filter
# =============================================================

FILTER_SIZE = 5


# =============================================================
# 기존 실측 거리 보정식
# =============================================================

LEFT_A = 0.9629
LEFT_B = -35.36

RIGHT_A = 0.8247
RIGHT_B = -22.04


class UWBTracker(Node):

    def __init__(self):

        super().__init__("uwb_tracker")

        # =========================================================
        # Publishers
        # =========================================================

        self.pub_left = self.create_publisher(
            Float32,
            "/uwb/left_distance",
            10
        )

        self.pub_right = self.create_publisher(
            Float32,
            "/uwb/right_distance",
            10
        )

        self.pub_error = self.create_publisher(
            Float32,
            "/uwb/direction_error",
            10
        )

        self.pub_distance = self.create_publisher(
            Float32,
            "/uwb/user_distance",
            10
        )

        self.pub_direction = self.create_publisher(
            String,
            "/uwb/direction",
            10
        )

        # =========================================================
        # Serial
        # =========================================================

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

        # =========================================================
        # Median buffers
        # =========================================================

        self.left_buffer = deque(
            maxlen=FILTER_SIZE
        )

        self.right_buffer = deque(
            maxlen=FILTER_SIZE
        )

        self.left_raw = None
        self.right_raw = None

        self.last_left_time = 0.0
        self.last_right_time = 0.0

        # 새로운 측정값이 들어왔는지
        self.left_fresh = False
        self.right_fresh = False

        # =========================================================
        # Timeout
        # =========================================================

        self.UWB_TIMEOUT = 1.0

        # =========================================================
        # 방향 판정
        #
        # 중앙 실측:
        # 대략 ERR -3 ~ +1
        #
        # 좌/우:
        # 대체로 |ERR| >= 10
        # =========================================================

        # CENTER -> LEFT/RIGHT 진입 기준
        self.TURN_ENTER = 10.0

        # LEFT/RIGHT -> CENTER 복귀 기준
        self.CENTER_ENTER = 4.0

        # 새로운 UWB 판정이 몇 번 연속 같아야
        # 실제 방향을 변경할 것인지
        self.CONFIRM_COUNT = 3

        self.current_direction = "CENTER"

        self.candidate_direction = "CENTER"

        self.candidate_count = 0

        # =========================================================
        # 현재 direction error
        # =========================================================

        self.direction_error = 0.0

        # =========================================================
        # Timer
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info(
            "UWB tracker started | "
            "hysteresis + 3-sample confirmation ON"
        )

    # =============================================================
    # Parse
    # =============================================================

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

    # =============================================================
    # Serial
    # =============================================================

    def read_latest(self, ser):

        latest = None

        while ser.in_waiting > 0:

            line = (
                ser.readline()
                .decode(
                    "utf-8",
                    errors="ignore"
                )
                .strip()
            )

            value = self.parse_distance(
                line
            )

            if value is not None:
                latest = value

        return latest

    # =============================================================
    # Calibration
    # =============================================================

    def correct_left(self, raw):

        corrected = (
            LEFT_A * raw
            +
            LEFT_B
        )

        return max(
            0.0,
            corrected
        )

    def correct_right(self, raw):

        corrected = (
            RIGHT_A * raw
            +
            RIGHT_B
        )

        return max(
            0.0,
            corrected
        )

    # =============================================================
    # 현재 ERR에서 원하는 방향 후보 계산
    #
    # 중요한 부분:
    #
    # CENTER 상태에서는 ±10을 넘어야 좌우 진입
    #
    # LEFT/RIGHT 상태에서는 ±4 안으로 들어와야
    # CENTER 복귀
    #
    # 중간 구간은 현재 상태 유지
    # =============================================================

    def get_direction_candidate(
        self,
        error
    ):

        # =========================================================
        # 현재 CENTER
        # =========================================================

        if self.current_direction == "CENTER":

            if error > self.TURN_ENTER:
                return "RIGHT"

            if error < -self.TURN_ENTER:
                return "LEFT"

            return "CENTER"

        # =========================================================
        # 현재 RIGHT
        # =========================================================

        if self.current_direction == "RIGHT":

            # 반대쪽으로 확실하게 넘어간 경우
            if error < -self.TURN_ENTER:
                return "LEFT"

            # 중앙까지 충분히 돌아온 경우
            if abs(error) <= self.CENTER_ENTER:
                return "CENTER"

            # +4 ~ +10 같은 애매한 구간
            # RIGHT 유지
            return "RIGHT"

        # =========================================================
        # 현재 LEFT
        # =========================================================

        if self.current_direction == "LEFT":

            if error > self.TURN_ENTER:
                return "RIGHT"

            if abs(error) <= self.CENTER_ENTER:
                return "CENTER"

            return "LEFT"

        return "CENTER"

    # =============================================================
    # 연속 판정
    # =============================================================

    def update_direction_state(
        self,
        error
    ):

        candidate = (
            self.get_direction_candidate(
                error
            )
        )

        # ---------------------------------------------------------
        # 현재 방향과 동일
        # ---------------------------------------------------------

        if candidate == self.current_direction:

            self.candidate_direction = candidate

            self.candidate_count = 0

            return self.current_direction

        # ---------------------------------------------------------
        # 새로운 후보
        # ---------------------------------------------------------

        if candidate != self.candidate_direction:

            self.candidate_direction = candidate

            self.candidate_count = 1

        else:

            self.candidate_count += 1

        # ---------------------------------------------------------
        # 3회 연속 확인
        # ---------------------------------------------------------

        if self.candidate_count >= self.CONFIRM_COUNT:

            old_direction = self.current_direction

            self.current_direction = (
                self.candidate_direction
            )

            self.candidate_count = 0

            self.get_logger().info(
                f"DIRECTION CHANGE: "
                f"{old_direction} -> "
                f"{self.current_direction}"
            )

        return self.current_direction

    # =============================================================
    # Main update
    # =============================================================

    def update(self):

        now = time.monotonic()

        # =========================================================
        # LEFT
        # =========================================================

        value = self.read_latest(
            self.left_serial
        )

        if value is not None:

            self.left_buffer.append(
                value
            )

            self.left_raw = statistics.median(
                self.left_buffer
            )

            self.last_left_time = now

            self.left_fresh = True

        # =========================================================
        # RIGHT
        # =========================================================

        value = self.read_latest(
            self.right_serial
        )

        if value is not None:

            self.right_buffer.append(
                value
            )

            self.right_raw = statistics.median(
                self.right_buffer
            )

            self.last_right_time = now

            self.right_fresh = True

        # =========================================================
        # LOST
        # =========================================================

        if (
            self.left_raw is None
            or
            self.right_raw is None
            or
            now - self.last_left_time > self.UWB_TIMEOUT
            or
            now - self.last_right_time > self.UWB_TIMEOUT
        ):

            self.current_direction = "CENTER"

            self.candidate_direction = "CENTER"

            self.candidate_count = 0

            msg = String()

            msg.data = "LOST"

            self.pub_direction.publish(
                msg
            )

            return

        # =========================================================
        # 방향 판정은
        # 좌/우 양쪽에 새로운 측정값이 들어온 뒤에만 수행
        #
        # 20Hz timer가 같은 값을 3번 세는 문제 방지
        # =========================================================

        if (
            self.left_fresh
            and
            self.right_fresh
        ):

            self.direction_error = (
                self.left_raw
                -
                self.right_raw
            )

            self.update_direction_state(
                self.direction_error
            )

            self.left_fresh = False
            self.right_fresh = False

        # =========================================================
        # 거리 보정
        # =========================================================

        left_corrected = self.correct_left(
            self.left_raw
        )

        right_corrected = self.correct_right(
            self.right_raw
        )

        # =========================================================
        # User distance
        # =========================================================

        user_distance = (
            left_corrected
            +
            right_corrected
        ) / 2.0

        # =========================================================
        # Publish
        # =========================================================

        msg = Float32()

        msg.data = float(
            left_corrected
        )

        self.pub_left.publish(
            msg
        )

        # ---------------------------------------------------------

        msg = Float32()

        msg.data = float(
            right_corrected
        )

        self.pub_right.publish(
            msg
        )

        # ---------------------------------------------------------

        msg = Float32()

        msg.data = float(
            self.direction_error
        )

        self.pub_error.publish(
            msg
        )

        # ---------------------------------------------------------

        msg = Float32()

        msg.data = float(
            user_distance
        )

        self.pub_distance.publish(
            msg
        )

        # ---------------------------------------------------------

        msg2 = String()

        msg2.data = (
            self.current_direction
        )

        self.pub_direction.publish(
            msg2
        )

        # =========================================================
        # Debug
        # =========================================================

        self.get_logger().info(
            f"L={left_corrected:.1f} "
            f"R={right_corrected:.1f} "
            f"D={user_distance:.1f} "
            f"ERR={self.direction_error:.1f} "
            f"DIR={self.current_direction} "
            f"CAND={self.candidate_direction} "
            f"COUNT={self.candidate_count}"
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = UWBTracker()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        try:
            node.left_serial.close()
            node.right_serial.close()

        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()