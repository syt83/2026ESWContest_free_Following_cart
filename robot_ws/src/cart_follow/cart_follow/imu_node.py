import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from smbus import SMBus


class MPU6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        # ==========================================
        # MPU6050 설정
        # ==========================================

        self.bus_num = 1
        self.addr = 0x68

        self.bus = SMBus(self.bus_num)

        # MPU6050 wake up
        self.bus.write_byte_data(
            self.addr,
            0x6B,
            0x00
        )

        time.sleep(0.1)

        # ==========================================
        # Gyro bias
        # ==========================================

        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        # ==========================================
        # Low Pass Filter
        #
        # filtered =
        # previous * (1-alpha)
        # + new * alpha
        #
        # alpha 작음 -> 더 부드러움, 반응 느림
        # alpha 큼 -> 반응 빠름, 노이즈 증가
        # ==========================================

        self.LPF_ALPHA = 0.20

        self.filtered_gx = 0.0
        self.filtered_gy = 0.0
        self.filtered_gz = 0.0

        # ==========================================
        # Deadband
        #
        # 이 값보다 작은 각속도는 0으로 처리
        #
        # 0.01 rad/s ≈ 0.573 deg/s
        # ==========================================

        self.GYRO_DEADBAND = 0.01

        self.get_logger().info(
            'MPU6050 connected at 0x68'
        )

        self.get_logger().info(
            'Keep robot still: calibrating gyro...'
        )

        self.calibrate_gyro()

        self.get_logger().info(
            f'Gyro bias: '
            f'X={self.gyro_bias_x:.3f}, '
            f'Y={self.gyro_bias_y:.3f}, '
            f'Z={self.gyro_bias_z:.3f} deg/s'
        )

        self.get_logger().info(
            f'LPF alpha={self.LPF_ALPHA:.2f}, '
            f'deadband={self.GYRO_DEADBAND:.3f} rad/s'
        )

        # ==========================================
        # ROS publisher
        # ==========================================

        self.publisher = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )

        # 100Hz
        self.timer = self.create_timer(
            0.01,
            self.timer_callback
        )

    # ==========================================
    # signed 16-bit read
    # ==========================================

    def read_word_2c(self, reg):

        high = self.bus.read_byte_data(
            self.addr,
            reg
        )

        low = self.bus.read_byte_data(
            self.addr,
            reg + 1
        )

        value = (
            high << 8
        ) | low

        if value >= 0x8000:
            value -= 65536

        return value

    # ==========================================
    # Gyro calibration
    # ==========================================

    def calibrate_gyro(self):

        samples = 500

        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0

        for _ in range(samples):

            gx_raw = self.read_word_2c(
                0x43
            )

            gy_raw = self.read_word_2c(
                0x45
            )

            gz_raw = self.read_word_2c(
                0x47
            )

            gx = (
                gx_raw / 131.0
            )

            gy = (
                gy_raw / 131.0
            )

            gz = (
                gz_raw / 131.0
            )

            sum_x += gx
            sum_y += gy
            sum_z += gz

            time.sleep(0.005)

        self.gyro_bias_x = (
            sum_x / samples
        )

        self.gyro_bias_y = (
            sum_y / samples
        )

        self.gyro_bias_z = (
            sum_z / samples
        )

    # ==========================================
    # Low-pass filter
    # ==========================================

    def low_pass_filter(
        self,
        previous,
        current
    ):

        alpha = self.LPF_ALPHA

        return (
            previous * (1.0 - alpha)
            +
            current * alpha
        )

    # ==========================================
    # Deadband
    # ==========================================

    def apply_deadband(
        self,
        value
    ):

        if (
            abs(value)
            <
            self.GYRO_DEADBAND
        ):
            return 0.0

        return value

    # ==========================================
    # Timer callback
    # ==========================================

    def timer_callback(self):

        # --------------------------------------
        # Accelerometer
        # --------------------------------------

        ax_raw = self.read_word_2c(
            0x3B
        )

        ay_raw = self.read_word_2c(
            0x3D
        )

        az_raw = self.read_word_2c(
            0x3F
        )

        ax_g = (
            ax_raw / 16384.0
        )

        ay_g = (
            ay_raw / 16384.0
        )

        az_g = (
            az_raw / 16384.0
        )

        # --------------------------------------
        # Gyroscope raw
        # --------------------------------------

        gx_raw = self.read_word_2c(
            0x43
        )

        gy_raw = self.read_word_2c(
            0x45
        )

        gz_raw = self.read_word_2c(
            0x47
        )

        # --------------------------------------
        # deg/s 변환 + bias 제거
        # --------------------------------------

        gx_dps = (
            gx_raw / 131.0
            -
            self.gyro_bias_x
        )

        gy_dps = (
            gy_raw / 131.0
            -
            self.gyro_bias_y
        )

        gz_dps = (
            gz_raw / 131.0
            -
            self.gyro_bias_z
        )

        # --------------------------------------
        # deg/s -> rad/s
        # --------------------------------------

        gx_rad = math.radians(
            gx_dps
        )

        gy_rad = math.radians(
            gy_dps
        )

        gz_rad = math.radians(
            gz_dps
        )

        # --------------------------------------
        # Low Pass Filter
        # --------------------------------------

        self.filtered_gx = (
            self.low_pass_filter(
                self.filtered_gx,
                gx_rad
            )
        )

        self.filtered_gy = (
            self.low_pass_filter(
                self.filtered_gy,
                gy_rad
            )
        )

        self.filtered_gz = (
            self.low_pass_filter(
                self.filtered_gz,
                gz_rad
            )
        )

        # --------------------------------------
        # Deadband
        # --------------------------------------

        output_gx = (
            self.apply_deadband(
                self.filtered_gx
            )
        )

        output_gy = (
            self.apply_deadband(
                self.filtered_gy
            )
        )

        output_gz = (
            self.apply_deadband(
                self.filtered_gz
            )
        )

        # ======================================
        # ROS Imu message
        # ======================================

        msg = Imu()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'imu_link'
        )

        # --------------------------------------
        # Acceleration
        #
        # g -> m/s²
        # --------------------------------------

        g = 9.80665

        msg.linear_acceleration.x = (
            ax_g * g
        )

        msg.linear_acceleration.y = (
            ay_g * g
        )

        msg.linear_acceleration.z = (
            az_g * g
        )

        # --------------------------------------
        # Angular velocity
        # --------------------------------------

        msg.angular_velocity.x = (
            output_gx
        )

        msg.angular_velocity.y = (
            output_gy
        )

        msg.angular_velocity.z = (
            output_gz
        )

        # MPU6050 단독으로는
        # orientation을 여기서 제공하지 않음
        msg.orientation_covariance[0] = -1.0

        # --------------------------------------
        # 간단한 covariance 설정
        # --------------------------------------

        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        msg.linear_acceleration_covariance[0] = 0.10
        msg.linear_acceleration_covariance[4] = 0.10
        msg.linear_acceleration_covariance[8] = 0.10

        self.publisher.publish(
            msg
        )

    # ==========================================
    # Shutdown
    # ==========================================

    def destroy_node(self):

        self.bus.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = MPU6050Node()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()