#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


class MPU6050Node(Node):
    # =============================================================
    # MPU6050 registers
    # =============================================================
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    WHO_AM_I = 0x75

    def __init__(self):
        super().__init__('mpu6050_node')

        # =========================================================
        # I2C / sensor
        # =========================================================
        self.I2C_BUS = 1
        self.ADDRESS = 0x68

        # ±250 deg/s
        self.GYRO_SCALE = 131.0

        # ±2 g
        self.ACCEL_SCALE = 16384.0

        self.GRAVITY = 9.80665

        # =========================================================
        # Output/filter
        # =========================================================
        self.PUBLISH_HZ = 50.0

        # 새 값 비율. 작을수록 부드러움.
        self.LPF_ALPHA = 0.20

        # rad/s
        self.GYRO_DEADBAND = 0.010

        # =========================================================
        # Startup calibration
        # =========================================================
        self.CALIBRATION_SAMPLES = 300
        self.CALIBRATION_SAMPLE_DELAY = 0.01

        # =========================================================
        # I2C fault tolerance
        # =========================================================
        # 한 샘플 안에서 추가 재시도
        self.READ_RETRY_COUNT = 2

        # 연속 실패가 이 횟수 이상이면 bus reopen + MPU 재초기화
        self.RECOVER_AFTER_FAILURES = 3

        # 너무 자주 reopen하지 않도록
        self.RECOVERY_COOLDOWN = 0.50

        self.bus = None

        self.consecutive_errors = 0
        self.total_errors = 0
        self.last_recovery_time = -999.0

        # =========================================================
        # Bias / filter
        # =========================================================
        self.bias_x_dps = 0.0
        self.bias_y_dps = 0.0
        self.bias_z_dps = 0.0

        self.filtered_gx = 0.0
        self.filtered_gy = 0.0
        self.filtered_gz = 0.0

        self.filter_initialized = False

        # =========================================================
        # Publisher
        # =========================================================
        self.pub = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )

        # =========================================================
        # Open + initialize
        # =========================================================
        self.open_bus()
        self.initialize_mpu()

        self.get_logger().info(
            f'MPU6050 connected at 0x{self.ADDRESS:02X}'
        )

        # =========================================================
        # Gyro calibration
        # =========================================================
        self.calibrate_gyro()

        self.get_logger().info(
            f'LPF alpha={self.LPF_ALPHA:.2f}, '
            f'deadband={self.GYRO_DEADBAND:.3f} rad/s'
        )

        self.get_logger().info(
            'I2C fault recovery ON | '
            'read error does NOT kill imu_node'
        )

        # =========================================================
        # Timer
        # =========================================================
        self.timer = self.create_timer(
            1.0 / self.PUBLISH_HZ,
            self.timer_callback
        )

    # =============================================================
    # I2C
    # =============================================================
    def open_bus(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass

        self.bus = SMBus(self.I2C_BUS)

    def initialize_mpu(self):
        # Wake up
        self.bus.write_byte_data(
            self.ADDRESS,
            self.PWR_MGMT_1,
            0x00
        )

        time.sleep(0.05)

        # DLPF = 3
        # gyro bandwidth 약 44 Hz
        self.bus.write_byte_data(
            self.ADDRESS,
            self.CONFIG,
            0x03
        )

        # 1 kHz / (1 + 19) = 50 Hz
        self.bus.write_byte_data(
            self.ADDRESS,
            self.SMPLRT_DIV,
            19
        )

        # gyro ±250 deg/s
        self.bus.write_byte_data(
            self.ADDRESS,
            self.GYRO_CONFIG,
            0x00
        )

        # accel ±2 g
        self.bus.write_byte_data(
            self.ADDRESS,
            self.ACCEL_CONFIG,
            0x00
        )

        who = self.bus.read_byte_data(
            self.ADDRESS,
            self.WHO_AM_I
        )

        # MPU6050 WHO_AM_I 보통 0x68
        if who not in (0x68, 0x69):
            self.get_logger().warn(
                f'Unexpected WHO_AM_I=0x{who:02X}'
            )

    @staticmethod
    def signed16(high, low):
        value = (high << 8) | low

        if value & 0x8000:
            value -= 65536

        return value

    def read_raw_sample_once(self):
        # 0x3B부터 14 byte를 한 번에 읽는다.
        # AX AY AZ TEMP GX GY GZ
        #
        # 예전처럼 각 byte를 여러 번 read_byte_data() 하는 것보다
        # I2C transaction 수가 적어서 노이즈/Errno121에 조금 더 유리하다.
        data = self.bus.read_i2c_block_data(
            self.ADDRESS,
            self.ACCEL_XOUT_H,
            14
        )

        ax = self.signed16(data[0], data[1])
        ay = self.signed16(data[2], data[3])
        az = self.signed16(data[4], data[5])

        gx = self.signed16(data[8], data[9])
        gy = self.signed16(data[10], data[11])
        gz = self.signed16(data[12], data[13])

        return ax, ay, az, gx, gy, gz

    def read_raw_sample(self):
        last_exception = None

        for attempt in range(self.READ_RETRY_COUNT):
            try:
                sample = self.read_raw_sample_once()

                # 성공
                if self.consecutive_errors > 0:
                    self.get_logger().info(
                        f'IMU I2C recovered after '
                        f'{self.consecutive_errors} failed read(s)'
                    )

                self.consecutive_errors = 0
                return sample

            except OSError as exc:
                last_exception = exc

                # 한 callback 안에서 짧게 한 번 더 시도
                if attempt + 1 < self.READ_RETRY_COUNT:
                    time.sleep(0.002)

        self.consecutive_errors += 1
        self.total_errors += 1

        # 매 샘플마다 로그를 쏟지 않도록 제한
        if (
            self.consecutive_errors == 1
            or
            self.consecutive_errors
            % self.RECOVER_AFTER_FAILURES
            == 0
        ):
            self.get_logger().warn(
                'MPU6050 I2C read failed | '
                f'consecutive={self.consecutive_errors} | '
                f'total={self.total_errors} | '
                f'{last_exception}'
            )

        return None

    def recover_i2c(self):
        now = time.monotonic()

        if (
            now
            -
            self.last_recovery_time
            <
            self.RECOVERY_COOLDOWN
        ):
            return False

        self.last_recovery_time = now

        self.get_logger().warn(
            'Attempting MPU6050 I2C recovery...'
        )

        try:
            self.open_bus()
            time.sleep(0.02)
            self.initialize_mpu()

            # 중요:
            # 주행 중일 수 있으므로 여기서는 gyro bias를 다시
            # calibration하지 않는다.
            self.consecutive_errors = 0

            self.get_logger().info(
                'MPU6050 I2C recovery SUCCESS'
            )

            return True

        except OSError as exc:
            self.get_logger().warn(
                f'MPU6050 I2C recovery failed: {exc}'
            )
            return False

        except Exception as exc:
            self.get_logger().warn(
                f'MPU6050 recovery exception: {exc}'
            )
            return False

    # =============================================================
    # Startup gyro calibration
    # =============================================================
    def calibrate_gyro(self):
        self.get_logger().info(
            'Keep robot still: calibrating gyro...'
        )

        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0

        valid = 0

        # I2C가 순간적으로 튀어도 calibration 전체가 죽지 않도록
        # 최대 4배 횟수까지 시도
        max_attempts = (
            self.CALIBRATION_SAMPLES
            *
            4
        )

        attempts = 0

        while (
            valid
            <
            self.CALIBRATION_SAMPLES
            and
            attempts
            <
            max_attempts
        ):
            attempts += 1

            sample = self.read_raw_sample()

            if sample is None:
                if (
                    self.consecutive_errors
                    >=
                    self.RECOVER_AFTER_FAILURES
                ):
                    self.recover_i2c()

                time.sleep(
                    self.CALIBRATION_SAMPLE_DELAY
                )
                continue

            _, _, _, gx, gy, gz = sample

            sum_x += gx / self.GYRO_SCALE
            sum_y += gy / self.GYRO_SCALE
            sum_z += gz / self.GYRO_SCALE

            valid += 1

            time.sleep(
                self.CALIBRATION_SAMPLE_DELAY
            )

        if valid < 50:
            # 여기서 프로세스를 죽이지 않는다.
            # bias=0으로 시작하고 센서 데이터는 계속 publish 가능.
            self.get_logger().error(
                'Gyro calibration FAILED: '
                f'only {valid} valid samples. '
                'Starting with zero bias.'
            )

            self.bias_x_dps = 0.0
            self.bias_y_dps = 0.0
            self.bias_z_dps = 0.0

            return

        self.bias_x_dps = sum_x / valid
        self.bias_y_dps = sum_y / valid
        self.bias_z_dps = sum_z / valid

        self.get_logger().info(
            'Gyro bias: '
            f'X={self.bias_x_dps:.3f}, '
            f'Y={self.bias_y_dps:.3f}, '
            f'Z={self.bias_z_dps:.3f} deg/s | '
            f'samples={valid}'
        )

    # =============================================================
    # Filter
    # =============================================================
    def low_pass_gyro(
        self,
        gx,
        gy,
        gz
    ):
        if not self.filter_initialized:
            self.filtered_gx = gx
            self.filtered_gy = gy
            self.filtered_gz = gz
            self.filter_initialized = True

        else:
            a = self.LPF_ALPHA

            self.filtered_gx = (
                a
                *
                gx
                +
                (
                    1.0
                    -
                    a
                )
                *
                self.filtered_gx
            )

            self.filtered_gy = (
                a
                *
                gy
                +
                (
                    1.0
                    -
                    a
                )
                *
                self.filtered_gy
            )

            self.filtered_gz = (
                a
                *
                gz
                +
                (
                    1.0
                    -
                    a
                )
                *
                self.filtered_gz
            )

        # 작은 gyro noise는 0
        if abs(self.filtered_gx) < self.GYRO_DEADBAND:
            self.filtered_gx = 0.0

        if abs(self.filtered_gy) < self.GYRO_DEADBAND:
            self.filtered_gy = 0.0

        if abs(self.filtered_gz) < self.GYRO_DEADBAND:
            self.filtered_gz = 0.0

        return (
            self.filtered_gx,
            self.filtered_gy,
            self.filtered_gz
        )

    # =============================================================
    # Main timer
    # =============================================================
    def timer_callback(self):
        sample = self.read_raw_sample()

        if sample is None:
            # 실패한 샘플은 그냥 버린다.
            # 절대 exception으로 node를 죽이지 않는다.
            if (
                self.consecutive_errors
                >=
                self.RECOVER_AFTER_FAILURES
            ):
                self.recover_i2c()

            return

        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = sample

        # ---------------------------------------------------------
        # Acceleration: m/s^2
        # ---------------------------------------------------------
        ax = (
            ax_raw
            /
            self.ACCEL_SCALE
            *
            self.GRAVITY
        )

        ay = (
            ay_raw
            /
            self.ACCEL_SCALE
            *
            self.GRAVITY
        )

        az = (
            az_raw
            /
            self.ACCEL_SCALE
            *
            self.GRAVITY
        )

        # ---------------------------------------------------------
        # Gyro: raw -> deg/s -> bias removal -> rad/s
        # ---------------------------------------------------------
        gx_dps = (
            gx_raw
            /
            self.GYRO_SCALE
            -
            self.bias_x_dps
        )

        gy_dps = (
            gy_raw
            /
            self.GYRO_SCALE
            -
            self.bias_y_dps
        )

        gz_dps = (
            gz_raw
            /
            self.GYRO_SCALE
            -
            self.bias_z_dps
        )

        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        gx, gy, gz = self.low_pass_gyro(
            gx,
            gy,
            gz
        )

        # ---------------------------------------------------------
        # ROS Imu
        # ---------------------------------------------------------
        msg = Imu()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'imu_link'

        # orientation은 계산하지 않는다.
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        # 너무 과도한 의미는 두지 않고 대략적인 covariance만 둔다.
        msg.angular_velocity_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.0004
        ]

        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        self.pub.publish(msg)

    # =============================================================
    # Shutdown
    # =============================================================
    def destroy_node(self):
        try:
            if self.bus is not None:
                self.bus.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = MPU6050Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
