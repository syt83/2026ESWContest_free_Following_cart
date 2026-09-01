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
    PWR_MGMT_1 = 107
    SMPLRT_DIV = 25
    CONFIG = 26
    GYRO_CONFIG = 27
    ACCEL_CONFIG = 28
    ACCEL_XOUT_H = 59
    WHO_AM_I = 117
	
    def __init__(self):
        super().__init__('mpu6050_node')
        self.I2C_BUS = 1
        self.ADDRESS = 104
        self.GYRO_SCALE = 131.0
        self.ACCEL_SCALE = 16384.0
        self.GRAVITY = 9.80665
        self.PUBLISH_HZ = 50.0
        self.LPF_ALPHA = 0.2
        self.GYRO_DEADBAND = 0.01
        self.CALIBRATION_SAMPLES = 300
        self.CALIBRATION_SAMPLE_DELAY = 0.01
        self.READ_RETRY_COUNT = 2
        self.RECOVER_AFTER_FAILURES = 3
        self.RECOVERY_COOLDOWN = 0.5
        self.bus = None
        self.consecutive_errors = 0
        self.total_errors = 0
        self.last_recovery_time = -999.0
        self.bias_x_dps = 0.0
        self.bias_y_dps = 0.0
        self.bias_z_dps = 0.0
        self.filtered_gx = 0.0
        self.filtered_gy = 0.0
        self.filtered_gz = 0.0
        self.filter_initialized = False
        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.open_bus()
        self.initialize_mpu()
        self.get_logger().info(f'MPU6050 connected at 0x{self.ADDRESS:02X}')
        self.calibrate_gyro()
        self.get_logger().info(f'LPF alpha={self.LPF_ALPHA:.2f}, deadband={self.GYRO_DEADBAND:.3f} rad/s')
        self.get_logger().info('I2C fault recovery ON | read error does NOT kill imu_node')
        self.timer = self.create_timer(1.0 / self.PUBLISH_HZ, self.timer_callback)

    def open_bus(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = SMBus(self.I2C_BUS)

    def initialize_mpu(self):
        self.bus.write_byte_data(self.ADDRESS, self.PWR_MGMT_1, 0)
        time.sleep(0.05)
        self.bus.write_byte_data(self.ADDRESS, self.CONFIG, 3)
        self.bus.write_byte_data(self.ADDRESS, self.SMPLRT_DIV, 19)
        self.bus.write_byte_data(self.ADDRESS, self.GYRO_CONFIG, 0)
        self.bus.write_byte_data(self.ADDRESS, self.ACCEL_CONFIG, 0)
        who = self.bus.read_byte_data(self.ADDRESS, self.WHO_AM_I)
        if who not in (104, 105):
            self.get_logger().warn(f'Unexpected WHO_AM_I=0x{who:02X}')

    @staticmethod
    def signed16(high, low):
        value = high << 8 | low
        if value & 32768:
            value -= 65536
        return value

    def read_raw_sample_once(self):
        data = self.bus.read_i2c_block_data(self.ADDRESS, self.ACCEL_XOUT_H, 14)
        ax = self.signed16(data[0], data[1])
        ay = self.signed16(data[2], data[3])
        az = self.signed16(data[4], data[5])
        gx = self.signed16(data[8], data[9])
        gy = self.signed16(data[10], data[11])
        gz = self.signed16(data[12], data[13])
        return (ax, ay, az, gx, gy, gz)

    def read_raw_sample(self):
        last_exception = None
        for attempt in range(self.READ_RETRY_COUNT):
            try:
                sample = self.read_raw_sample_once()
                if self.consecutive_errors > 0:
                    self.get_logger().info(f'IMU I2C recovered after {self.consecutive_errors} failed read(s)')
                self.consecutive_errors = 0
                return sample
            except OSError as exc:
                last_exception = exc
                if attempt + 1 < self.READ_RETRY_COUNT:
                    time.sleep(0.002)
        self.consecutive_errors += 1
        self.total_errors += 1
        if self.consecutive_errors == 1 or self.consecutive_errors % self.RECOVER_AFTER_FAILURES == 0:
            self.get_logger().warn(f'MPU6050 I2C read failed | consecutive={self.consecutive_errors} | total={self.total_errors} | {last_exception}')
        return None

    def recover_i2c(self):
        now = time.monotonic()
        if now - self.last_recovery_time < self.RECOVERY_COOLDOWN:
            return False
        self.last_recovery_time = now
        self.get_logger().warn('Attempting MPU6050 I2C recovery...')
        try:
            self.open_bus()
            time.sleep(0.02)
            self.initialize_mpu()
            self.consecutive_errors = 0
            self.get_logger().info('MPU6050 I2C recovery SUCCESS')
            return True
        except OSError as exc:
            self.get_logger().warn(f'MPU6050 I2C recovery failed: {exc}')
            return False
        except Exception as exc:
            self.get_logger().warn(f'MPU6050 recovery exception: {exc}')
            return False

    def calibrate_gyro(self):
        self.get_logger().info('Keep robot still: calibrating gyro...')
        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0
        valid = 0
        max_attempts = self.CALIBRATION_SAMPLES * 4
        attempts = 0
        while valid < self.CALIBRATION_SAMPLES and attempts < max_attempts:
            attempts += 1
            sample = self.read_raw_sample()
            if sample is None:
                if self.consecutive_errors >= self.RECOVER_AFTER_FAILURES:
                    self.recover_i2c()
                time.sleep(self.CALIBRATION_SAMPLE_DELAY)
                continue
            _, _, _, gx, gy, gz = sample
            sum_x += gx / self.GYRO_SCALE
            sum_y += gy / self.GYRO_SCALE
            sum_z += gz / self.GYRO_SCALE
            valid += 1
            time.sleep(self.CALIBRATION_SAMPLE_DELAY)
        if valid < 50:
            self.get_logger().error(f'Gyro calibration FAILED: only {valid} valid samples. Starting with zero bias.')
            self.bias_x_dps = 0.0
            self.bias_y_dps = 0.0
            self.bias_z_dps = 0.0
            return
        self.bias_x_dps = sum_x / valid
        self.bias_y_dps = sum_y / valid
        self.bias_z_dps = sum_z / valid
        self.get_logger().info(f'Gyro bias: X={self.bias_x_dps:.3f}, Y={self.bias_y_dps:.3f}, Z={self.bias_z_dps:.3f} deg/s | samples={valid}')

    def low_pass_gyro(self, gx, gy, gz):
        if not self.filter_initialized:
            self.filtered_gx = gx
            self.filtered_gy = gy
            self.filtered_gz = gz
            self.filter_initialized = True
        else:
            a = self.LPF_ALPHA
            self.filtered_gx = a * gx + (1.0 - a) * self.filtered_gx
            self.filtered_gy = a * gy + (1.0 - a) * self.filtered_gy
            self.filtered_gz = a * gz + (1.0 - a) * self.filtered_gz
        if abs(self.filtered_gx) < self.GYRO_DEADBAND:
            self.filtered_gx = 0.0
        if abs(self.filtered_gy) < self.GYRO_DEADBAND:
            self.filtered_gy = 0.0
        if abs(self.filtered_gz) < self.GYRO_DEADBAND:
            self.filtered_gz = 0.0
        return (self.filtered_gx, self.filtered_gy, self.filtered_gz)

    def timer_callback(self):
        sample = self.read_raw_sample()
        if sample is None:
            if self.consecutive_errors >= self.RECOVER_AFTER_FAILURES:
                self.recover_i2c()
            return
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = sample
        ax = ax_raw / self.ACCEL_SCALE * self.GRAVITY
        ay = ay_raw / self.ACCEL_SCALE * self.GRAVITY
        az = az_raw / self.ACCEL_SCALE * self.GRAVITY
        gx_dps = gx_raw / self.GYRO_SCALE - self.bias_x_dps
        gy_dps = gy_raw / self.GYRO_SCALE - self.bias_y_dps
        gz_dps = gz_raw / self.GYRO_SCALE - self.bias_z_dps
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)
        gx, gy, gz = self.low_pass_gyro(gx, gy, gz)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self.pub.publish(msg)

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
