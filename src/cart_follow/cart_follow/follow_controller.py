import time
import statistics
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Bool, Int32MultiArray

class FollowController(Node):

    def __init__(self):
        super().__init__('follow_controller')
        self.STOP_DISTANCE = 130.0
        self.RESTART_DISTANCE = 145.0
        self.SLOW_DISTANCE = 170.0
        self.NORMAL_DISTANCE = 250.0
        self.MAX_TRACK_DISTANCE = 600.0
        self.SLOW_SPEED = 50.0
        self.NORMAL_SPEED = 80.0
        self.MAX_SPEED = 120.0
        self.K_UWB = 0.35
        self.MAX_UWB_TURN = 22.0
        self.UWB_SIGN = 1.0
        self.UWB_MEDIAN_SIZE = 5
        self.uwb_error_buffer = deque(maxlen=self.UWB_MEDIAN_SIZE)
        self.UWB_ALPHA = 0.4
        self.UWB_DEADBAND = 5.0
        self.UWB_MAX_ERROR_STEP = 8.0
        self.ema_direction_error = 0.0
        self.filtered_direction_error = 0.0
        self.STARTUP_STABLE_TIME = 1.0
        self.STARTUP_DIRECTION_COUNT = 3
        self.startup_begin_time = time.monotonic()
        self.startup_direction = None
        self.startup_direction_count = 0
        self.startup_ready = False
        self.startup_ready_logged = False
        self.K_IMU = 8.0
        self.MAX_IMU_TURN = 6.0
        self.IMU_YAW_DEADBAND = 0.015
        self.IMU_TIMEOUT = 0.3
        self.yaw_rate = 0.0
        self.last_imu = 0.0
        self.AVOID_GAIN = 1.0
        self.MAX_LIDAR_TURN = 45.0
        self.MAX_TOTAL_TURN = 55.0
        self.UWB_SCALE_TRACK = 1.0
        self.UWB_SCALE_AVOID = 0.2
        self.UWB_SCALE_PASSING = 0.15
        self.UWB_SCALE_RECOVER_START = 0.15
        self.UWB_RECOVER_TIME = 1.0
        self.EMERGENCY_STOP_TIME = 1.0
        self.REVERSE_SPEED = -45.0
        self.REVERSE_TIME = 2.0
        self.POST_REVERSE_UWB_TIME = 1.5
        self.POST_REVERSE_MAX_SPEED = 35.0
        self.post_reverse_uwb_until = 0.0
        self.post_reverse_recovery_active = False
        self.REAR_SAFE_DISTANCE = 0.8
        self.REAR_STOP_DISTANCE = 0.3
        self.EMERGENCY_CLEAR_TIME = 0.3
        self.UWB_TIMEOUT = 0.7
        self.LIDAR_TIMEOUT = 0.5
        self.raw_direction_error = 0.0
        self.user_distance = 9999.0
        self.direction = 'LOST'
        self.last_uwb = 0.0
        self.lidar_steering = 0.0
        self.lidar_threat = 0.0
        self.emergency = False
        self.front_clearance = 0.0
        self.rear_clearance = 0.0
        self.last_lidar = 0.0
        self.avoidance_state = 'TRACK'
        self.previous_avoidance_state = 'TRACK'
        self.recover_start_time = 0.0
        self.avoidance_disabled = False
        self.stopped_for_user = False
        self.emergency_state = 'TRACK'
        self.emergency_start_time = 0.0
        self.reverse_start_time = 0.0
        self.emergency_armed = True
        self.emergency_clear_start = None
        self.create_subscription(Float32, '/uwb/direction_error', self.cb_direction_error, 10)
        self.create_subscription(Float32, '/uwb/user_distance', self.cb_user_distance, 10)
        self.create_subscription(String, '/uwb/direction', self.cb_direction, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, 10)
        self.create_subscription(Float32, '/avoidance/steering', self.cb_lidar_steering, 10)
        self.create_subscription(Float32, '/avoidance/threat', self.cb_lidar_threat, 10)
        self.create_subscription(Bool, '/emergency_stop', self.cb_emergency, 10)
        self.create_subscription(Float32, '/avoidance/front_clearance', self.cb_front_clearance, 10)
        self.create_subscription(Float32, '/avoidance/rear_clearance', self.cb_rear_clearance, 10)
        self.create_subscription(String, '/avoidance/state', self.cb_avoidance_state, 10)
        self.motor_pub = self.create_publisher(Int32MultiArray, '/motor_cmd_pwm', 10)
        self.filtered_error_pub = self.create_publisher(Float32, '/uwb/filtered_direction_error', 10)
        self.imu_turn_pub = self.create_publisher(Float32, '/control/imu_turn', 10)
        self.total_turn_pub = self.create_publisher(Float32, '/control/total_turn', 10)
        self.avoidance_disable_pub = self.create_publisher(Bool, '/avoidance/disable', 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Follow controller started | OLD BASELINE + MIN PATCH | IMU FAIL-SOFT | K_IMU=8 MAX=6 | STOP=130cm | POST REVERSE UWB=1.5s')

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def set_avoidance_disabled(self, disabled):
        disabled = bool(disabled)
        if disabled == self.avoidance_disabled:
            return
        self.avoidance_disabled = disabled
        msg = Bool()
        msg.data = disabled
        self.avoidance_disable_pub.publish(msg)
        if disabled:
            self.get_logger().warn('NORMAL LIDAR AVOIDANCE -> DISABLED')
        else:
            self.get_logger().info('NORMAL LIDAR AVOIDANCE -> ENABLED')

    def clear_old_steering(self):
        self.uwb_error_buffer.clear()
        self.ema_direction_error = 0.0
        self.filtered_direction_error = 0.0
        self.lidar_steering = 0.0
        self.lidar_threat = 0.0
        self.avoidance_state = 'TRACK'
        self.previous_avoidance_state = 'TRACK'
        self.recover_start_time = 0.0
        self.yaw_rate = 0.0

    def cb_direction_error(self, msg):
        raw = float(msg.data)
        self.raw_direction_error = raw
        self.uwb_error_buffer.append(raw)
        median_error = statistics.median(self.uwb_error_buffer)
        self.ema_direction_error = self.UWB_ALPHA * median_error + (1.0 - self.UWB_ALPHA) * self.ema_direction_error
        if abs(self.ema_direction_error) < self.UWB_DEADBAND:
            target_error = 0.0
        else:
            target_error = self.ema_direction_error
        delta = target_error - self.filtered_direction_error
        delta = self.clamp(delta, -self.UWB_MAX_ERROR_STEP, self.UWB_MAX_ERROR_STEP)
        self.filtered_direction_error += delta
        self.last_uwb = time.monotonic()
        out = Float32()
        out.data = float(self.filtered_direction_error)
        self.filtered_error_pub.publish(out)

    def cb_user_distance(self, msg):
        self.user_distance = float(msg.data)
        self.last_uwb = time.monotonic()

    def cb_direction(self, msg):
        new_direction = str(msg.data)
        if new_direction in ('LEFT', 'CENTER', 'RIGHT'):
            if new_direction == self.startup_direction:
                self.startup_direction_count += 1
            else:
                self.startup_direction = new_direction
                self.startup_direction_count = 1
        else:
            self.startup_direction = None
            self.startup_direction_count = 0
        if new_direction == 'CENTER' and self.direction != 'CENTER':
            self.ema_direction_error = 0.0
            self.filtered_direction_error = 0.0
            self.uwb_error_buffer.clear()
        self.direction = new_direction
        self.last_uwb = time.monotonic()

    def cb_imu(self, msg):
        self.yaw_rate = float(msg.angular_velocity.z)
        self.last_imu = time.monotonic()

    def calculate_imu_turn(self, now):
        if now - self.last_imu > self.IMU_TIMEOUT:
            return 0.0
        yaw = self.yaw_rate
        if abs(yaw) < self.IMU_YAW_DEADBAND:
            return 0.0
        imu_turn = self.K_IMU * yaw
        return self.clamp(imu_turn, -self.MAX_IMU_TURN, self.MAX_IMU_TURN)

    def cb_lidar_steering(self, msg):
        self.lidar_steering = float(msg.data)
        self.last_lidar = time.monotonic()

    def cb_lidar_threat(self, msg):
        self.lidar_threat = float(msg.data)
        self.last_lidar = time.monotonic()

    def cb_emergency(self, msg):
        self.emergency = bool(msg.data)
        self.last_lidar = time.monotonic()

    def cb_front_clearance(self, msg):
        self.front_clearance = float(msg.data)
        self.last_lidar = time.monotonic()

    def cb_rear_clearance(self, msg):
        self.rear_clearance = float(msg.data)
        self.last_lidar = time.monotonic()

    def cb_avoidance_state(self, msg):
        new_state = str(msg.data)
        if new_state != self.avoidance_state:
            self.previous_avoidance_state = self.avoidance_state
            self.avoidance_state = new_state
            if new_state == 'RECOVER':
                self.recover_start_time = time.monotonic()
        self.last_lidar = time.monotonic()

    def publish_motor(self, left, right):
        msg = Int32MultiArray()
        msg.data = [int(self.clamp(left, -255, 255)), int(self.clamp(right, -255, 255))]
        self.motor_pub.publish(msg)

    def calculate_speed(self):
        d = self.user_distance
        if d <= self.STOP_DISTANCE:
            return 0.0
        if d <= self.SLOW_DISTANCE:
            ratio = (d - self.STOP_DISTANCE) / (self.SLOW_DISTANCE - self.STOP_DISTANCE)
            ratio = self.clamp(ratio, 0.0, 1.0)
            return self.SLOW_SPEED * ratio
        if d <= self.NORMAL_DISTANCE:
            ratio = (d - self.SLOW_DISTANCE) / (self.NORMAL_DISTANCE - self.SLOW_DISTANCE)
            ratio = self.clamp(ratio, 0.0, 1.0)
            return self.SLOW_SPEED + (self.NORMAL_SPEED - self.SLOW_SPEED) * ratio
        return self.MAX_SPEED

    def calculate_uwb_scale(self, now):
        if self.post_reverse_recovery_active:
            return 1.0
        if self.avoidance_state == 'AVOID':
            return self.UWB_SCALE_AVOID
        if self.avoidance_state == 'PASSING':
            return self.UWB_SCALE_PASSING
        if self.avoidance_state == 'RECOVER':
            elapsed = now - self.recover_start_time
            ratio = self.clamp(elapsed / self.UWB_RECOVER_TIME, 0.0, 1.0)
            return self.UWB_SCALE_RECOVER_START + (1.0 - self.UWB_SCALE_RECOVER_START) * ratio
        return self.UWB_SCALE_TRACK

    def calculate_uwb_turn(self, now):
        if self.direction == 'LOST':
            return 0.0
        if self.direction == 'CENTER':
            return 0.0
        error = self.filtered_direction_error
        if self.direction == 'RIGHT' and error <= 0.0:
            return 0.0
        if self.direction == 'LEFT' and error >= 0.0:
            return 0.0
        turn = self.UWB_SIGN * self.K_UWB * error
        turn = self.clamp(turn, -self.MAX_UWB_TURN, self.MAX_UWB_TURN)
        scale = self.calculate_uwb_scale(now)
        return turn * scale

    def startup_check(self, now):
        if self.startup_ready:
            return True
        elapsed = now - self.startup_begin_time
        direction_ok = self.startup_direction_count >= self.STARTUP_DIRECTION_COUNT
        uwb_ok = self.direction != 'LOST' and now - self.last_uwb <= self.UWB_TIMEOUT
        imu_ok = now - self.last_imu <= self.IMU_TIMEOUT
        time_ok = elapsed >= self.STARTUP_STABLE_TIME
        if direction_ok and uwb_ok and imu_ok and time_ok:
            self.startup_ready = True
            self.uwb_error_buffer.clear()
            self.ema_direction_error = 0.0
            self.filtered_direction_error = 0.0
            if not self.startup_ready_logged:
                self.get_logger().info(f'STARTUP STABLE | DIR={self.direction} | FOLLOW ENABLED')
                self.startup_ready_logged = True
            return True
        return False

    def start_emergency(self, now):
        self.emergency_armed = False
        self.emergency_clear_start = None
        self.emergency_state = 'STOP'
        self.emergency_start_time = now
        self.stopped_for_user = False
        self.post_reverse_recovery_active = False
        self.post_reverse_uwb_until = 0.0
        self.set_avoidance_disabled(True)
        self.clear_old_steering()
        self.publish_motor(0, 0)
        self.get_logger().warn('EMERGENCY | STOP')

    def run_emergency_stop(self, now):
        elapsed = now - self.emergency_start_time
        if elapsed < self.EMERGENCY_STOP_TIME:
            self.publish_motor(0, 0)
            return
        if self.user_distance < self.RESTART_DISTANCE:
            self.stopped_for_user = True
            self.emergency_state = 'TRACK'
            self.emergency_armed = True
            self.emergency_clear_start = None
            self.set_avoidance_disabled(False)
            self.clear_old_steering()
            self.publish_motor(0, 0)
            self.get_logger().warn('EMERGENCY REVERSE CANCELLED | USER STOP PRIORITY')
            return
        if self.rear_clearance >= self.REAR_SAFE_DISTANCE:
            self.emergency_state = 'REVERSE'
            self.reverse_start_time = now
            self.publish_motor(self.REVERSE_SPEED, self.REVERSE_SPEED)
            self.get_logger().warn('EMERGENCY | REVERSE')
        else:
            self.emergency_state = 'WAIT_REAR'
            self.publish_motor(0, 0)
            self.get_logger().warn('EMERGENCY | WAIT_REAR')

    def run_wait_rear(self, now):
        self.publish_motor(0, 0)
        if self.rear_clearance >= self.REAR_SAFE_DISTANCE:
            self.emergency_state = 'REVERSE'
            self.reverse_start_time = now
            self.get_logger().warn('EMERGENCY | REVERSE')

    def finish_reverse(self):
        self.publish_motor(0, 0)
        self.emergency_state = 'WAIT_CLEAR'
        self.emergency_clear_start = None
        self.clear_old_steering()
        self.get_logger().warn('REVERSE FINISHED | waiting emergency clear')

    def run_reverse(self, now):
        if self.rear_clearance <= self.REAR_STOP_DISTANCE:
            self.finish_reverse()
            self.get_logger().warn('REVERSE STOPPED | rear obstacle')
            return
        elapsed = now - self.reverse_start_time
        if elapsed < self.REVERSE_TIME:
            self.publish_motor(self.REVERSE_SPEED, self.REVERSE_SPEED)
            return
        self.finish_reverse()

    def run_wait_clear(self, now):
        self.publish_motor(0, 0)
        if self.emergency:
            self.emergency_clear_start = None
            return
        if self.emergency_clear_start is None:
            self.emergency_clear_start = now
            return
        if now - self.emergency_clear_start < self.EMERGENCY_CLEAR_TIME:
            return
        self.clear_old_steering()
        self.emergency_armed = True
        self.emergency_state = 'TRACK'
        self.emergency_clear_start = None
        self.post_reverse_recovery_active = True
        self.post_reverse_uwb_until = now + self.POST_REVERSE_UWB_TIME
        self.set_avoidance_disabled(True)
        self.get_logger().warn('EMERGENCY CLEARED | POST-REVERSE UWB PRIORITY START')

    def update_post_reverse_recovery(self, now):
        if not self.post_reverse_recovery_active:
            return False
        if now < self.post_reverse_uwb_until:
            return True
        self.post_reverse_recovery_active = False
        self.post_reverse_uwb_until = 0.0
        self.lidar_steering = 0.0
        self.lidar_threat = 0.0
        self.set_avoidance_disabled(False)
        self.get_logger().info('POST-REVERSE UWB PRIORITY END | LiDAR avoidance enabled')
        return False

    def control_loop(self):
        now = time.monotonic()
        if self.emergency_state == 'STOP':
            self.run_emergency_stop(now)
            return
        if self.emergency_state == 'WAIT_REAR':
            self.run_wait_rear(now)
            return
        if self.emergency_state == 'REVERSE':
            self.run_reverse(now)
            return
        if self.emergency_state == 'WAIT_CLEAR':
            self.run_wait_clear(now)
            return
        if now - self.last_lidar > self.LIDAR_TIMEOUT:
            self.publish_motor(0, 0)
            return
        if now - self.last_uwb > self.UWB_TIMEOUT:
            self.publish_motor(0, 0)
            return
        if not self.startup_check(now):
            self.publish_motor(0, 0)
            return
        if self.direction == 'LOST':
            self.publish_motor(0, 0)
            return
        if self.user_distance > self.MAX_TRACK_DISTANCE:
            self.publish_motor(0, 0)
            return
        if self.stopped_for_user:
            if self.user_distance < self.RESTART_DISTANCE:
                self.publish_motor(0, 0)
                return
            self.stopped_for_user = False
        elif self.user_distance <= self.STOP_DISTANCE:
            self.stopped_for_user = True
            self.publish_motor(0, 0)
            return
        if self.emergency and self.emergency_armed:
            self.start_emergency(now)
            return
        post_reverse_active = self.update_post_reverse_recovery(now)
        base_speed = self.calculate_speed()
        if post_reverse_active:
            base_speed = min(base_speed, self.POST_REVERSE_MAX_SPEED)
        uwb_turn = self.calculate_uwb_turn(now)
        if post_reverse_active:
            lidar_turn = 0.0
        else:
            lidar_turn = self.AVOID_GAIN * self.lidar_steering
            lidar_turn = self.clamp(lidar_turn, -self.MAX_LIDAR_TURN, self.MAX_LIDAR_TURN)
        imu_turn = self.calculate_imu_turn(now)
        total_turn = uwb_turn + lidar_turn + imu_turn
        total_turn = self.clamp(total_turn, -self.MAX_TOTAL_TURN, self.MAX_TOTAL_TURN)
        msg = Float32()
        msg.data = float(imu_turn)
        self.imu_turn_pub.publish(msg)
        msg = Float32()
        msg.data = float(total_turn)
        self.total_turn_pub.publish(msg)
        if post_reverse_active:
            threat = 0.0
        else:
            threat = self.clamp(self.lidar_threat, 0.0, 1.0)
        speed_scale = 1.0 - 0.45 * threat
        speed_scale = self.clamp(speed_scale, 0.55, 1.0)
        base_speed *= speed_scale
        if not post_reverse_active and self.avoidance_state == 'PASSING':
            base_speed *= 0.82
        left = base_speed + total_turn
        right = base_speed - total_turn
        left = max(-20.0, left)
        right = max(-20.0, right)
        self.publish_motor(left, right)

def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_motor(0, 0)
                node.set_avoidance_disabled(False)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
