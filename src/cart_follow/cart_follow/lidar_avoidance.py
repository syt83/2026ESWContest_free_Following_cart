import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, String

class LidarAvoidance(Node):

    def __init__(self):
        super().__init__('lidar_avoidance')
        self.AVOID_START_DISTANCE = 1.2
        self.FULL_AVOID_DISTANCE = 0.45
        self.FRONT_HALF_ANGLE = 25.0
        self.MAX_STEERING = 30.0
        self.DIRECTION_SWITCH_MARGIN = 0.15
        self.EMERGENCY_DISTANCE = 0.3
        self.EMERGENCY_HALF_ANGLE = 25.0
        self.EMERGENCY_HOLD_TIME = 3.0
        self.last_emergency_detect_time = -999.0
        self.BOUNDARY_ARC_MIN = 25.0
        self.BOUNDARY_ARC_MAX = 155.0
        self.TARGET_SIDE_DISTANCE = 0.55
        self.BOUNDARY_DETECT_DISTANCE = 1.45
        self.BOUNDARY_CLUSTER_BAND = 0.25
        self.BOUNDARY_DISTANCE_KP = 30.0
        self.MAX_DISTANCE_TURN = 16.0
        self.BOUNDARY_ANGLE_TARGET = 90.0
        self.BOUNDARY_ANGLE_KP = 0.35
        self.MAX_ANGLE_TURN = 16.0
        self.MAX_BOUNDARY_STEERING = 26.0
        self.BOUNDARY_SEARCH_TURN = 10.0
        self.BOUNDARY_LOST_HOLD_TIME = 0.8
        self.boundary_lost_start = None
        self.ACQUIRE_SEARCH_TURN = 10.0
        self.ACQUIRE_TIMEOUT = 1.2
        self.acquire_start_time = None
        self.BOUNDARY_FRONT_START = 0.85
        self.BOUNDARY_FRONT_FULL = 0.45
        self.BOUNDARY_FRONT_MAX_TURN = 12.0
        self.PASS_MIN_TIME = 1.2
        self.pass_start_time = 0.0
        self.UWB_ANCHOR_BASELINE_CM = 28.0
        self.UWB_TIMEOUT = 0.8
        self.GOAL_SECTOR_HALF_ANGLE = 18.0
        self.GOAL_CLEAR_DISTANCE = 1.3
        self.GOAL_CLEAR_HOLD_TIME = 0.55
        self.goal_clear_start = None
        self.MAX_BOUNDARY_TIME = 12.0
        self.boundary_timeout_warned = False
        self.RECOVER_TIME = 0.45
        self.RECOVER_RETURN_TURN = 8.0
        self.recover_start_time = 0.0
        self.SCAN_MAX_DISTANCE = 2.5
        self.avoid_state = 'TRACK'
        self.avoid_direction = 0
        self.avoid_start_time = 0.0
        self.uwb_direction_error = 0.0
        self.uwb_direction = 'LOST'
        self.last_uwb = 0.0
        self.avoidance_disabled = False
        self.steering_pub = self.create_publisher(Float32, '/avoidance/steering', 10)
        self.threat_pub = self.create_publisher(Float32, '/avoidance/threat', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.front_clearance_pub = self.create_publisher(Float32, '/avoidance/front_clearance', 10)
        self.rear_clearance_pub = self.create_publisher(Float32, '/avoidance/rear_clearance', 10)
        self.state_pub = self.create_publisher(String, '/avoidance/state', 10)
        self.left_score_pub = self.create_publisher(Float32, '/avoidance/left_score', 10)
        self.right_score_pub = self.create_publisher(Float32, '/avoidance/right_score', 10)
        self.side_clearance_pub = self.create_publisher(Float32, '/avoidance/obstacle_side_clearance', 10)
        self.rear_side_clearance_pub = self.create_publisher(Float32, '/avoidance/obstacle_rear_side_clearance', 10)
        self.goal_clearance_pub = self.create_publisher(Float32, '/avoidance/goal_clearance', 10)
        self.boundary_distance_pub = self.create_publisher(Float32, '/avoidance/boundary_distance', 10)
        self.boundary_angle_pub = self.create_publisher(Float32, '/avoidance/boundary_angle', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/avoidance/disable', self.disable_callback, 10)
        self.create_subscription(Float32, '/uwb/direction_error', self.uwb_error_callback, 10)
        self.create_subscription(String, '/uwb/direction', self.uwb_direction_callback, 10)
        self.get_logger().info('LiDAR avoidance started | STRONG BOUNDARY + RECOVER RETURN ON | TRACK -> AVOID -> PASSING -> RECOVER')

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def normalize_angle_deg(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def reset_avoidance_state(self):
        self.avoid_state = 'TRACK'
        self.avoid_direction = 0
        self.avoid_start_time = 0.0
        self.pass_start_time = 0.0
        self.recover_start_time = 0.0
        self.acquire_start_time = None
        self.boundary_lost_start = None
        self.goal_clear_start = None
        self.boundary_timeout_warned = False

    def disable_callback(self, msg):
        disabled = bool(msg.data)
        if disabled and (not self.avoidance_disabled):
            self.avoidance_disabled = True
            self.reset_avoidance_state()
            self.get_logger().warn('NORMAL AVOIDANCE DISABLED')
        elif not disabled and self.avoidance_disabled:
            self.reset_avoidance_state()
            self.avoidance_disabled = False
            self.get_logger().info('NORMAL AVOIDANCE ENABLED')

    def uwb_error_callback(self, msg):
        self.uwb_direction_error = float(msg.data)
        self.last_uwb = time.monotonic()

    def uwb_direction_callback(self, msg):
        self.uwb_direction = str(msg.data)
        self.last_uwb = time.monotonic()

    def estimate_user_bearing_deg(self):
        now = time.monotonic()
        if self.uwb_direction == 'LOST':
            return None
        if now - self.last_uwb > self.UWB_TIMEOUT:
            return None
        if self.uwb_direction == 'CENTER':
            return 0.0
        ratio = self.uwb_direction_error / self.UWB_ANCHOR_BASELINE_CM
        ratio = self.clamp(ratio, -0.95, 0.95)
        bearing = -math.degrees(math.asin(ratio))
        return self.clamp(bearing, -72.0, 72.0)

    def valid_range(self, r, msg):
        if not math.isfinite(r):
            return False
        if r <= 0.0:
            return False
        if r < msg.range_min:
            return False
        if r > msg.range_max:
            return False
        return True

    def percentile(self, values, fraction):
        if not values:
            return self.SCAN_MAX_DISTANCE
        values = sorted(values)
        index = int((len(values) - 1) * fraction)
        index = max(0, min(len(values) - 1, index))
        return values[index]

    def get_sector_values(self, msg, center_deg, half_width_deg):
        values = []
        angle = msg.angle_min
        for r in msg.ranges:
            deg = math.degrees(angle)
            deg = self.normalize_angle_deg(deg)
            diff = self.normalize_angle_deg(deg - center_deg)
            if abs(diff) <= half_width_deg:
                if self.valid_range(r, msg):
                    values.append(min(r, self.SCAN_MAX_DISTANCE))
            angle += msg.angle_increment
        return values

    def sector_clearance(self, msg, center_deg, half_width_deg, fraction=0.25):
        values = self.get_sector_values(msg, center_deg, half_width_deg)
        if not values:
            return self.SCAN_MAX_DISTANCE
        return self.percentile(values, fraction)

    def calculate_front_clearance(self, msg):
        values = self.get_sector_values(msg, 0.0, self.FRONT_HALF_ANGLE)
        if not values:
            return self.SCAN_MAX_DISTANCE
        return self.percentile(values, 0.1)

    def calculate_rear_clearance(self, msg):
        return self.sector_clearance(msg, 180.0, 25.0)

    def calculate_emergency(self, msg):
        now = time.monotonic()
        values = self.get_sector_values(msg, 0.0, self.EMERGENCY_HALF_ANGLE)
        minimum = None
        if values:
            minimum = min(values)
        if minimum is not None and minimum <= self.EMERGENCY_DISTANCE:
            self.last_emergency_detect_time = now
            return True
        if now - self.last_emergency_detect_time < self.EMERGENCY_HOLD_TIME:
            return True
        return False

    def calculate_side_scores(self, msg):
        left_30 = self.sector_clearance(msg, 30.0, 18.0)
        left_60 = self.sector_clearance(msg, 60.0, 18.0)
        left_90 = self.sector_clearance(msg, 90.0, 18.0)
        left_score = 0.5 * left_30 + 0.3 * left_60 + 0.2 * left_90
        right_30 = self.sector_clearance(msg, -30.0, 18.0)
        right_60 = self.sector_clearance(msg, -60.0, 18.0)
        right_90 = self.sector_clearance(msg, -90.0, 18.0)
        right_score = 0.5 * right_30 + 0.3 * right_60 + 0.2 * right_90
        return (left_score, right_score)

    def start_avoidance(self, left_score, right_score, now):
        if right_score > left_score + self.DIRECTION_SWITCH_MARGIN:
            self.avoid_direction = 1
            direction_text = 'RIGHT'
        elif left_score > right_score + self.DIRECTION_SWITCH_MARGIN:
            self.avoid_direction = -1
            direction_text = 'LEFT'
        elif right_score >= left_score:
            self.avoid_direction = 1
            direction_text = 'RIGHT'
        else:
            self.avoid_direction = -1
            direction_text = 'LEFT'
        self.avoid_state = 'AVOID'
        self.avoid_start_time = now
        self.acquire_start_time = None
        self.boundary_lost_start = None
        self.goal_clear_start = None
        self.boundary_timeout_warned = False
        self.get_logger().info(f'AVOID START -> {direction_text} | L={left_score:.2f} R={right_score:.2f}')

    def calculate_avoid_steering(self, front_clearance):
        proximity = (self.AVOID_START_DISTANCE - front_clearance) / (self.AVOID_START_DISTANCE - self.FULL_AVOID_DISTANCE)
        proximity = self.clamp(proximity, 0.0, 1.0)
        ratio = proximity ** 0.8
        steering = self.avoid_direction * self.MAX_STEERING * ratio
        return self.clamp(steering, -self.MAX_STEERING, self.MAX_STEERING)

    def find_boundary(self, msg):
        if self.avoid_direction == 0:
            return (False, self.SCAN_MAX_DISTANCE, 90.0)
        obstacle_side_sign = 1.0 if self.avoid_direction == 1 else -1.0
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            deg = math.degrees(angle)
            deg = self.normalize_angle_deg(deg)
            side_angle = obstacle_side_sign * deg
            if self.BOUNDARY_ARC_MIN <= side_angle <= self.BOUNDARY_ARC_MAX:
                if self.valid_range(r, msg):
                    r2 = min(r, self.SCAN_MAX_DISTANCE)
                    points.append((r2, side_angle))
            angle += msg.angle_increment
        if not points:
            return (False, self.SCAN_MAX_DISTANCE, 90.0)
        distances = [p[0] for p in points]
        near_distance = self.percentile(distances, 0.12)
        cluster_limit = near_distance + self.BOUNDARY_CLUSTER_BAND
        cluster = [p for p in points if p[0] <= cluster_limit]
        if not cluster:
            return (False, self.SCAN_MAX_DISTANCE, 90.0)
        cluster_distances = [p[0] for p in cluster]
        boundary_distance = self.percentile(cluster_distances, 0.3)
        weighted_angle = 0.0
        weight_sum = 0.0
        for distance, side_angle in cluster:
            weight = 1.0 / max(0.1, distance * distance)
            weighted_angle += weight * side_angle
            weight_sum += weight
        if weight_sum <= 0.0:
            boundary_angle = 90.0
        else:
            boundary_angle = weighted_angle / weight_sum
        boundary_found = boundary_distance < self.BOUNDARY_DETECT_DISTANCE
        return (boundary_found, boundary_distance, boundary_angle)

    def calculate_obstacle_rear_side(self, msg):
        if self.avoid_direction == 1:
            return self.sector_clearance(msg, 135.0, 25.0)
        if self.avoid_direction == -1:
            return self.sector_clearance(msg, -135.0, 25.0)
        return self.SCAN_MAX_DISTANCE

    def calculate_boundary_steering(self, msg, front_clearance, boundary_found, boundary_distance, boundary_angle):
        if boundary_found:
            distance_error = self.TARGET_SIDE_DISTANCE - boundary_distance
            distance_turn = self.avoid_direction * self.BOUNDARY_DISTANCE_KP * distance_error
            distance_turn = self.clamp(distance_turn, -self.MAX_DISTANCE_TURN, self.MAX_DISTANCE_TURN)
            angle_error = self.BOUNDARY_ANGLE_TARGET - boundary_angle
            angle_turn = self.avoid_direction * self.BOUNDARY_ANGLE_KP * angle_error
            angle_turn = self.clamp(angle_turn, -self.MAX_ANGLE_TURN, self.MAX_ANGLE_TURN)
            steering = distance_turn + angle_turn
        else:
            steering = -self.avoid_direction * self.BOUNDARY_SEARCH_TURN
        if front_clearance < self.BOUNDARY_FRONT_START:
            proximity = (self.BOUNDARY_FRONT_START - front_clearance) / (self.BOUNDARY_FRONT_START - self.BOUNDARY_FRONT_FULL)
            proximity = self.clamp(proximity, 0.0, 1.0)
            steering += self.avoid_direction * self.BOUNDARY_FRONT_MAX_TURN * proximity
        return self.clamp(steering, -self.MAX_BOUNDARY_STEERING, self.MAX_BOUNDARY_STEERING)

    def calculate_recover_steering(self, now):
        if self.avoid_direction == 0:
            return 0.0
        elapsed = now - self.recover_start_time
        ratio = self.clamp(elapsed / self.RECOVER_TIME, 0.0, 1.0)
        strength = 1.0 - ratio
        steering = -self.avoid_direction * self.RECOVER_RETURN_TURN * strength
        return steering

    def calculate_goal_clearance(self, msg):
        bearing = self.estimate_user_bearing_deg()
        if bearing is None:
            return (None, None)
        clearance = self.sector_clearance(msg, bearing, self.GOAL_SECTOR_HALF_ANGLE)
        return (bearing, clearance)

    def calculate_threat(self, front_clearance):
        if front_clearance >= self.AVOID_START_DISTANCE:
            return 0.0
        if front_clearance <= self.EMERGENCY_DISTANCE:
            return 1.0
        threat = (self.AVOID_START_DISTANCE - front_clearance) / (self.AVOID_START_DISTANCE - self.EMERGENCY_DISTANCE)
        return self.clamp(threat, 0.0, 1.0)

    def update_avoidance(self, msg, front_clearance):
        now = time.monotonic()
        left_score, right_score = self.calculate_side_scores(msg)
        boundary_found = False
        boundary_distance = self.SCAN_MAX_DISTANCE
        boundary_angle = 90.0
        obstacle_rear_side = self.SCAN_MAX_DISTANCE
        goal_clearance = self.SCAN_MAX_DISTANCE
        steering = 0.0
        if self.avoid_state == 'TRACK':
            if front_clearance < self.AVOID_START_DISTANCE:
                self.start_avoidance(left_score, right_score, now)
                steering = self.calculate_avoid_steering(front_clearance)
        elif self.avoid_state == 'AVOID':
            boundary_found, boundary_distance, boundary_angle = self.find_boundary(msg)
            obstacle_rear_side = self.calculate_obstacle_rear_side(msg)
            if front_clearance <= 0.95:
                self.acquire_start_time = None
                steering = self.calculate_avoid_steering(front_clearance)
            elif boundary_found:
                self.avoid_state = 'PASSING'
                self.pass_start_time = now
                self.acquire_start_time = None
                self.boundary_lost_start = None
                self.goal_clear_start = None
                steering = self.calculate_boundary_steering(msg, front_clearance, boundary_found, boundary_distance, boundary_angle)
                self.get_logger().info(f'AVOID -> PASSING | BOUNDARY ACQUIRED | D={boundary_distance:.2f}m | A={boundary_angle:.1f}deg')
            else:
                if self.acquire_start_time is None:
                    self.acquire_start_time = now
                acquire_elapsed = now - self.acquire_start_time
                if acquire_elapsed < self.ACQUIRE_TIMEOUT:
                    steering = -self.avoid_direction * self.ACQUIRE_SEARCH_TURN
                else:
                    self.avoid_state = 'RECOVER'
                    self.recover_start_time = now
                    self.acquire_start_time = None
                    steering = self.calculate_recover_steering(now)
                    self.get_logger().info('AVOID -> RECOVER | BOUNDARY NOT FOUND')
        elif self.avoid_state == 'PASSING':
            boundary_found, boundary_distance, boundary_angle = self.find_boundary(msg)
            obstacle_rear_side = self.calculate_obstacle_rear_side(msg)
            steering = self.calculate_boundary_steering(msg, front_clearance, boundary_found, boundary_distance, boundary_angle)
            pass_elapsed = now - self.pass_start_time
            if boundary_found:
                self.boundary_lost_start = None
            elif self.boundary_lost_start is None:
                self.boundary_lost_start = now
            goal_bearing, measured_goal_clearance = self.calculate_goal_clearance(msg)
            if measured_goal_clearance is not None:
                goal_clearance = measured_goal_clearance
            goal_path_clear = goal_bearing is not None and goal_clearance >= self.GOAL_CLEAR_DISTANCE and (front_clearance >= 1.0)
            if pass_elapsed < self.PASS_MIN_TIME:
                self.goal_clear_start = None
            elif goal_path_clear:
                if self.goal_clear_start is None:
                    self.goal_clear_start = now
                elif now - self.goal_clear_start >= self.GOAL_CLEAR_HOLD_TIME:
                    self.avoid_state = 'RECOVER'
                    self.recover_start_time = now
                    self.goal_clear_start = None
                    self.boundary_lost_start = None
                    steering = self.calculate_recover_steering(now)
                    self.get_logger().info(f'PASSING -> RECOVER | USER PATH CLEAR | angle={goal_bearing:.1f} | clear={goal_clearance:.2f}m')
            else:
                self.goal_clear_start = None
            if self.avoid_state == 'PASSING' and self.boundary_lost_start is not None:
                lost_elapsed = now - self.boundary_lost_start
                if pass_elapsed >= self.PASS_MIN_TIME and lost_elapsed >= self.BOUNDARY_LOST_HOLD_TIME and (front_clearance >= 1.0):
                    self.avoid_state = 'RECOVER'
                    self.recover_start_time = now
                    self.boundary_lost_start = None
                    self.goal_clear_start = None
                    steering = self.calculate_recover_steering(now)
                    self.get_logger().info('PASSING -> RECOVER | BOUNDARY END')
            if self.avoid_state == 'PASSING' and pass_elapsed >= self.MAX_BOUNDARY_TIME and (not self.boundary_timeout_warned):
                self.boundary_timeout_warned = True
                self.get_logger().warn('BOUNDARY FOLLOW LONG | waiting safe exit')
        elif self.avoid_state == 'RECOVER':
            steering = self.calculate_recover_steering(now)
            if front_clearance < self.AVOID_START_DISTANCE:
                self.start_avoidance(left_score, right_score, now)
                steering = self.calculate_avoid_steering(front_clearance)
            elif now - self.recover_start_time >= self.RECOVER_TIME:
                self.reset_avoidance_state()
                self.get_logger().info('RECOVER -> TRACK')
                steering = 0.0
        return (steering, left_score, right_score, boundary_distance, boundary_angle, obstacle_rear_side, goal_clearance)

    def scan_callback(self, msg):
        front_clearance = self.calculate_front_clearance(msg)
        rear_clearance = self.calculate_rear_clearance(msg)
        emergency = self.calculate_emergency(msg)
        if self.avoidance_disabled:
            self.reset_avoidance_state()
            steering = 0.0
            threat = 0.0
            left_score = self.SCAN_MAX_DISTANCE
            right_score = self.SCAN_MAX_DISTANCE
            boundary_distance = self.SCAN_MAX_DISTANCE
            boundary_angle = 90.0
            obstacle_rear_side = self.SCAN_MAX_DISTANCE
            goal_clearance = self.SCAN_MAX_DISTANCE
        else:
            steering, left_score, right_score, boundary_distance, boundary_angle, obstacle_rear_side, goal_clearance = self.update_avoidance(msg, front_clearance)
            threat = self.calculate_threat(front_clearance)
        m = Float32()
        m.data = float(steering)
        self.steering_pub.publish(m)
        m = Float32()
        m.data = float(threat)
        self.threat_pub.publish(m)
        m = Bool()
        m.data = bool(emergency)
        self.emergency_pub.publish(m)
        m = Float32()
        m.data = float(front_clearance)
        self.front_clearance_pub.publish(m)
        m = Float32()
        m.data = float(rear_clearance)
        self.rear_clearance_pub.publish(m)
        m = String()
        if self.avoidance_disabled:
            m.data = 'DISABLED'
        else:
            m.data = self.avoid_state
        self.state_pub.publish(m)
        m = Float32()
        m.data = float(left_score)
        self.left_score_pub.publish(m)
        m = Float32()
        m.data = float(right_score)
        self.right_score_pub.publish(m)
        m = Float32()
        m.data = float(boundary_distance)
        self.side_clearance_pub.publish(m)
        m = Float32()
        m.data = float(obstacle_rear_side)
        self.rear_side_clearance_pub.publish(m)
        m = Float32()
        m.data = float(goal_clearance)
        self.goal_clearance_pub.publish(m)
        m = Float32()
        m.data = float(boundary_distance)
        self.boundary_distance_pub.publish(m)
        m = Float32()
        m.data = float(boundary_angle)
        self.boundary_angle_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
