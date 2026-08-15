#라이다를 통해 가장 가까운 장애물의 위치를 알려주는 코드

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math

class Lidar360ClosestNode(Node):
    def __init__(self):
        super().__init__('lidar_360_closest_node')
        
        # 라이다 토픽 구독 (/scan)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("✅ 360도 최근접 장애물 탐지 노드 시작!")

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) == 0:
            return

        min_dist = float('inf')
        closest_angle = 0.0

        # 360도 전방위 4분할 데이터 수집용
        front_dists = []
        right_dists = []
        left_dists = []
        rear_dists = []

        for i, r in enumerate(ranges):
            # 유효한 거리 값 필터링 (nan, inf 및 거리 범위 제외)
            if 0.05 < r < msg.range_max and not math.isnan(r) and not math.isinf(r):
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad)

                # 각도 정규화 (-180도 ~ +180도)
                while angle_deg > 180: angle_deg -= 360
                while angle_deg < -180: angle_deg += 360

                # 1. 360도 전체에서 가장 가까운 장애물 업데이트
                if r < min_dist:
                    min_dist = r
                    closest_angle = angle_deg

                # 2. 360도 4분할 방위 영역 분류 (-180° ~ +180°)
                if -45.0 <= angle_deg <= 45.0:
                    front_dists.append(r)
                elif 45.0 < angle_deg <= 135.0:
                    right_dists.append(r)
                elif -135.0 <= angle_deg < -45.0:
                    left_dists.append(r)
                else:  # 135.0 < angle_deg <= 180.0 또는 -180.0 <= angle_deg < -135.0
                    rear_dists.append(r)

        # 감지된 장애물이 없는 경우 처리
        if min_dist == float('inf'):
            self.get_logger().info("⚠️ 360도 범위 내 유효한 장애물이 없습니다.")
            return

        # 가장 가까운 장애물의 방향 텍스트 구하기
        if -45.0 <= closest_angle <= 45.0:
            dir_str = "전방"
        elif 45.0 < closest_angle <= 135.0:
            dir_str = "우측"
        elif -135.0 <= closest_angle < -45.0:
            dir_str = "좌측"
        else:
            dir_str = "후방"

        # 각 방위별 최소 거리 산출 (없을 경우 9.9m)
        d_front = min(front_dists) if front_dists else 9.9
        d_right = min(right_dists) if right_dists else 9.9
        d_left = min(left_dists) if left_dists else 9.9
        d_rear = min(rear_dists) if rear_dists else 9.9

        # 터미널 실시간 출력
        self.get_logger().info(
            f"🎯 [최근접] 거리: {min_dist:5.2f}m | 각도: {closest_angle:+6.1f}° ({dir_str}) || "
            f"전방: {d_front:4.2f}m | 우측: {d_right:4.2f}m | 좌측: {d_left:4.2f}m | 후방: {d_rear:4.2f}m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Lidar360ClosestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()