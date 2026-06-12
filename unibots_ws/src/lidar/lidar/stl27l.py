import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import json

# Obstacle threshold in metres
OBSTACLE_THRESHOLD = 0.30
MAX_RANGE = 1.5
CLUSTER_GAP_DEG = 10.0
MIN_CLUSTER_POINTS = 3

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.publisher = self.create_publisher(String, '/obstacle', 10)
        self.get_logger().info('Lidar obstacle detector ready!')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        total = len(ranges)

        # STL27L outputs 0 to 2pi
        # Front 180 = last 90 degrees (270-360) + first 90 degrees (0-90)
        # i.e. angles 270-360 deg and 0-90 deg = index where angle < pi/2 or > 3pi/2

        close_points = []
        for i, r in enumerate(ranges):
            angle_rad = angle_min + i * angle_increment
            angle_deg = np.degrees(angle_rad) % 360

            # Front 180: 0-90 (right) and 270-360 (left)
            if not (angle_deg <= 90.0 or angle_deg >= 270.0):
                continue

            if np.isinf(r) or np.isnan(r) or r <= 0.0:
                continue

            if r > MAX_RANGE:
                continue

            if r < OBSTACLE_THRESHOLD:
                # Convert to -180 to 180 range for intuitive output
                if angle_deg > 180:
                    normalised = angle_deg - 360
                else:
                    normalised = angle_deg

                close_points.append({
                    'angle_deg': round(normalised, 1),
                    'distance_cm': round(float(r) * 100, 1)
                })

        # Cluster by angle proximity
        obstacles = []
        if close_points:
            close_points.sort(key=lambda x: x['angle_deg'])
            clusters = []
            current = [close_points[0]]

            for pt in close_points[1:]:
                if pt['angle_deg'] - current[-1]['angle_deg'] < CLUSTER_GAP_DEG:
                    current.append(pt)
                else:
                    clusters.append(current)
                    current = [pt]
            clusters.append(current)

            for cluster in clusters:
                if len(cluster) >= MIN_CLUSTER_POINTS:
                    closest = min(cluster, key=lambda x: x['distance_cm'])
                    avg_angle = float(np.mean([p['angle_deg'] for p in cluster]))
                    obstacles.append({
                        'angle_deg': round(avg_angle, 1),
                        'distance_cm': float(closest['distance_cm']),
                        'point_count': len(cluster),
                        'side': 'left' if avg_angle < -5 else 'right' if avg_angle > 5 else 'center'
                    })

        if obstacles:
            self.get_logger().info(f'Obstacles detected: {obstacles}')

        out = String()
        out.data = json.dumps({
            'obstacle_count': len(obstacles),
            'obstacles': obstacles
        })
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
