import json
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def parse_arc(text):
    parts = str(text).replace(',', ':').split(':')
    if len(parts) != 2:
        return None
    try:
        return float(parts[0]) % 360.0, float(parts[1]) % 360.0
    except ValueError:
        return None


def in_wrapped_arc(angle, start, end):
    angle = angle % 360.0
    start = start % 360.0
    end = end % 360.0
    if start <= end:
        return start <= angle <= end
    return angle >= start or angle <= end


def robot_angle(raw_angle, front_center):
    return (front_center - raw_angle) % 360.0


def signed_robot_angle(angle):
    return angle if angle <= 180.0 else angle - 360.0


def in_sector(angle, center, half_width):
    diff = ((angle - center + 180.0) % 360.0) - 180.0
    return abs(diff) <= half_width


def percentile(values, pct):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), pct))


def sector_stats(points, center, half_width):
    values = sorted(dist for angle, dist in points if in_sector(angle, center, half_width))
    return {
        'min_m': values[0] if values else None,
        'p10_m': percentile(values, 10.0),
        'median_m': percentile(values, 50.0),
        'count': len(values),
    }


def choose_stat(stats, name):
    if name == 'min':
        return stats['min_m']
    if name == 'median':
        return stats['median_m']
    return stats['p10_m']


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        self.declare_parameter('front_center_deg', 0.0)
        self.declare_parameter('front_width_deg', 36.0)
        self.declare_parameter('diagonal_width_deg', 42.0)
        self.declare_parameter('side_width_deg', 70.0)
        self.declare_parameter('rear_width_deg', 46.0)
        self.declare_parameter('decision_stat', 'p10')
        self.declare_parameter('obstacle_threshold_m', 0.30)
        self.declare_parameter('max_range_m', 1.50)
        self.declare_parameter('min_range_m', 0.03)
        self.declare_parameter('stale_timeout_s', 0.50)
        self.declare_parameter('cluster_gap_deg', 10.0)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('ignore_close_arcs', '35:55')
        self.declare_parameter('ignore_close_under_m', 0.08)

        self.front_center = float(self.get_parameter('front_center_deg').value)
        self.front_width = float(self.get_parameter('front_width_deg').value)
        self.diagonal_width = float(self.get_parameter('diagonal_width_deg').value)
        self.side_width = float(self.get_parameter('side_width_deg').value)
        self.rear_width = float(self.get_parameter('rear_width_deg').value)
        self.decision_stat = str(self.get_parameter('decision_stat').value)
        self.obstacle_threshold = float(self.get_parameter('obstacle_threshold_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout_s').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap_deg').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.ignore_close_under_m = float(self.get_parameter('ignore_close_under_m').value)
        self.ignore_arcs = [
            arc for arc in (
                parse_arc(part.strip())
                for part in str(self.get_parameter('ignore_close_arcs').value).split(';')
                if part.strip()
            )
            if arc is not None
        ]

        self.last_scan_s = 0.0
        self.last_summary = None

        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle', 10)
        self.sector_pub = self.create_publisher(String, '/lidar_sectors', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('LiDAR obstacle/sector detector ready')

    def ignored_close_point(self, rel_angle, distance):
        if distance >= self.ignore_close_under_m:
            return False
        return any(in_wrapped_arc(rel_angle, start, end) for start, end in self.ignore_arcs)

    def scan_points(self, msg):
        points = []
        for i, distance in enumerate(msg.ranges):
            if np.isinf(distance) or np.isnan(distance):
                continue
            distance = float(distance)
            if distance <= 0.0:
                continue
            if distance < max(self.min_range, float(msg.range_min or 0.0)):
                continue
            if msg.range_max > 0 and distance > float(msg.range_max):
                continue
            if distance > self.max_range:
                continue
            raw_deg = np.degrees(float(msg.angle_min) + i * float(msg.angle_increment)) % 360.0
            rel = robot_angle(raw_deg, self.front_center)
            if self.ignored_close_point(rel, distance):
                continue
            points.append((rel, distance))
        return points

    def make_summary(self, points):
        front = sector_stats(points, 0.0, self.front_width * 0.5)
        front_right = sector_stats(points, 35.0, self.diagonal_width * 0.5)
        front_left = sector_stats(points, 325.0, self.diagonal_width * 0.5)
        right = sector_stats(points, 90.0, self.side_width * 0.5)
        left = sector_stats(points, 270.0, self.side_width * 0.5)
        rear = sector_stats(points, 180.0, self.rear_width * 0.5)
        return {
            'ok': True,
            'age_s': 0.0,
            'point_count': len(points),
            'front_m': choose_stat(front, self.decision_stat),
            'front_left_m': choose_stat(front_left, self.decision_stat),
            'front_right_m': choose_stat(front_right, self.decision_stat),
            'left_m': choose_stat(left, 'median'),
            'right_m': choose_stat(right, 'median'),
            'rear_m': choose_stat(rear, 'median'),
            'front_count': front['count'],
            'left_count': left['count'],
            'right_count': right['count'],
            'rear_count': rear['count'],
            'reason': 'ok',
            'timestamp': time.time(),
        }

    def make_obstacles(self, points):
        close = []
        for rel, distance in points:
            if distance >= self.obstacle_threshold:
                continue
            signed = signed_robot_angle(rel)
            if abs(signed) > 90.0:
                continue
            close.append({
                'angle_deg': round(signed, 1),
                'distance_cm': round(distance * 100.0, 1),
            })

        obstacles = []
        if close:
            close.sort(key=lambda item: item['angle_deg'])
            clusters = []
            current = [close[0]]
            for point in close[1:]:
                if point['angle_deg'] - current[-1]['angle_deg'] < self.cluster_gap:
                    current.append(point)
                else:
                    clusters.append(current)
                    current = [point]
            clusters.append(current)

            for cluster in clusters:
                if len(cluster) < self.min_cluster_points:
                    continue
                closest = min(cluster, key=lambda item: item['distance_cm'])
                avg_angle = float(np.mean([item['angle_deg'] for item in cluster]))
                obstacles.append({
                    'angle_deg': round(avg_angle, 1),
                    'distance_cm': float(closest['distance_cm']),
                    'point_count': len(cluster),
                    'side': 'left' if avg_angle < -5 else 'right' if avg_angle > 5 else 'center',
                })
        return obstacles

    def publish(self, summary, obstacles):
        sector_msg = String()
        sector_msg.data = json.dumps(summary)
        self.sector_pub.publish(sector_msg)

        obstacle_msg = String()
        obstacle_msg.data = json.dumps({
            'obstacle_count': len(obstacles),
            'obstacles': obstacles,
            'lidar_ok': summary.get('ok', False),
            'age_s': summary.get('age_s', 999.0),
            'front_m': summary.get('front_m'),
            'left_m': summary.get('left_m'),
            'right_m': summary.get('right_m'),
            'rear_m': summary.get('rear_m'),
        })
        self.obstacle_pub.publish(obstacle_msg)

    def scan_callback(self, msg):
        points = self.scan_points(msg)
        summary = self.make_summary(points)
        obstacles = self.make_obstacles(points)
        self.last_scan_s = time.monotonic()
        self.last_summary = summary
        self.publish(summary, obstacles)

    def timer_callback(self):
        if self.last_scan_s <= 0.0:
            return
        age = time.monotonic() - self.last_scan_s
        if age <= self.stale_timeout:
            return
        summary = dict(self.last_summary or {})
        summary.update({
            'ok': False,
            'age_s': round(age, 3),
            'reason': f'lidar stale {age:.2f}s',
            'timestamp': time.time(),
        })
        self.publish(summary, [])


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
