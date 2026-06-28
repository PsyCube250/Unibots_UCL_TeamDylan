import json
import math

import cv2
import dt_apriltags as apriltag
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


WALL_MAP = {
    **{i: 'North' for i in range(0, 6)},
    **{i: 'East' for i in range(6, 12)},
    **{i: 'South' for i in range(12, 18)},
    **{i: 'West' for i in range(18, 24)},
}


def parse_int_set(value):
    return {
        int(part.strip())
        for part in str(value).split(',')
        if part.strip()
    }


def focal_from_hfov(width_px, hfov_deg):
    hfov_rad = math.radians(max(20.0, min(170.0, float(hfov_deg))))
    return (float(width_px) * 0.5) / math.tan(hfov_rad * 0.5)


def image_msg_to_bgr(msg: Image):
    channels_by_encoding = {
        'bgr8': 3,
        'rgb8': 3,
        'mono8': 1,
        'bgra8': 4,
        'rgba8': 4,
    }
    encoding = msg.encoding.lower()
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')

    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    useful = row[:, : msg.width * channels]
    frame = useful.reshape(msg.height, msg.width, channels)

    if encoding == 'bgr8':
        return frame.copy()
    if encoding == 'rgb8':
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if encoding == 'mono8':
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)


def polygon_area(points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    x = pts[:, 0]
    y = pts[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def pose_distance(det, focal_px, tag_size_m):
    try:
        if det.pose_t is not None:
            return float(det.pose_t[2][0])
    except Exception:
        pass
    area = max(1.0, polygon_area(det.corners))
    side_px = math.sqrt(area)
    return float(tag_size_m) * float(focal_px) / side_px


def tag_to_dict(det, frame_w, focal_px, tag_size_m, own_zone):
    tag_id = int(det.tag_id)
    center = np.asarray(det.center, dtype=np.float32).reshape(2)
    angle = math.degrees(math.atan2(float(center[0]) - frame_w * 0.5, focal_px))
    distance_m = pose_distance(det, focal_px, tag_size_m)
    wall = WALL_MAP.get(tag_id, 'Unknown')
    return {
        'id': tag_id,
        'tag_id': tag_id,
        'wall': wall,
        'angle': round(angle, 1),
        'angle_deg': round(angle, 2),
        'distance_cm': round(distance_m * 100.0, 1),
        'distance_m': round(distance_m, 3),
        'center': [round(float(center[0]), 1), round(float(center[1]), 1)],
        'area_px2': round(polygon_area(det.corners), 1),
        'own_zone': wall == own_zone,
    }


def choose_target(tags, target_ids, frame_w, focal_px):
    targets = [tag for tag in tags if int(tag['id']) in target_ids]
    distractor_count = max(0, len(tags) - len(targets))
    if not targets:
        return None, distractor_count

    by_id = {int(tag['id']): tag for tag in targets}
    if 20 in by_id and 21 in by_id:
        pair = [by_id[20], by_id[21]]
        cx = sum(tag['center'][0] for tag in pair) * 0.5
        cy = sum(tag['center'][1] for tag in pair) * 0.5
        distance_m = sum(tag['distance_m'] for tag in pair) / len(pair)
        angle = math.degrees(math.atan2(cx - frame_w * 0.5, focal_px))
        return {
            'id': '20+21',
            'tag_id': '20+21',
            'wall': 'West',
            'angle': round(angle, 1),
            'angle_deg': round(angle, 2),
            'distance_cm': round(distance_m * 100.0, 1),
            'distance_m': round(distance_m, 3),
            'center': [round(cx, 1), round(cy, 1)],
            'area_px2': round(sum(tag['area_px2'] for tag in pair), 1),
            'note': 'midpoint between target tags 20 and 21',
        }, distractor_count

    visible = min(targets, key=lambda tag: tag['distance_m'] if tag['distance_m'] is not None else 99.0)
    return visible, distractor_count


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        self.declare_parameter('own_zone', 'North')
        self.declare_parameter('target_tag_ids', '20,21')
        self.declare_parameter('tag_size_m', 0.10)
        self.declare_parameter('hfov_deg', 120.0)
        self.declare_parameter('families', 'tag36h11')
        self.declare_parameter('quad_decimate', 1.5)
        self.declare_parameter('quad_sigma', 0.0)
        self.declare_parameter('nthreads', 2)

        self.target_ids = parse_int_set(self.get_parameter('target_tag_ids').value)
        self.tag_size_m = float(self.get_parameter('tag_size_m').value)
        self.hfov_deg = float(self.get_parameter('hfov_deg').value)

        self.subscription = self.create_subscription(Image, '/img', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/apriltag_detections', 10)

        self.detector = apriltag.Detector(
            families=str(self.get_parameter('families').value),
            nthreads=int(self.get_parameter('nthreads').value),
            quad_decimate=float(self.get_parameter('quad_decimate').value),
            quad_sigma=float(self.get_parameter('quad_sigma').value),
            refine_edges=1,
            decode_sharpening=0.25,
        )
        self.get_logger().info(f'AprilTag detector ready, target IDs={sorted(self.target_ids)}')

    def image_callback(self, msg):
        own_zone = self.get_parameter('own_zone').get_parameter_value().string_value
        try:
            frame = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f'Bad image frame: {exc}')
            return

        frame_h, frame_w = frame.shape[:2]
        focal_px = focal_from_hfov(frame_w, self.hfov_deg)
        camera_params = (focal_px, focal_px, frame_w * 0.5, frame_h * 0.5)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=self.tag_size_m,
        )

        tags = [tag_to_dict(det, frame_w, focal_px, self.tag_size_m, own_zone) for det in detections]
        target, distractor_count = choose_target(tags, self.target_ids, frame_w, focal_px)
        facing_wall = None
        if tags:
            facing_wall = min(tags, key=lambda tag: tag['distance_m'])['wall']

        out = String()
        out.data = json.dumps({
            'facing': facing_wall,
            'tags': tags,
            'target': target,
            'target_ids': sorted(self.target_ids),
            'distractor_count': distractor_count,
            'own_zone': own_zone,
            'frame_width': frame_w,
            'frame_height': frame_h,
        })
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
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
