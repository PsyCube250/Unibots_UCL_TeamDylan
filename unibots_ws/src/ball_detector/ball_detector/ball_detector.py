import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import time
import numpy as np

# Camera constants
FOCAL_LENGTH = 516.0
BALL_DIAMETER_CM = 4.0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 320

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        self.subscription = self.create_subscription(
            Image,
            '/img',
            self.image_callback,
            10)

        self.detection_pub = self.create_publisher(String, '/ball_detections', 10)
        self.navigate_pub = self.create_publisher(Float32MultiArray, '/ball_navigate', 10)

        self.bridge = CvBridge()
        self.target_class = 32

        self.get_logger().info('Loading TensorRT engine...')
        self.model = YOLO('/home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan/unibots_ws/src/ball_detector/ball_detector/yolo26n.engine')

        self.get_logger().info('Warming up model...')
        self.model.predict(
            'https://ultralytics.com/images/bus.jpg',
            device='cuda',
            half=True,
            verbose=False,
            imgsz=320
        )

        self.t0 = time.time()
        self.get_logger().info('Ball detector ready!')

    def estimate_distance(self, box):
        box_height = box[3] - box[1]
        if box_height <= 0:
            return float('inf')
        return (BALL_DIAMETER_CM * FOCAL_LENGTH) / box_height

    def estimate_angle(self, box):
        cx = (box[0] + box[2]) / 2.0
        offset = cx - (IMAGE_WIDTH / 2.0)
        return float(np.degrees(np.arctan(offset / FOCAL_LENGTH))-17.5)

    def find_closest_cluster(self, boxes):
        if not boxes:
            return None, None, None

        detections = sorted(
            [(self.estimate_distance(b), self.estimate_angle(b)) for b in boxes],
            key=lambda x: x[0]
        )

        closest_dist = detections[0][0]
        cluster = [d for d in detections if d[0] - closest_dist < 20.0]

        avg_angle = float(np.mean([d[1] for d in cluster]))
        min_dist = float(cluster[0][0])
        count = len(cluster)

        return avg_angle, min_dist, count

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(
            frame,
            verbose=False,
            device='cuda',
            half=True,
            classes=[self.target_class],
            conf=0.01,
            imgsz=320,
            stream=True
        )

        for r in results:
            boxes = r.boxes.xyxy.cpu().numpy().tolist()
            fps = 1 / (time.time() - self.t0)
            self.t0 = time.time()

            self.get_logger().info(f'FPS: {fps:.1f} | Detections: {len(boxes)}')

            if len(boxes) > 0:
                self.get_logger().info(f'Coordinates: {boxes}')

            # Publish raw detections
            detection_msg = String()
            detection_msg.data = json.dumps({
                'boxes': boxes,
                'fps': round(fps, 1),
                'timestamp': msg.header.stamp.sec
            })
            self.detection_pub.publish(detection_msg)

            # Publish navigation target
            angle, distance, count = self.find_closest_cluster(boxes)
            if angle is not None:
                self.get_logger().info(
                    f'Cluster: {count} ball(s) | Angle: {angle:.1f}° | Distance: {distance:.1f}cm'
                )
                nav_msg = Float32MultiArray()
                nav_msg.data = [angle, distance, float(count)]
                self.navigate_pub.publish(nav_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
