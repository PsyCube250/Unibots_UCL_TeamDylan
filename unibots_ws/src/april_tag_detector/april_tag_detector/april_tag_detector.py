import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import dt_apriltags as apriltag

TAG_SIZE = 0.1

FX = 500.0
FY = 500.0
CX = 640.0
CY = 360.0

WALL_MAP = {
    **{i: 'North' for i in range(0, 6)},
    **{i: 'East'  for i in range(6, 12)},
    **{i: 'South' for i in range(12, 18)},
    **{i: 'West'  for i in range(18, 24)},
}

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        self.declare_parameter('own_zone', 'North')

        self.subscription = self.create_subscription(
            Image,
            '/img',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(String, '/apriltag_detections', 10)

        self.bridge = CvBridge()

        self.detector = apriltag.Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )

        self.camera_params = (FX, FY, CX, CY)

        self.get_logger().info('AprilTag detector ready!')

    def estimate_angle(self, cx_tag):
        offset = cx_tag - CX
        return float(-1*(np.degrees(np.arctan(offset / FX))))

    def image_callback(self, msg):
        own_zone = self.get_parameter('own_zone').get_parameter_value().string_value

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=TAG_SIZE
        )

        results = []
        facing_wall = None

        for d in detections:
            tag_id = d.tag_id
            wall = WALL_MAP.get(tag_id, 'Unknown')

            distance_m = float(d.pose_t[2][0])
            distance_cm = distance_m * 100.0

            cx_tag = float(d.center[0])
            angle = self.estimate_angle(cx_tag)

            results.append({
                'tag_id': tag_id,
                'wall': wall,
                'angle': round(angle, 1),
                'distance_cm': round(distance_cm, 1),
                'own_zone': wall == own_zone
            })

            self.get_logger().info(
                f'Tag {tag_id} | Wall: {wall} | Angle: {angle:.1f}° | Dist: {distance_cm:.1f}cm'
            )

        if results:
            closest = min(results, key=lambda x: x['distance_cm'])
            facing_wall = closest['wall']
            self.get_logger().info(f'Facing: {facing_wall}')

        out = String()
        out.data = json.dumps({
            'facing': facing_wall,
            'tags': results,
            'own_zone': own_zone
        })
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
