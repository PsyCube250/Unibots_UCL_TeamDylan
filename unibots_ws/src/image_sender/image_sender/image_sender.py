import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


WIDTH  = 1280
HEIGHT = 720
FPS    = 90


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/img', 10)
        self.bridge    = CvBridge()

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC,          cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,     WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,    HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,             FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,      1)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return

        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'Camera opened: {actual_w}x{actual_h} @ {actual_fps}fps')

        self.timer = self.create_timer(1.0 / FPS, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        msg                  = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'camera'
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
