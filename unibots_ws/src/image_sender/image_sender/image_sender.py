import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('rotate_180', True)
        self.declare_parameter('frame_id', 'camera')

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.publisher = self.create_publisher(Image, '/img', 10)

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Camera opened: {actual_w}x{actual_h} @ {actual_fps:.1f}fps, rotate_180={self.rotate_180}'
        )

        self.timer = self.create_timer(1.0 / max(1, self.fps), self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to grab frame')
            return

        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        msg = self.frame_to_image_msg(frame)
        try:
            self.publisher.publish(msg)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f'Failed to publish frame: {exc}')

    def frame_to_image_msg(self, frame):
        frame = np.ascontiguousarray(frame)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * 3)
        msg.data = frame.tobytes()
        return msg

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
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
