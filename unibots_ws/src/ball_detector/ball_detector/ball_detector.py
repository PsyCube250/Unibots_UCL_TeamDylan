import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import numpy as np

# Camera constants
FOCAL_LENGTH = 500.0
BALL_DIAMETER_CM = 4.0
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

# Orange colour filter in HSV (wider range to catch reddish-orange)
ORANGE_LOW = np.array([3, 80, 80])
ORANGE_HIGH = np.array([30, 255, 255])
ORANGE_RATIO_THRESHOLD = 0.20

WEB_PORT = 8770

_latest_frame = None
_frame_lock = threading.Lock()


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body style="margin:0;background:#000">'
                            b'<img src="/stream" style="width:100%;height:auto">'
                            b'</body></html>')
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with _frame_lock:
                    frame = _latest_frame
                if frame is None:
                    time.sleep(0.05)
                    continue
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                try:
                    self.wfile.write(b'--frame\r\n'
                                    b'Content-Type: image/jpeg\r\n\r\n' +
                                    jpeg.tobytes() + b'\r\n')
                except BrokenPipeError:
                    break
                time.sleep(0.1)
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        self.detection_pub = self.create_publisher(String, '/ball_detections', 10)
        self.navigate_pub = self.create_publisher(String, '/ball_navigate', 10)

        self.target_class = 32

        self.get_logger().info('Opening camera...')
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Camera opened: {actual_w}x{actual_h}')

        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('/home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan/Testing_Code/Sensors/yolo26n.pt')

        self.get_logger().info('Warming up model...')
        dummy = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
        self.model.predict(dummy, device='cuda', half=True, verbose=False, imgsz=320)

        self.t0 = time.time()

        server = HTTPServer(('0.0.0.0', WEB_PORT), MJPEGHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Live view: http://0.0.0.0:{WEB_PORT}')
        self.get_logger().info('Ball detector ready!')

        self.timer = self.create_timer(1.0 / 10.0, self.detect_callback)

    def is_orange(self, frame, box):
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return False
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)
        ratio = np.count_nonzero(mask) / mask.size
        return ratio >= ORANGE_RATIO_THRESHOLD

    def is_ball_shaped(self, box):
        w = box[2] - box[0]
        h = box[3] - box[1]
        if w < 10 or h < 10:
            return False
        if w > IMAGE_WIDTH * 0.4 or h > IMAGE_HEIGHT * 0.4:
            return False
        aspect = w / h
        return 0.6 < aspect < 1.7

    def estimate_distance(self, box):
        box_height = box[3] - box[1]
        if box_height <= 0:
            return float('inf')
        return (BALL_DIAMETER_CM * FOCAL_LENGTH) / box_height

    def estimate_angle(self, box):
        cx = (box[0] + box[2]) / 2.0
        offset = cx - (IMAGE_WIDTH / 2.0)
        return float(np.degrees(np.arctan(offset / FOCAL_LENGTH)))

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

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, -1)

        results = self.model.predict(
            frame,
            verbose=False,
            device='cuda',
            half=True,
            conf=0.10,
            imgsz=640,
            stream=True
        )

        for r in results:
            global _latest_frame
            all_boxes = r.boxes.xyxy.cpu().numpy().tolist()
            boxes = [b for b in all_boxes if self.is_orange(frame, b) and self.is_ball_shaped(b)]
            fps = 1 / (time.time() - self.t0)
            self.t0 = time.time()

            self.get_logger().info(f'FPS: {fps:.1f} | Orange balls: {len(boxes)}/{len(all_boxes)}')

            if len(boxes) > 0:
                self.get_logger().info(f'Coordinates: {boxes}')

            # Draw overlays for live view
            display = frame.copy()
            for b in boxes:
                x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                dist = self.estimate_distance(b)
                ang = self.estimate_angle(b)
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 140, 255), 2)
                cv2.putText(display, f'{dist:.0f}cm {ang:+.0f}deg',
                            (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)
            cv2.putText(display, f'FPS: {fps:.1f} | Balls: {len(boxes)}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            with _frame_lock:
                _latest_frame = display

            # Publish raw detections
            detection_msg = String()
            detection_msg.data = json.dumps({
                'boxes': boxes,
                'fps': round(fps, 1),
                'timestamp': time.time()
            })
            self.detection_pub.publish(detection_msg)

            # Publish navigation target
            angle, distance, count = self.find_closest_cluster(boxes)
            if angle is not None:
                self.get_logger().info(
                    f'Cluster: {count} ball(s) | Angle: {angle:.1f}° | Distance: {distance:.1f}cm'
                )
                nav_msg = String()
                nav_msg.data = json.dumps({
                    'angle': round(angle, 1),
                    'distance_cm': round(distance, 1),
                    'count': count
                })
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
