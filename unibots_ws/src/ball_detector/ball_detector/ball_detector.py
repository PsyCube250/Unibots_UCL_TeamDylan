import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


REPO_ROOT = Path('/home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan')
PACKAGE_DIR = REPO_ROOT / 'unibots_ws/src/ball_detector/ball_detector'
MODEL_CANDIDATES = (
    PACKAGE_DIR / 'yolo26n.engine',
    REPO_ROOT / 'Testing_Code/Sensors/yolo26n.engine',
    REPO_ROOT / 'Testing_Code/Sensors/yolo26n.pt',
    PACKAGE_DIR / 'yolo26n.pt',
)

COLOUR_RANGES = {
    'orange': (
        (np.array([3, 80, 80]), np.array([30, 255, 255])),
    ),
    'yellow': (
        (np.array([18, 70, 80]), np.array([42, 255, 255])),
    ),
    'white': (
        (np.array([0, 0, 120]), np.array([179, 95, 255])),
    ),
}

WEB_PORT = 8770
_latest_frame = None
_frame_lock = threading.Lock()


def parse_csv_ints(value):
    if value is None:
        return []
    if isinstance(value, int):
        return [value]
    if isinstance(value, (list, tuple)):
        return [int(part) for part in value]
    text = str(value).strip()
    if not text:
        return []
    return [int(part.strip()) for part in text.split(',') if part.strip()]


def parse_csv_words(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(part).strip().lower() for part in value if str(part).strip()]
    return [part.strip().lower() for part in str(value).split(',') if part.strip()]


def resolve_model_path(value):
    text = str(value).strip()
    if text and text.lower() != 'auto':
        return text
    for path in MODEL_CANDIDATES:
        if path.exists():
            return str(path)
    return 'yolo26n.pt'


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


def clamp_box(box, frame_w, frame_h):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    x1 = max(0, min(frame_w - 1, x1))
    y1 = max(0, min(frame_h - 1, y1))
    x2 = max(x1 + 1, min(frame_w, x2))
    y2 = max(y1 + 1, min(frame_h, y2))
    return x1, y1, x2, y2


def colour_ratios(frame, box):
    frame_h, frame_w = frame.shape[:2]
    x1, y1, x2, y2 = clamp_box(box, frame_w, frame_h)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return {name: 0.0 for name in COLOUR_RANGES}

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    area = float(crop.shape[0] * crop.shape[1])
    ratios = {}
    for name, ranges in COLOUR_RANGES.items():
        mask = None
        for low, high in ranges:
            part = cv2.inRange(hsv, low, high)
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        ratios[name] = float(cv2.countNonZero(mask)) / area
    return ratios


def box_allowed(box, frame_w, frame_h, min_diameter, max_diameter, min_aspect, max_aspect, edge_margin):
    x1, y1, x2, y2 = [float(v) for v in box]
    width = max(1.0, x2 - x1)
    height = max(1.0, y2 - y1)
    diameter = max(width, height)
    if diameter < min_diameter or diameter > max_diameter:
        return False
    if diameter > max(frame_w, frame_h) * 0.40:
        return False
    aspect = width / height
    if aspect < min_aspect or aspect > max_aspect:
        return False
    if edge_margin > 0:
        if x1 <= edge_margin or y1 <= edge_margin:
            return False
        if x2 >= frame_w - edge_margin or y2 >= frame_h - edge_margin:
            return False
    return True


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(
                b'<html><body style="margin:0;background:#000">'
                b'<img src="/stream" style="width:100%;height:auto">'
                b'</body></html>'
            )
            return

        if self.path != '/stream':
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            with _frame_lock:
                frame = None if _latest_frame is None else _latest_frame.copy()
            if frame is None:
                time.sleep(0.05)
                continue
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
            if not ok:
                time.sleep(0.05)
                continue
            try:
                self.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                    + jpeg.tobytes()
                    + b'\r\n'
                )
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(0.08)

    def log_message(self, format, *args):
        pass


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        self.declare_parameter('model_path', 'auto')
        self.declare_parameter('target_classes', '32')
        self.declare_parameter('confidence', 0.08)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('half', True)
        self.declare_parameter('target_colours', 'orange')
        self.declare_parameter('require_colour', True)
        self.declare_parameter('min_colour_ratio', 0.08)
        self.declare_parameter('ball_diameter_cm', 4.0)
        self.declare_parameter('hfov_deg', 120.0)
        self.declare_parameter('min_diameter_px', 6.0)
        self.declare_parameter('max_diameter_px', 180.0)
        self.declare_parameter('min_aspect', 0.50)
        self.declare_parameter('max_aspect', 2.00)
        self.declare_parameter('edge_margin_px', 2)
        self.declare_parameter('cluster_distance_cm', 20.0)
        self.declare_parameter('web_port', WEB_PORT)
        self.declare_parameter('enable_colour_fallback', True)
        self.declare_parameter('fallback_colours', 'orange')
        self.declare_parameter('fallback_min_score', 0.68)
        self.declare_parameter('fallback_min_y_fraction', 0.34)
        self.declare_parameter('fallback_min_radius_px', 4)
        self.declare_parameter('fallback_max_radius_px', 26)
        self.declare_parameter('fallback_hough_param2', 22)
        self.declare_parameter('fallback_max_candidates', 3)
        self.declare_parameter('target_center_bias_cm_per_deg', 0.9)
        self.declare_parameter('target_lock_s', 1.2)
        self.declare_parameter('target_lock_bias_cm_per_deg', 3.0)

        self.model_path = resolve_model_path(self.get_parameter('model_path').value)
        self.target_classes = parse_csv_ints(self.get_parameter('target_classes').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = str(self.get_parameter('device').value)
        self.half = bool(self.get_parameter('half').value)
        self.target_colours = parse_csv_words(self.get_parameter('target_colours').value)
        self.require_colour = bool(self.get_parameter('require_colour').value)
        self.min_colour_ratio = float(self.get_parameter('min_colour_ratio').value)
        self.ball_diameter_cm = float(self.get_parameter('ball_diameter_cm').value)
        self.hfov_deg = float(self.get_parameter('hfov_deg').value)
        self.min_diameter_px = float(self.get_parameter('min_diameter_px').value)
        self.max_diameter_px = float(self.get_parameter('max_diameter_px').value)
        self.min_aspect = float(self.get_parameter('min_aspect').value)
        self.max_aspect = float(self.get_parameter('max_aspect').value)
        self.edge_margin_px = int(self.get_parameter('edge_margin_px').value)
        self.cluster_distance_cm = float(self.get_parameter('cluster_distance_cm').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self.enable_colour_fallback = bool(self.get_parameter('enable_colour_fallback').value)
        self.fallback_colours = parse_csv_words(self.get_parameter('fallback_colours').value)
        self.fallback_min_score = float(self.get_parameter('fallback_min_score').value)
        self.fallback_min_y_fraction = float(self.get_parameter('fallback_min_y_fraction').value)
        self.fallback_min_radius_px = int(self.get_parameter('fallback_min_radius_px').value)
        self.fallback_max_radius_px = int(self.get_parameter('fallback_max_radius_px').value)
        self.fallback_hough_param2 = int(self.get_parameter('fallback_hough_param2').value)
        self.fallback_max_candidates = int(self.get_parameter('fallback_max_candidates').value)
        self.target_center_bias = float(self.get_parameter('target_center_bias_cm_per_deg').value)
        self.target_lock_s = float(self.get_parameter('target_lock_s').value)
        self.target_lock_bias = float(self.get_parameter('target_lock_bias_cm_per_deg').value)
        self.last_target_angle = None
        self.last_target_distance = None
        self.last_target_s = 0.0

        self.subscription = self.create_subscription(Image, '/img', self.image_callback, 10)
        self.detection_pub = self.create_publisher(String, '/ball_detections', 10)
        self.navigate_pub = self.create_publisher(String, '/ball_navigate', 10)

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(
                'Ultralytics/Torch is not importable. Check the Jetson venv/CUDA libraries before running ball_detector.'
            ) from exc

        self.model = YOLO(self.model_path, task='detect')
        self.t0 = time.time()

        server = ThreadingHTTPServer(('0.0.0.0', self.web_port), MJPEGHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(
            'Ball detector ready: '
            f'classes={self.target_classes or "all"}, colours={self.target_colours}, '
            f'web=http://0.0.0.0:{self.web_port}'
        )

    def focal_px(self, frame_w):
        half_angle = math.radians(self.hfov_deg * 0.5)
        return (frame_w * 0.5) / math.tan(half_angle)

    def estimate_distance_cm(self, box, focal_px):
        box_height = max(1.0, float(box[3]) - float(box[1]))
        return (self.ball_diameter_cm * focal_px) / box_height

    def estimate_angle_deg(self, box, frame_w, focal_px):
        cx = (float(box[0]) + float(box[2])) * 0.5
        offset = cx - (frame_w * 0.5)
        return float(math.degrees(math.atan(offset / focal_px)))

    def run_model(self, frame):
        classes = self.target_classes if self.target_classes else None
        kwargs = {
            'verbose': False,
            'classes': classes,
            'conf': self.confidence,
            'imgsz': self.imgsz,
            'stream': False,
        }
        if self.device.lower() != 'auto':
            kwargs['device'] = self.device
        if self.half:
            kwargs['half'] = True

        results = self.model.predict(frame, **kwargs)
        if not results:
            return []

        boxes_obj = getattr(results[0], 'boxes', None)
        if boxes_obj is None or boxes_obj.xyxy is None:
            return []

        xyxy = boxes_obj.xyxy.cpu().numpy().tolist()
        confs = boxes_obj.conf.cpu().numpy().tolist() if boxes_obj.conf is not None else [0.0] * len(xyxy)
        classes = boxes_obj.cls.cpu().numpy().tolist() if boxes_obj.cls is not None else [-1] * len(xyxy)
        return list(zip(xyxy, confs, classes))

    def classify_colour(self, frame, box):
        ratios = colour_ratios(frame, box)
        accepted = [
            (name, ratios.get(name, 0.0))
            for name in self.target_colours
            if name in ratios and ratios.get(name, 0.0) >= self.min_colour_ratio
        ]
        if not accepted:
            best_name = max(ratios, key=ratios.get)
            return None, ratios, ratios[best_name]
        best_name, best_ratio = max(accepted, key=lambda item: item[1])
        return best_name, ratios, best_ratio

    def filter_boxes(self, frame, raw_boxes):
        frame_h, frame_w = frame.shape[:2]
        filtered = []
        for box, conf, cls_id in raw_boxes:
            if not box_allowed(
                box,
                frame_w,
                frame_h,
                self.min_diameter_px,
                self.max_diameter_px,
                self.min_aspect,
                self.max_aspect,
                self.edge_margin_px,
            ):
                continue

            colour, ratios, best_ratio = self.classify_colour(frame, box)
            if self.require_colour and colour is None:
                continue

            filtered.append({
                'box': [float(v) for v in box],
                'conf': float(conf),
                'class_id': int(cls_id),
                'colour': colour or 'unfiltered',
                'colour_ratio': round(best_ratio, 3),
                'colour_ratios': {name: round(value, 3) for name, value in ratios.items()},
            })
        return filtered

    def fallback_colour_score(self, hsv, inner_mask, outer_mask):
        inner = hsv[inner_mask]
        outer = hsv[outer_mask]
        if inner.size == 0 or outer.size == 0:
            return None

        h = inner[:, 0]
        s = inner[:, 1]
        v = inner[:, 2]
        scores = {}
        if 'orange' in self.fallback_colours:
            scores['orange'] = float(((h >= 3) & (h <= 26) & (s >= 70) & (v >= 45)).mean())
        if 'yellow' in self.fallback_colours:
            scores['yellow'] = float(((h >= 18) & (h <= 44) & (s >= 45) & (v >= 50)).mean())
        if 'white' in self.fallback_colours:
            scores['white'] = float(((s <= 65) & (v >= 145)).mean())
        if not scores:
            return None

        colour, colour_ratio = max(scores.items(), key=lambda item: item[1])
        sat_delta = float(np.mean(inner[:, 1]) - np.mean(outer[:, 1]))
        value_delta = float(np.mean(inner[:, 2]) - np.mean(outer[:, 2]))
        contrast_bonus = max(0.0, sat_delta) / 160.0 + max(0.0, value_delta) / 220.0
        score = colour_ratio + contrast_bonus
        if colour == 'white':
            # White balls are high risk on a white arena. Require stronger local contrast.
            score = colour_ratio + max(0.0, value_delta) / 120.0
        return colour, colour_ratio, score

    def colour_fallback_boxes(self, frame):
        if not self.enable_colour_fallback:
            return []
        frame_h, frame_w = frame.shape[:2]
        y0 = int(max(0.0, min(0.85, self.fallback_min_y_fraction)) * frame_h)
        roi = frame[y0:, :]
        if roi.size == 0:
            return []

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(18, self.fallback_max_radius_px),
            param1=60,
            param2=self.fallback_hough_param2,
            minRadius=self.fallback_min_radius_px,
            maxRadius=self.fallback_max_radius_px,
        )
        if circles is None:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yy, xx = np.ogrid[:frame_h, :frame_w]
        detections = []
        for x_raw, y_raw, r_raw in np.round(circles[0]).astype(int):
            x = int(x_raw)
            y = int(y_raw + y0)
            r = int(r_raw)
            if r <= 0:
                continue
            if x - r < self.edge_margin_px or x + r >= frame_w - self.edge_margin_px:
                continue
            if y - r < y0 or y + r >= frame_h - self.edge_margin_px:
                continue

            dist_sq = (xx - x) ** 2 + (yy - y) ** 2
            inner_mask = dist_sq <= (r * r)
            outer_mask = (dist_sq >= int((r * 1.35) ** 2)) & (dist_sq <= int((r * 2.25) ** 2))
            score_data = self.fallback_colour_score(hsv, inner_mask, outer_mask)
            if score_data is None:
                continue
            colour, colour_ratio, score = score_data
            y_bonus = 0.08 * (y / max(1.0, frame_h))
            center_penalty = 0.12 * abs((x - frame_w * 0.5) / max(1.0, frame_w * 0.5))
            ranked_score = score + y_bonus - center_penalty
            if ranked_score < self.fallback_min_score:
                continue

            x1 = float(x - r)
            y1 = float(y - r)
            x2 = float(x + r)
            y2 = float(y + r)
            if not box_allowed(
                [x1, y1, x2, y2],
                frame_w,
                frame_h,
                self.min_diameter_px,
                self.max_diameter_px,
                self.min_aspect,
                self.max_aspect,
                self.edge_margin_px,
            ):
                continue
            detections.append({
                'box': [x1, y1, x2, y2],
                'conf': round(float(ranked_score), 3),
                'class_id': -1,
                'colour': colour,
                'colour_ratio': round(float(colour_ratio), 3),
                'colour_ratios': {colour: round(float(colour_ratio), 3)},
                'source': 'colour_fallback',
            })

        detections.sort(
            key=lambda det: (
                -float(det['conf']),
                abs(((det['box'][0] + det['box'][2]) * 0.5) - frame_w * 0.5),
            )
        )
        return detections[: max(1, self.fallback_max_candidates)]

    def find_target_cluster(self, detections, frame_w):
        if not detections:
            return None, None, None, None

        focal_px = self.focal_px(frame_w)
        measured = []
        for det in detections:
            box = det['box']
            measured.append((
                self.estimate_distance_cm(box, focal_px),
                self.estimate_angle_deg(box, frame_w, focal_px),
                det,
            ))

        now = time.monotonic()
        if self.last_target_angle is not None and now - self.last_target_s <= self.target_lock_s:
            seed = min(
                measured,
                key=lambda item: (
                    item[0]
                    + abs(item[1] - self.last_target_angle) * self.target_lock_bias
                    - float(item[2].get('conf', 0.0)) * 8.0
                ),
            )
        else:
            seed = min(
                measured,
                key=lambda item: (
                    item[0]
                    + abs(item[1]) * self.target_center_bias
                    - float(item[2].get('conf', 0.0)) * 8.0
                ),
            )
        seed_dist = seed[0]
        seed_angle = seed[1]
        cluster = [
            item for item in measured
            if abs(item[0] - seed_dist) < self.cluster_distance_cm
            and abs(item[1] - seed_angle) < 18.0
        ]
        avg_angle = float(np.mean([item[1] for item in cluster]))
        target = min(cluster, key=lambda item: item[0])[2]
        return avg_angle, float(seed_dist), len(cluster), target

    def collect_geometry(self, target, frame_w, frame_h):
        if target is None:
            return {}
        x1, y1, x2, y2 = target['box']
        cx = (float(x1) + float(x2)) * 0.5
        cy = (float(y1) + float(y2)) * 0.5
        diameter = max(float(x2) - float(x1), float(y2) - float(y1))
        center_error = (cx - frame_w * 0.5) / max(1.0, frame_w)
        bottom_fraction = float(y2) / max(1.0, frame_h)
        low_center = cy >= frame_h * 0.72
        low_edge = y2 >= frame_h * 0.88
        bottom_centered = abs(cx - frame_w * 0.5) <= frame_w * 0.18
        return {
            'target_box': [round(float(v), 1) for v in target['box']],
            'target_cx': round(cx, 1),
            'target_cy': round(cy, 1),
            'diameter_px': round(diameter, 1),
            'frame_width': int(frame_w),
            'frame_height': int(frame_h),
            'center_error_fraction': round(center_error, 4),
            'bottom_fraction': round(bottom_fraction, 4),
            'bottom_centered': bool(bottom_centered),
            'low_center': bool(low_center),
            'low_edge': bool(low_edge),
            'collect_candidate': bool(bottom_centered and low_center and low_edge),
        }

    def draw_overlay(self, frame, detections, fps):
        display = frame.copy()
        focal_px = self.focal_px(display.shape[1])
        colours = {
            'orange': (0, 140, 255),
            'yellow': (0, 230, 255),
            'white': (255, 255, 255),
            'unfiltered': (180, 180, 180),
        }
        for det in detections:
            box = det['box']
            x1, y1, x2, y2 = clamp_box(box, display.shape[1], display.shape[0])
            colour = det.get('colour', 'unfiltered')
            bgr = colours.get(colour, (0, 255, 0))
            distance = self.estimate_distance_cm(box, focal_px)
            angle = self.estimate_angle_deg(box, display.shape[1], focal_px)
            cv2.rectangle(display, (x1, y1), (x2, y2), bgr, 2)
            cv2.putText(
                display,
                f'{colour} {det["conf"]:.2f} {angle:+.0f}deg {distance:.0f}cm',
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                bgr,
                2,
            )
        cv2.putText(
            display,
            f'FPS {fps:.1f} balls {len(detections)}',
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
        )
        return display

    def image_callback(self, msg):
        try:
            frame = image_msg_to_bgr(msg)
            raw_boxes = self.run_model(frame)
            detections = self.filter_boxes(frame, raw_boxes)
            fallback_count = 0
            if not detections:
                fallback = self.colour_fallback_boxes(frame)
                fallback_count = len(fallback)
                detections = fallback
        except Exception as exc:
            self.get_logger().error(f'Ball detection failed: {exc}')
            return

        fps = 1.0 / max(1e-6, time.time() - self.t0)
        self.t0 = time.time()

        display = self.draw_overlay(frame, detections, fps)
        global _latest_frame
        with _frame_lock:
            _latest_frame = display

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        detection_msg = String()
        detection_msg.data = json.dumps({
            'boxes': [det['box'] for det in detections],
            'detections': detections,
            'raw_count': len(raw_boxes),
            'fallback_count': fallback_count,
            'count': len(detections),
            'fps': round(fps, 1),
            'timestamp': stamp if stamp > 0 else time.time(),
        })
        self.detection_pub.publish(detection_msg)

        angle, distance, count, target = self.find_target_cluster(detections, msg.width)
        if angle is None:
            self.get_logger().info(f'FPS: {fps:.1f} | raw={len(raw_boxes)} | filtered=0')
            nav_msg = String()
            nav_msg.data = json.dumps({
                'visible': False,
                'count': 0,
                'raw_count': len(raw_boxes),
                'fallback_count': fallback_count,
                'source': 'yolo_colour',
                'timestamp': stamp if stamp > 0 else time.time(),
            })
            self.navigate_pub.publish(nav_msg)
            return

        self.get_logger().info(
            f'Ball: {target.get("colour")} class={target.get("class_id")} '
            f'angle={angle:.1f}deg distance={distance:.1f}cm count={count}'
        )
        self.last_target_angle = float(angle)
        self.last_target_distance = float(distance)
        self.last_target_s = time.monotonic()
        nav_msg = String()
        payload = {
            'visible': True,
            'angle': round(angle, 1),
            'distance_cm': round(distance, 1),
            'count': count,
            'colour': target.get('colour'),
            'class_id': target.get('class_id'),
            'source': target.get('source', 'yolo_colour'),
        }
        payload.update(self.collect_geometry(target, msg.width, msg.height))
        nav_msg.data = json.dumps(payload)
        self.navigate_pub.publish(nav_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
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
