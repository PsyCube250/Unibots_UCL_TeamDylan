#!/usr/bin/env python3
"""
Ping-pong ball tracker for the Jetson USB camera.

Default mode is camera/detector/monitor only. It does not open the STM32 port
and does not send motor commands unless both --live-motors and --ground-test
are supplied.

The detector is YOLO-first, with an optional yellow/white circular blob fallback
because a stock COCO YOLO model usually has "sports ball" rather than a
specific ping-pong-ball class.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Iterable, Optional

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

STM32_PORT = os.environ.get("UNIBOTS_STM32_PORT", "/dev/ttyTHS1")
STM32_BAUD = 115200
LIDAR_PORT = os.environ.get(
    "UNIBOTS_LIDAR_PORT",
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
)
if not os.path.exists(LIDAR_PORT):
    LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 921600


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Ping-Pong Tracker</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0a0f1f;
      color: #e5e7eb;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 360px;
      background: #0a0f1f;
    }
    main {
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 14px;
      background: #0b1120;
    }
    img {
      width: min(100%, calc(100vh * 1.34));
      max-height: calc(100vh - 28px);
      object-fit: contain;
      border: 1px solid #1f2937;
      background: #020617;
    }
    aside {
      min-height: 100vh;
      border-left: 1px solid #1f2937;
      background: #111827;
      padding: 16px;
    }
    h1 {
      margin: 0 0 14px;
      font-size: 18px;
      font-weight: 650;
    }
    .row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      margin-bottom: 12px;
    }
    .status {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border: 1px solid #374151;
      padding: 6px 9px;
      font-size: 13px;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 999px;
      background: #ef4444;
    }
    .dot.ok { background: #22c55e; }
    button {
      appearance: none;
      border: 1px solid #7f1d1d;
      background: #991b1b;
      color: #fff;
      padding: 8px 11px;
      font: inherit;
      cursor: pointer;
    }
    button:active { transform: translateY(1px); }
    dl {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px 12px;
      margin: 14px 0 0;
      font-size: 14px;
    }
    dt { color: #9ca3af; }
    dd {
      margin: 0;
      font-variant-numeric: tabular-nums;
      text-align: right;
      max-width: 210px;
      overflow-wrap: anywhere;
    }
    .action {
      font-size: 24px;
      font-weight: 750;
      margin: 12px 0 2px;
      line-height: 1.1;
    }
    .reason {
      min-height: 54px;
      color: #cbd5e1;
      font-size: 13px;
      line-height: 1.45;
    }
    .bad { color: #ef4444; }
    .warn { color: #f59e0b; }
    @media (max-width: 860px) {
      body { grid-template-columns: 1fr; }
      aside {
        min-height: auto;
        border-left: 0;
        border-top: 1px solid #1f2937;
      }
      main { min-height: auto; }
    }
  </style>
</head>
<body>
  <main><img id="frame" alt="camera frame"></main>
  <aside>
    <h1>Ping-Pong Tracker</h1>
    <div class="row">
      <div class="status"><span id="dot" class="dot"></span><span id="status">connecting</span></div>
      <button id="stop">STOP</button>
    </div>
    <div id="action" class="action">--</div>
    <div id="reason" class="reason">--</div>
    <dl>
      <dt>Mode</dt><dd id="mode">--</dd>
      <dt>Runtime</dt><dd id="runtime">--</dd>
      <dt>Balls</dt><dd id="balls">--</dd>
      <dt>Detections</dt><dd id="detections">--</dd>
      <dt>Target</dt><dd id="target">--</dd>
      <dt>Distance</dt><dd id="distance">--</dd>
      <dt>Angle</dt><dd id="angle">--</dd>
      <dt>LiDAR front</dt><dd id="lidar">--</dd>
      <dt>Safety</dt><dd id="safety">--</dd>
      <dt>PWM</dt><dd id="pwm">--</dd>
      <dt>Last STOP</dt><dd id="lastStop">--</dd>
    </dl>
  </aside>
<script>
const el = Object.fromEntries([
  "frame", "dot", "status", "action", "reason", "mode", "runtime", "balls",
  "detections", "target", "distance", "angle", "lidar", "safety", "pwm",
  "lastStop", "stop"
].map(id => [id, document.getElementById(id)]));

function fmtM(value) {
  return value === null || value === undefined ? "--" : `${value.toFixed(2)} m`;
}

function fmtDeg(value) {
  return value === null || value === undefined ? "--" : `${value.toFixed(1)} deg`;
}

async function update() {
  try {
    const res = await fetch("/state", { cache: "no-store" });
    const data = await res.json();
    el.dot.classList.toggle("ok", data.live_camera && !data.stop_requested);
    el.status.textContent = data.live_camera ? "live" : "waiting";
    el.action.textContent = data.action || "--";
    el.reason.textContent = data.reason || "--";
    el.mode.textContent = data.live_motors ? "LIVE" : "monitor";
    el.runtime.textContent = `${data.runtime_s.toFixed(1)} s`;
    el.balls.textContent = `${data.collected}/${data.target_count}`;
    el.detections.textContent = `${data.detection_count}`;
    el.target.textContent = data.target_label || "--";
    el.distance.textContent = fmtM(data.target_distance_m);
    el.angle.textContent = fmtDeg(data.target_angle_deg);
    el.lidar.textContent = fmtM(data.front_min_m);
    el.safety.textContent = data.safety || "--";
    el.pwm.textContent = `[${data.pwm.join(", ")}]`;
    el.lastStop.textContent = data.last_stop || "--";
    el.frame.src = `/frame.jpg?t=${Date.now()}`;
  } catch (e) {
    el.dot.classList.remove("ok");
    el.status.textContent = "offline";
  }
}

el.stop.addEventListener("click", () => fetch("/stop", { method: "POST" }));
setInterval(update, 160);
update();
</script>
</body>
</html>
"""


@dataclass
class Detection:
    source: str
    label: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float
    color: str = ""

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) * 0.5

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) * 0.5

    @property
    def width(self) -> float:
        return max(1.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(1.0, self.y2 - self.y1)

    @property
    def diameter_px(self) -> float:
        return max(self.width, self.height)

    @property
    def area(self) -> float:
        return self.width * self.height

    def to_json(self, frame_w: int, focal_px: float, ball_diameter_m: float) -> dict:
        return {
            "source": self.source,
            "label": self.label,
            "conf": round(self.conf, 3),
            "color": self.color,
            "box": [round(self.x1), round(self.y1), round(self.x2), round(self.y2)],
            "angle_deg": round(estimate_angle_deg(self.cx, frame_w, focal_px), 2),
            "distance_m": round(estimate_distance_m(self.diameter_px, focal_px, ball_diameter_m), 3),
            "diameter_px": round(self.diameter_px, 1),
        }


@dataclass
class LidarSnapshot:
    ok: bool = False
    age_s: float = 999.0
    front_min_m: Optional[float] = None
    front_left_m: Optional[float] = None
    front_right_m: Optional[float] = None
    point_count: int = 0
    reason: str = "lidar not opened"


@dataclass(frozen=True)
class SectorStats:
    min_m: Optional[float]
    p10_m: Optional[float]
    median_m: Optional[float]
    count: int


@dataclass(frozen=True)
class AngleArc:
    start: float
    end: float

    def contains(self, angle: float) -> bool:
        start = self.start % 360.0
        end = self.end % 360.0
        angle = angle % 360.0
        if start <= end:
            return start <= angle <= end
        return angle >= start or angle <= end


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    stop_event: threading.Event = field(default_factory=threading.Event)
    latest: dict = field(default_factory=dict)
    jpeg: bytes = b""
    last_stop_reason: str = ""

    def update(self, data: dict, jpeg: Optional[bytes] = None) -> None:
        with self.lock:
            self.latest = data
            if jpeg is not None:
                self.jpeg = jpeg

    def data(self) -> dict:
        with self.lock:
            return dict(self.latest)

    def frame(self) -> bytes:
        with self.lock:
            return bytes(self.jpeg)


@dataclass
class MotionSegment:
    pwm: list[int]
    duration_s: float


@dataclass
class MissionMemory:
    state: str = "SEARCH"
    collected: int = 0
    collect_since_s: float = 0.0
    last_target_s: float = 0.0
    last_command_s: float = 0.0
    last_pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    active_segment_pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    active_segment_since_s: float = 0.0
    history: list[MotionSegment] = field(default_factory=list)
    return_plan: list[MotionSegment] = field(default_factory=list)
    return_index: int = 0
    return_segment_until_s: float = 0.0
    finished: bool = False


def clamp(value: int, limit: int) -> int:
    return max(-limit, min(limit, int(round(value))))


def parse_points(raw: bytes) -> list[tuple[float, float, int]]:
    points = []
    i = 0
    while i <= len(raw) - 47:
        if raw[i] == 0x54 and raw[i + 1] == 0x2C:
            frame = raw[i : i + 47]
            start = struct.unpack_from("<H", frame, 4)[0] / 100.0
            end = struct.unpack_from("<H", frame, 42)[0] / 100.0
            delta = (end - start) % 360.0
            step = delta / 11.0 if delta <= 180.0 else -(360.0 - delta) / 11.0
            for p in range(12):
                off = 6 + p * 3
                dist_mm = struct.unpack_from("<H", frame, off)[0]
                confidence = frame[off + 2]
                angle = (start + step * p) % 360.0
                if 30 <= dist_mm <= 25000 and confidence >= 30:
                    points.append((angle, dist_mm / 1000.0, confidence))
            i += 47
        else:
            i += 1
    return points


def robot_angle(raw_angle: float, front_center: float) -> float:
    return (raw_angle - front_center) % 360.0


def in_arc(angle: float, center: float, half_width: float) -> bool:
    diff = ((angle - center + 180.0) % 360.0) - 180.0
    return abs(diff) <= half_width


def parse_angle_arc(text: str) -> AngleArc:
    parts = text.replace(",", ":").split(":")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected START:END degrees, for example 140:175")
    try:
        start = float(parts[0])
        end = float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("arc degrees must be numbers") from exc
    return AngleArc(start, end)


def filter_points(
    points: list[tuple[float, float, int]],
    front_center: float,
    ignore_raw_arcs: list[AngleArc],
    ignore_robot_arcs: list[AngleArc],
) -> list[tuple[float, float, int]]:
    filtered = []
    for raw_angle, dist, confidence in points:
        robot = robot_angle(raw_angle, front_center)
        if any(arc.contains(raw_angle) for arc in ignore_raw_arcs):
            continue
        if any(arc.contains(robot) for arc in ignore_robot_arcs):
            continue
        filtered.append((raw_angle, dist, confidence))
    return filtered


def percentile(sorted_values: list[float], pct: float) -> Optional[float]:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * pct / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return sorted_values[lo]
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (pos - lo)


def sector_stats(
    points: list[tuple[float, float, int]],
    front_center: float,
    center: float,
    half_width: float,
) -> SectorStats:
    values = sorted(
        dist
        for raw_angle, dist, _ in points
        if in_arc(robot_angle(raw_angle, front_center), center, half_width)
    )
    return SectorStats(
        min_m=values[0] if values else None,
        p10_m=percentile(values, 10.0),
        median_m=percentile(values, 50.0),
        count=len(values),
    )


def choose(stats: SectorStats, decision_stat: str) -> Optional[float]:
    return {
        "min": stats.min_m,
        "p10": stats.p10_m,
        "median": stats.median_m,
    }[decision_stat]


def mix_pwm(forward: int, turn_right: int, limit: int) -> list[int]:
    return [
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
    ]


def invert_pwm(pwm: Iterable[int], limit: int) -> list[int]:
    return [clamp(-v, limit) for v in pwm]


def estimate_angle_deg(cx: float, frame_w: int, focal_px: float) -> float:
    return math.degrees(math.atan2(cx - frame_w * 0.5, focal_px))


def estimate_distance_m(diameter_px: float, focal_px: float, ball_diameter_m: float) -> Optional[float]:
    if diameter_px <= 1.0:
        return None
    return ball_diameter_m * focal_px / diameter_px


def focal_from_hfov(width_px: int, hfov_deg: float) -> float:
    hfov_rad = math.radians(max(20.0, min(170.0, hfov_deg)))
    return (width_px * 0.5) / math.tan(hfov_rad * 0.5)


def parse_csv_set(text: str) -> set[str]:
    return {part.strip().lower() for part in text.split(",") if part.strip()}


def normalize_label(text: str) -> str:
    return text.lower().replace("_", " ").replace("-", " ").strip()


def label_allowed(label: str, allowed_names: set[str]) -> bool:
    if not allowed_names:
        return True
    label_norm = normalize_label(label)
    allowed_norm = {normalize_label(name) for name in allowed_names}
    return any(name == label_norm or name in label_norm or label_norm in name for name in allowed_norm)


def parse_int_set(text: str) -> set[int]:
    out = set()
    for part in text.split(","):
        part = part.strip()
        if part:
            out.add(int(part))
    return out


def camera_arg(value: str) -> int | str:
    return int(value) if value.isdigit() else value


def resolve_model_path(text: str) -> str:
    if text != "auto":
        return text
    env_model = os.environ.get("UNIBOTS_PINGPONG_MODEL")
    if env_model:
        return env_model
    repo_root = Path(__file__).resolve().parents[1]
    engine = repo_root / "unibots_ws" / "src" / "ball_detector" / "ball_detector" / "yolo26n.engine"
    if engine.exists():
        return str(engine)
    return "yolov8n.pt"


def load_yolo(model_path: str):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("Missing ultralytics. Use --detector color or install ultralytics.") from exc
    return YOLO(model_path)


def require_vision_modules() -> None:
    if cv2 is None or np is None:
        raise RuntimeError("Missing OpenCV/numpy. On Jetson install python3-opencv and numpy.")


def run_yolo(model, frame, args: argparse.Namespace) -> list[Detection]:
    require_vision_modules()
    kwargs = {
        "verbose": False,
        "conf": args.yolo_conf,
        "imgsz": args.imgsz,
    }
    if args.device != "auto":
        kwargs["device"] = args.device
    if args.half:
        kwargs["half"] = True

    results = model.predict(frame, **kwargs)
    if not results:
        return []

    names = getattr(model, "names", {}) or {}
    class_ids = parse_int_set(args.yolo_class_ids)
    allowed_names = parse_csv_set(args.yolo_class_names)
    detections: list[Detection] = []

    boxes = getattr(results[0], "boxes", None)
    if boxes is None:
        return detections

    xyxy = boxes.xyxy.cpu().numpy() if boxes.xyxy is not None else []
    confs = boxes.conf.cpu().numpy() if boxes.conf is not None else np.zeros(len(xyxy))
    classes = boxes.cls.cpu().numpy() if boxes.cls is not None else np.full(len(xyxy), -1)

    for box, conf, cls_value in zip(xyxy, confs, classes):
        cls_id = int(cls_value)
        label = str(names.get(cls_id, cls_id)).lower()
        if class_ids and cls_id not in class_ids:
            continue
        if not args.accept_any_yolo_class and not class_ids:
            if not label_allowed(label, allowed_names):
                continue
        x1, y1, x2, y2 = [float(v) for v in box]
        color, ratio = classify_ball_color(frame, x1, y1, x2, y2)
        if args.require_ball_color and ratio < args.min_color_ratio:
            continue
        detections.append(Detection("yolo", label, float(conf), x1, y1, x2, y2, color))
    return detections


def classify_ball_color(frame, x1: float, y1: float, x2: float, y2: float) -> tuple[str, float]:
    h, w = frame.shape[:2]
    ix1 = max(0, min(w - 1, int(x1)))
    iy1 = max(0, min(h - 1, int(y1)))
    ix2 = max(ix1 + 1, min(w, int(x2)))
    iy2 = max(iy1 + 1, min(h, int(y2)))
    crop = frame[iy1:iy2, ix1:ix2]
    if crop.size == 0:
        return "", 0.0
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array([18, 55, 80]), np.array([40, 255, 255]))
    white = cv2.inRange(hsv, np.array([0, 0, 145]), np.array([179, 78, 255]))
    total = float(crop.shape[0] * crop.shape[1])
    yellow_ratio = float(cv2.countNonZero(yellow)) / total
    white_ratio = float(cv2.countNonZero(white)) / total
    if yellow_ratio >= white_ratio:
        return "yellow", yellow_ratio
    return "white", white_ratio


def color_blob_detections(frame, args: argparse.Namespace) -> list[Detection]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array([18, 55, 80]), np.array([40, 255, 255]))
    white = cv2.inRange(hsv, np.array([0, 0, 145]), np.array([179, 70, 255]))
    kernel = np.ones((5, 5), np.uint8)
    masks = {
        "yellow": cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel),
        "white": cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel),
    }
    detections: list[Detection] = []
    frame_area = frame.shape[0] * frame.shape[1]

    for color, mask in masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < args.color_min_area or area > frame_area * args.color_max_area_fraction:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 1.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < args.color_min_circularity:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect = w / max(1.0, h)
            if aspect < 0.55 or aspect > 1.8:
                continue
            fill = area / max(1.0, w * h)
            conf = max(0.05, min(0.95, circularity * 0.55 + fill * 0.45))
            detections.append(
                Detection("color", f"{color} ball", conf, float(x), float(y), float(x + w), float(y + h), color)
            )
    return detections


def find_target(detections: list[Detection], frame_w: int, frame_h: int) -> Optional[Detection]:
    if not detections:
        return None
    center_x = frame_w * 0.5

    def score(det: Detection) -> float:
        center_penalty = abs(det.cx - center_x) / max(1.0, frame_w)
        lower_bonus = det.cy / max(1.0, frame_h)
        return det.area * (1.0 + 0.15 * lower_bonus) * (1.0 - 0.20 * center_penalty)

    return max(detections, key=score)


def open_camera(args: argparse.Namespace):
    cap = cv2.VideoCapture(camera_arg(args.camera), cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {args.camera}")
    return cap


def send_line(ser, line: str) -> None:
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()


def drain_lines(ser, seconds: float) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = ser.readline()
        if raw:
            lines.append(raw.decode("ascii", errors="replace").strip())
    return lines


def preflight_stm32(ser, args: argparse.Namespace) -> None:
    ser.reset_input_buffer()
    for command in ("STOP", "ENABLE 0", "PING", "STATUS", f"LIMIT {args.limit}", f"TELEM {args.telemetry_hz}"):
        send_line(ser, command)
        lines = drain_lines(ser, 0.35)
        if command == "PING" and not any(line.startswith("OK PONG") for line in lines):
            raise RuntimeError("STM32 did not answer PING")
        if command.startswith("LIMIT") and not any(line.startswith(f"OK LIMIT {args.limit}") for line in lines):
            raise RuntimeError(f"STM32 did not accept LIMIT {args.limit}")


def stop_and_disable(stm) -> None:
    if stm is None:
        return
    try:
        send_line(stm, "STOP")
        send_line(stm, "ENABLE 0")
    except Exception:
        pass


def send_motion(stm, action: str, pwm: list[int], live: bool) -> None:
    if not live:
        return
    if action == "STOP":
        send_line(stm, "STOP")
    else:
        send_line(stm, "PWM " + " ".join(str(v) for v in pwm))
    drain_lines(stm, 0.02)


class LidarReader:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.ser = None
        self.last_scan_s = 0.0
        self.last_snapshot = LidarSnapshot(reason="lidar disabled")

    def open(self) -> None:
        if self.args.no_lidar:
            return
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("Missing pyserial for LiDAR safety") from exc
        self.ser = serial.Serial(self.args.lidar_port, self.args.lidar_baud, timeout=0.01)
        self.last_snapshot = LidarSnapshot(reason="waiting for lidar scan")

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def read(self) -> LidarSnapshot:
        if self.ser is None:
            return self.last_snapshot
        raw = self.ser.read(4096)
        if not raw:
            age = time.monotonic() - self.last_scan_s if self.last_scan_s else 999.0
            self.last_snapshot.age_s = age
            self.last_snapshot.ok = age <= self.args.lidar_stale_s
            if not self.last_snapshot.ok:
                self.last_snapshot.reason = f"lidar stale {age:.2f}s"
            return self.last_snapshot

        points = filter_points(
            parse_points(raw),
            self.args.front_center,
            self.args.ignore_raw_arc,
            self.args.ignore_robot_arc,
        )
        now = time.monotonic()
        if points:
            self.last_scan_s = now
        age = now - self.last_scan_s if self.last_scan_s else 999.0

        front = sector_stats(points, self.args.front_center, 0.0, self.args.front_width / 2.0)
        front_left = sector_stats(points, self.args.front_center, 315.0, self.args.diagonal_width / 2.0)
        front_right = sector_stats(points, self.args.front_center, 45.0, self.args.diagonal_width / 2.0)
        front_values = [
            choose(front, self.args.front_stat),
            choose(front_left, self.args.front_stat),
            choose(front_right, self.args.front_stat),
        ]
        present = [v for v in front_values if v is not None]
        front_min = min(present) if present else None
        ok = age <= self.args.lidar_stale_s and front_min is not None
        reason = "ok" if ok else ("no front lidar data" if front_min is None else f"lidar stale {age:.2f}s")
        self.last_snapshot = LidarSnapshot(
            ok=ok,
            age_s=age,
            front_min_m=front_min,
            front_left_m=choose(front_left, self.args.front_stat),
            front_right_m=choose(front_right, self.args.front_stat),
            point_count=len(points),
            reason=reason,
        )
        return self.last_snapshot


def lidar_allows_motion(snapshot: LidarSnapshot, args: argparse.Namespace, live: bool) -> tuple[bool, str, float]:
    if args.no_lidar:
        return True, "lidar disabled", 1.0
    if not snapshot.ok:
        if live or args.require_lidar_in_dry_run:
            return False, snapshot.reason, 0.0
        return True, snapshot.reason, 1.0
    front = snapshot.front_min_m
    if front is None:
        return False, "no front lidar data", 0.0
    if front < args.lidar_stop_m:
        return False, f"front {front:.2f}m < stop {args.lidar_stop_m:.2f}m", 0.0
    if front < args.lidar_slow_m:
        scale = max(0.25, (front - args.lidar_stop_m) / max(0.01, args.lidar_slow_m - args.lidar_stop_m))
        return True, f"front caution {front:.2f}m", scale
    return True, "clear", 1.0


def close_active_segment(mem: MissionMemory, now: float, min_duration_s: float = 0.05) -> None:
    if mem.active_segment_since_s <= 0.0:
        return
    duration = now - mem.active_segment_since_s
    if duration >= min_duration_s and any(mem.active_segment_pwm):
        mem.history.append(MotionSegment(list(mem.active_segment_pwm), duration))
    mem.active_segment_since_s = 0.0
    mem.active_segment_pwm = [0, 0, 0, 0]


def record_motion(mem: MissionMemory, pwm: list[int], now: float) -> None:
    if pwm == mem.active_segment_pwm:
        return
    close_active_segment(mem, now)
    mem.active_segment_pwm = list(pwm)
    mem.active_segment_since_s = now


def build_return_plan(mem: MissionMemory, args: argparse.Namespace, now: float) -> list[MotionSegment]:
    close_active_segment(mem, now)
    plan: list[MotionSegment] = []
    max_segments = max(1, args.return_max_segments)
    for segment in reversed(mem.history[-max_segments:]):
        duration = min(args.return_segment_max_s, max(0.05, segment.duration_s * args.return_duration_scale))
        inverse = invert_pwm(segment.pwm, args.limit)
        if any(inverse):
            plan.append(MotionSegment(inverse, duration))
    return plan


def decide_motion(
    target: Optional[Detection],
    frame_w: int,
    focal_px: float,
    lidar: LidarSnapshot,
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
    live: bool,
) -> tuple[str, list[int], str, Optional[float], Optional[float], str]:
    safety_ok, safety_reason, safety_scale = lidar_allows_motion(lidar, args, live)
    if mem.finished:
        return "STOP", [0, 0, 0, 0], "finished", None, None, safety_reason

    if mem.collected >= args.target_count:
        if mem.state != "RETURN_HOME":
            mem.state = "RETURN_HOME"
            mem.return_plan = build_return_plan(mem, args, now)
            mem.return_index = 0
            mem.return_segment_until_s = 0.0

        if not mem.return_plan:
            mem.finished = True
            return "STOP", [0, 0, 0, 0], "no return history; stopped at finish", None, None, safety_reason

        if not safety_ok:
            return "STOP", [0, 0, 0, 0], f"return blocked: {safety_reason}", None, None, safety_reason

        if mem.return_segment_until_s <= now:
            if mem.return_index >= len(mem.return_plan):
                mem.finished = True
                return "STOP", [0, 0, 0, 0], "return replay complete", None, None, safety_reason
            segment = mem.return_plan[mem.return_index]
            mem.return_segment_until_s = now + segment.duration_s
            mem.return_index += 1
        pwm = list(mem.return_plan[mem.return_index - 1].pwm)
        return "RETURN_HOME", pwm, f"open-loop return {mem.return_index}/{len(mem.return_plan)}", None, None, safety_reason

    if target is None:
        mem.state = "SEARCH"
        if not safety_ok:
            return "STOP", [0, 0, 0, 0], f"search blocked: {safety_reason}", None, None, safety_reason
        pwm = mix_pwm(0, args.search_pwm, args.limit)
        return "SEARCH_RIGHT", pwm, "no ball target", None, None, safety_reason

    mem.last_target_s = now
    angle = estimate_angle_deg(target.cx, frame_w, focal_px)
    distance = estimate_distance_m(target.diameter_px, focal_px, args.ball_diameter_m)
    close_by_distance = distance is not None and distance <= args.collect_distance_m
    close_by_size = target.diameter_px >= args.collect_diameter_px
    centered = abs(angle) <= args.center_tolerance_deg

    if centered and (close_by_distance or close_by_size):
        if mem.state != "COLLECT":
            mem.state = "COLLECT"
            mem.collect_since_s = now
        if now - mem.collect_since_s >= args.collect_hold_s:
            mem.collected += 1
            mem.state = "SEARCH"
            mem.collect_since_s = 0.0
            return "STOP", [0, 0, 0, 0], f"ball counted {mem.collected}/{args.target_count}", distance, angle, safety_reason
        return "STOP", [0, 0, 0, 0], "target reached; holding", distance, angle, safety_reason

    mem.state = "APPROACH"
    if not safety_ok:
        return "STOP", [0, 0, 0, 0], f"approach blocked: {safety_reason}", distance, angle, safety_reason

    if abs(angle) >= args.turn_in_place_deg:
        turn = args.turn_pwm if angle > 0.0 else -args.turn_pwm
        pwm = mix_pwm(0, turn, args.limit)
        return "TURN_TO_BALL", pwm, f"target angle {angle:.1f}deg", distance, angle, safety_reason

    forward = args.approach_pwm
    if distance is not None and distance < args.slow_distance_m:
        forward = min(forward, args.creep_pwm)
    forward = clamp(forward * safety_scale, args.limit)
    turn = clamp(angle * args.turn_gain, args.turn_pwm)
    pwm = mix_pwm(forward, turn, args.limit)
    return "APPROACH_BALL", pwm, f"target {target.label} via {target.source}", distance, angle, safety_reason


def draw_overlay(
    frame,
    detections: list[Detection],
    target: Optional[Detection],
    action: str,
    reason: str,
    collected: int,
    args: argparse.Namespace,
    focal_px: float,
) -> bytes:
    out = frame.copy()
    h, w = out.shape[:2]
    cv2.line(out, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
    cv2.line(out, (0, h // 2), (w, h // 2), (50, 50, 50), 1)
    for det in detections:
        color = (0, 220, 255) if det.color == "yellow" else (220, 220, 220)
        if det is target:
            color = (40, 220, 80)
        p1 = (int(det.x1), int(det.y1))
        p2 = (int(det.x2), int(det.y2))
        cv2.rectangle(out, p1, p2, color, 2)
        distance = estimate_distance_m(det.diameter_px, focal_px, args.ball_diameter_m)
        angle = estimate_angle_deg(det.cx, w, focal_px)
        label = f"{det.label} {det.conf:.2f} {angle:.0f}deg"
        if distance is not None:
            label += f" {distance:.2f}m"
        cv2.putText(out, label, (p1[0], max(18, p1[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    cv2.putText(out, f"{action} balls {collected}/{args.target_count}", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (60, 220, 120), 2)
    cv2.putText(out, reason[:80], (12, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
    ok, jpeg = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
    return jpeg.tobytes() if ok else b""


def build_state(
    start_s: float,
    live: bool,
    mem: MissionMemory,
    action: str,
    pwm: list[int],
    reason: str,
    safety: str,
    target: Optional[Detection],
    detections: list[Detection],
    frame_w: int,
    focal_px: float,
    lidar: LidarSnapshot,
    args: argparse.Namespace,
    shared: SharedState,
) -> dict:
    target_json = target.to_json(frame_w, focal_px, args.ball_diameter_m) if target is not None else None
    return {
        "runtime_s": round(time.monotonic() - start_s, 2),
        "live_camera": True,
        "live_motors": live,
        "stop_requested": shared.stop_event.is_set(),
        "state": mem.state,
        "action": action,
        "reason": reason,
        "safety": safety,
        "pwm": pwm,
        "collected": mem.collected,
        "target_count": args.target_count,
        "detection_count": len(detections),
        "detections": [det.to_json(frame_w, focal_px, args.ball_diameter_m) for det in detections[:8]],
        "target_label": target_json["label"] if target_json else None,
        "target_distance_m": target_json["distance_m"] if target_json else None,
        "target_angle_deg": target_json["angle_deg"] if target_json else None,
        "front_min_m": round(lidar.front_min_m, 3) if lidar.front_min_m is not None else None,
        "lidar_age_s": round(lidar.age_s, 3),
        "lidar_points": lidar.point_count,
        "last_stop": shared.last_stop_reason,
    }


def start_server(shared: SharedState, host: str, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/?"):
                body = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path.startswith("/state"):
                body = json.dumps(shared.data()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path.startswith("/frame.jpg"):
                frame = shared.frame()
                if not frame:
                    self.send_error(503, "no frame")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                return
            self.send_error(404)

        def do_POST(self):
            if self.path == "/stop":
                shared.last_stop_reason = "browser stop"
                shared.stop_event.set()
                self.send_response(204)
                self.end_headers()
                return
            self.send_error(404)

    server = ThreadingHTTPServer((host, port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def run(args: argparse.Namespace) -> int:
    require_vision_modules()
    live = args.live_motors and args.ground_test
    if args.live_motors != args.ground_test:
        raise SystemExit("Ground motion requires both --live-motors and --ground-test")
    max_pwm = max(args.approach_pwm + args.turn_pwm, args.creep_pwm + args.turn_pwm, args.search_pwm, args.turn_pwm)
    if max_pwm > args.limit:
        raise SystemExit("PWM values must not exceed --limit")

    shared = SharedState()
    server = start_server(shared, args.host, args.http_port)
    print(f"Ping-pong tracker monitor: http://{args.host}:{args.http_port}")
    if args.host in ("0.0.0.0", "::"):
        print(f"Jetson URL from Mac: http://192.168.55.1:{args.http_port}")

    model = None
    if args.detector in ("yolo", "hybrid"):
        model_path = resolve_model_path(args.model)
        print(f"Loading YOLO model: {model_path}")
        model = load_yolo(model_path)

    cap = None
    stm = None
    lidar_reader = LidarReader(args)
    mem = MissionMemory()
    start_s = time.monotonic()

    try:
        cap = open_camera(args)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        focal_px = focal_from_hfov(actual_w, args.hfov_deg)
        print(f"Camera {args.camera}: {actual_w}x{actual_h} @ {actual_fps:.1f}fps, focal~{focal_px:.1f}px")

        lidar_reader.open()

        if live:
            import serial

            stm = serial.Serial(args.stm32_port, args.stm32_baud, timeout=0.08)
            preflight_stm32(stm, args)
            if not args.no_lidar and not lidar_reader.ser:
                raise RuntimeError("Live motion requires LiDAR safety unless --no-lidar is set")
            send_line(stm, "ENABLE 1")
            drain_lines(stm, 0.25)
            print("LIVE motors enabled. Browser STOP or Ctrl-C stops.")
        else:
            print("Monitor only. Motors not enabled.")

        while not shared.stop_event.is_set():
            if args.duration > 0 and time.monotonic() - start_s >= args.duration:
                shared.last_stop_reason = "duration elapsed"
                break

            ok, frame = cap.read()
            now = time.monotonic()
            if not ok or frame is None:
                action, pwm, reason = "STOP", [0, 0, 0, 0], "camera frame failed"
                send_motion(stm, action, pwm, live)
                time.sleep(0.05)
                continue

            detections: list[Detection] = []
            if model is not None and args.detector in ("yolo", "hybrid"):
                detections.extend(run_yolo(model, frame, args))
            if args.detector in ("color", "hybrid") and (args.detector == "color" or not detections):
                detections.extend(color_blob_detections(frame, args))

            target = find_target(detections, frame.shape[1], frame.shape[0])
            lidar_snapshot = lidar_reader.read()
            action, pwm, reason, distance, angle, safety = decide_motion(
                target,
                frame.shape[1],
                focal_px,
                lidar_snapshot,
                mem,
                args,
                now,
                live,
            )

            if action not in ("STOP", "RETURN_HOME"):
                record_motion(mem, pwm, now)
            elif action == "STOP":
                close_active_segment(mem, now)

            if now - mem.last_command_s >= args.command_period:
                send_motion(stm, action, pwm, live)
                mem.last_command_s = now
                mem.last_pwm = list(pwm)

            jpeg = draw_overlay(frame, detections, target, action, reason, mem.collected, args, focal_px)
            shared.update(
                build_state(
                    start_s,
                    live,
                    mem,
                    action,
                    pwm,
                    reason,
                    safety,
                    target,
                    detections,
                    frame.shape[1],
                    focal_px,
                    lidar_snapshot,
                    args,
                    shared,
                ),
                jpeg,
            )
            target_desc = "none"
            if target is not None:
                target_desc = f"{target.label}/{target.source}"
                if distance is not None and angle is not None:
                    target_desc += f" {distance:.2f}m {angle:.1f}deg"
            print(
                f"t={time.monotonic() - start_s:05.2f}s balls={mem.collected}/{args.target_count} "
                f"det={len(detections)} target={target_desc} action={action} pwm={pwm} {reason}",
                flush=True,
            )
            time.sleep(args.loop_sleep)

    except KeyboardInterrupt:
        shared.last_stop_reason = "keyboard interrupt"
        print("\nInterrupted")
    finally:
        stop_and_disable(stm)
        if stm is not None:
            stm.close()
        lidar_reader.close()
        if cap is not None:
            cap.release()
        shared.last_stop_reason = shared.last_stop_reason or "program exit"
        data = shared.data()
        data["action"] = "STOP"
        data["pwm"] = [0, 0, 0, 0]
        data["live_camera"] = False
        data["last_stop"] = shared.last_stop_reason
        shared.update(data)
        server.shutdown()
        server.server_close()
        print("Sent STOP and ENABLE 0 on exit")

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="/dev/video0", help="OpenCV camera index or /dev/video path")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--hfov-deg", type=float, default=120.0, help="ELP-USBGS1200P01-H120 is roughly 120 deg")
    parser.add_argument("--ball-diameter-m", type=float, default=0.040)
    parser.add_argument("--detector", choices=["yolo", "color", "hybrid"], default="hybrid")
    parser.add_argument("--model", default="auto", help="'auto', a .pt/.engine path, or an Ultralytics model name")
    parser.add_argument("--device", default="auto", help="Ultralytics device, e.g. cuda, 0, cpu, or auto")
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--yolo-conf", type=float, default=0.10)
    parser.add_argument("--yolo-class-ids", default="", help="Optional comma list, e.g. 32 for COCO sports ball")
    parser.add_argument(
        "--yolo-class-names",
        default="sports ball,ball,ping pong ball,ping-pong ball,tennis ball",
        help="Accepted YOLO class names when --yolo-class-ids is empty",
    )
    parser.add_argument("--accept-any-yolo-class", action="store_true")
    parser.add_argument("--require-ball-color", action="store_true")
    parser.add_argument("--min-color-ratio", type=float, default=0.08)
    parser.add_argument("--color-min-area", type=float, default=35.0)
    parser.add_argument("--color-max-area-fraction", type=float, default=0.18)
    parser.add_argument("--color-min-circularity", type=float, default=0.45)
    parser.add_argument("--target-count", type=int, default=3)
    parser.add_argument("--collect-distance-m", type=float, default=0.30)
    parser.add_argument("--collect-diameter-px", type=float, default=94.0)
    parser.add_argument("--collect-hold-s", type=float, default=0.45)
    parser.add_argument("--center-tolerance-deg", type=float, default=6.0)
    parser.add_argument("--turn-in-place-deg", type=float, default=18.0)
    parser.add_argument("--slow-distance-m", type=float, default=0.55)
    parser.add_argument("--approach-pwm", type=int, default=28)
    parser.add_argument("--creep-pwm", type=int, default=18)
    parser.add_argument("--turn-pwm", type=int, default=16)
    parser.add_argument("--search-pwm", type=int, default=14)
    parser.add_argument("--turn-gain", type=float, default=1.0)
    parser.add_argument("--return-duration-scale", type=float, default=0.90)
    parser.add_argument("--return-segment-max-s", type=float, default=1.20)
    parser.add_argument("--return-max-segments", type=int, default=160)
    parser.add_argument("--lidar-port", default=LIDAR_PORT)
    parser.add_argument("--lidar-baud", type=int, default=LIDAR_BAUD)
    parser.add_argument("--front-center", type=float, default=270.0)
    parser.add_argument("--front-width", type=float, default=36.0)
    parser.add_argument("--diagonal-width", type=float, default=42.0)
    parser.add_argument("--front-stat", choices=["min", "p10", "median"], default="p10")
    parser.add_argument("--lidar-stop-m", type=float, default=0.16)
    parser.add_argument("--lidar-slow-m", type=float, default=0.34)
    parser.add_argument("--lidar-stale-s", type=float, default=0.50)
    parser.add_argument("--ignore-raw-arc", action="append", type=parse_angle_arc, default=[])
    parser.add_argument("--ignore-robot-arc", action="append", type=parse_angle_arc, default=[])
    parser.add_argument("--no-lidar", action="store_true", help="Disable LiDAR gating; not recommended for live floor tests")
    parser.add_argument("--require-lidar-in-dry-run", action="store_true")
    parser.add_argument("--stm32-port", default=STM32_PORT)
    parser.add_argument("--stm32-baud", type=int, default=STM32_BAUD)
    parser.add_argument("--telemetry-hz", type=int, default=10)
    parser.add_argument("--limit", type=int, default=45)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8770)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--command-period", type=float, default=0.10)
    parser.add_argument("--loop-sleep", type=float, default=0.02)
    parser.add_argument("--jpeg-quality", type=int, default=82)
    parser.add_argument("--live-motors", action="store_true")
    parser.add_argument("--ground-test", action="store_true")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
