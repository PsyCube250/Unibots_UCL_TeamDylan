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
import random
import struct
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from typing import Iterable, Optional

from jetson_safety_imu import add_safety_imu_args, open_imu, open_safety_io

try:
    import dt_apriltags as apriltag
    from apriltag_dock_controller import (
        DockMemory as AprilDockMemory,
        LidarSnapshot as AprilDockLidarSnapshot,
        decide_motion as decide_apriltag_dock_motion,
    )
    from apriltag_live_view import (
        choose_target as choose_apriltag_target,
        parse_tag_ids as parse_apriltag_ids,
        tag_to_dict as apriltag_to_dict,
    )
except ImportError:
    apriltag = None
    AprilDockMemory = None
    AprilDockLidarSnapshot = None
    decide_apriltag_dock_motion = None
    choose_apriltag_target = None
    parse_apriltag_ids = None
    apriltag_to_dict = None

try:
    from servo_gate_test import bounded_pulse_us, make_gate, move_servo
except ImportError:
    bounded_pulse_us = None
    make_gate = None
    move_servo = None

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
      <dt>Mission</dt><dd id="mission">--</dd>
      <dt>Balls</dt><dd id="balls">--</dd>
      <dt>Detections</dt><dd id="detections">--</dd>
      <dt>Target</dt><dd id="target">--</dd>
      <dt>Distance</dt><dd id="distance">--</dd>
      <dt>Angle</dt><dd id="angle">--</dd>
      <dt>Home tag</dt><dd id="homeTarget">--</dd>
      <dt>Home dist</dt><dd id="homeDistance">--</dd>
      <dt>Release</dt><dd id="release">--</dd>
      <dt>LiDAR front</dt><dd id="lidar">--</dd>
      <dt>LiDAR left</dt><dd id="lidarLeft">--</dd>
      <dt>LiDAR right</dt><dd id="lidarRight">--</dd>
      <dt>Target clear</dt><dd id="targetClear">--</dd>
      <dt>Wall</dt><dd id="wallState">--</dd>
      <dt>Safety</dt><dd id="safety">--</dd>
      <dt>Kill</dt><dd id="kill">--</dd>
      <dt>LED</dt><dd id="led">--</dd>
      <dt>IMU</dt><dd id="imu">--</dd>
      <dt>Yaw</dt><dd id="yaw">--</dd>
      <dt>Gyro Z</dt><dd id="gyroZ">--</dd>
      <dt>PWM</dt><dd id="pwm">--</dd>
      <dt>Last STOP</dt><dd id="lastStop">--</dd>
    </dl>
  </aside>
<script>
const el = Object.fromEntries([
  "frame", "dot", "status", "action", "reason", "mode", "runtime", "mission", "balls",
  "detections", "target", "distance", "angle", "lidar", "lidarLeft",
  "lidarRight", "targetClear", "wallState", "homeTarget", "homeDistance",
  "release", "safety", "kill", "led", "imu", "yaw", "gyroZ", "pwm", "lastStop", "stop"
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
    el.mission.textContent = data.state || "--";
    el.balls.textContent = `${data.collected}/${data.target_count}`;
    el.detections.textContent = `${data.detection_count}`;
    el.target.textContent = data.target_label || "--";
    el.distance.textContent = fmtM(data.target_distance_m);
    el.angle.textContent = fmtDeg(data.target_angle_deg);
    el.homeTarget.textContent = data.home_target_label || "--";
    el.homeDistance.textContent = fmtM(data.home_target_distance_m);
    el.release.textContent = data.release_status || "--";
    el.lidar.textContent = fmtM(data.front_min_m);
    el.lidarLeft.textContent = fmtM(data.left_m);
    el.lidarRight.textContent = fmtM(data.right_m);
    el.targetClear.textContent = fmtM(data.target_clearance_m);
    el.wallState.textContent = data.wall_state || "--";
    el.safety.textContent = data.safety || "--";
    el.kill.textContent = data.kill_active == null ? "--" : (data.kill_active ? "KILL" : "OK");
    el.led.textContent = data.led || "--";
    el.imu.textContent = data.imu || "--";
    el.yaw.textContent = data.imu_yaw_deg == null ? "--" : `${data.imu_yaw_deg.toFixed(1)} deg`;
    el.gyroZ.textContent = data.imu_gyro_z_rad_s == null ? "--" : `${data.imu_gyro_z_rad_s.toFixed(3)} rad/s`;
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

    def to_json(
        self,
        frame_w: int,
        focal_px: float,
        ball_diameter_m: float,
        angle_model: str = "pinhole",
    ) -> dict:
        distance = estimate_distance_m(self.diameter_px, focal_px, ball_diameter_m)
        return {
            "source": self.source,
            "label": self.label,
            "conf": round(self.conf, 3),
            "color": self.color,
            "box": [round(self.x1), round(self.y1), round(self.x2), round(self.y2)],
            "angle_deg": round(estimate_angle_deg(self.cx, frame_w, focal_px, angle_model), 2),
            "distance_m": round(distance, 3) if distance is not None else None,
            "diameter_px": round(self.diameter_px, 1),
        }


@dataclass
class LidarSnapshot:
    ok: bool = False
    age_s: float = 999.0
    front_min_m: Optional[float] = None
    front_left_m: Optional[float] = None
    front_right_m: Optional[float] = None
    left_m: Optional[float] = None
    right_m: Optional[float] = None
    rear_m: Optional[float] = None
    point_count: int = 0
    reason: str = "lidar not opened"
    front_close_count: int = 0


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
    prev_angle_error_deg: float = 0.0
    last_pd_s: float = 0.0
    wall_state: str = "clear"
    step_action: str = ""
    step_pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    step_until_s: float = 0.0
    step_reason: str = ""
    held_target: Optional[Detection] = None
    held_target_s: float = 0.0
    locked_target_s: float = 0.0
    last_kick_s: float = 0.0
    stuck_last_change_s: float = 0.0
    stuck_last_front_m: Optional[float] = None
    stuck_last_left_m: Optional[float] = None
    stuck_last_right_m: Optional[float] = None
    stuck_last_target_cx: Optional[float] = None
    stuck_last_target_diameter_px: Optional[float] = None
    stuck_last_yaw_deg: Optional[float] = None
    recovery_until_s: float = 0.0
    recovery_action: str = ""
    recovery_pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    recovery_reason: str = ""
    search_yaw_start_deg: Optional[float] = None
    search_turn_dir: int = 1
    search_last_progress_deg: float = 0.0
    search_last_progress_s: float = 0.0
    search_recovery_until_s: float = 0.0
    search_recovery_pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    home_dock: object | None = None
    home_tags: list[dict] = field(default_factory=list)
    home_target: Optional[dict] = None
    home_distractor_count: int = 0
    release_started_s: float = 0.0
    release_done: bool = False
    release_status: str = ""


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
    return (front_center - raw_angle) % 360.0


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


def min_present(values: Iterable[Optional[float]]) -> Optional[float]:
    present = [v for v in values if v is not None]
    return min(present) if present else None


def mix_pwm(forward: int, turn_right: int, limit: int) -> list[int]:
    return [
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
    ]


def mecanum_pwm(
    forward: int | float,
    strafe_right: int | float,
    turn_right: int | float,
    args: argparse.Namespace,
) -> list[int]:
    if args.drive_mode in ("tank", "tank_step"):
        return mix_pwm(clamp(forward, args.limit), clamp(turn_right, args.limit), args.limit)

    f = float(forward)
    s = float(strafe_right) * float(args.strafe_sign)
    t = float(turn_right)
    return [
        clamp(f + s + t, args.limit),  # FL
        clamp(f - s - t, args.limit),  # FR
        clamp(f - s + t, args.limit),  # BL
        clamp(f + s - t, args.limit),  # BR
    ]


def signed_pwm(value: float, minimum: int, limit: int) -> int:
    pwm = clamp(value, limit)
    if pwm == 0:
        return 0
    if abs(pwm) < minimum:
        return minimum if pwm > 0 else -minimum
    return pwm


def clearer_strafe(lidar: LidarSnapshot, args: argparse.Namespace) -> int:
    left = lidar.left_m if lidar.left_m is not None else 0.0
    right = lidar.right_m if lidar.right_m is not None else 0.0
    return args.wall_strafe_pwm if right >= left else -args.wall_strafe_pwm


def invert_pwm(pwm: Iterable[int], limit: int) -> list[int]:
    return [clamp(-v, limit) for v in pwm]


def estimate_angle_deg(cx: float, frame_w: int, focal_px: float, angle_model: str = "pinhole") -> float:
    if angle_model == "linear":
        hfov_deg = math.degrees(2.0 * math.atan2(frame_w * 0.5, focal_px))
        normalized = (cx - frame_w * 0.5) / max(1.0, frame_w * 0.5)
        return normalized * hfov_deg * 0.5
    return math.degrees(math.atan2(cx - frame_w * 0.5, focal_px))


def estimate_distance_m(diameter_px: float, focal_px: float, ball_diameter_m: float) -> Optional[float]:
    if diameter_px <= 1.0:
        return None
    return ball_diameter_m * focal_px / diameter_px


def collect_reached(
    target: Detection,
    frame_w: int,
    frame_h: int,
    angle: float,
    distance: Optional[float],
    args: argparse.Namespace,
) -> tuple[bool, str]:
    close_by_distance = distance is not None and distance <= args.collect_distance_m
    close_by_size = target.diameter_px >= args.collect_diameter_px
    centered = abs(angle) <= args.center_tolerance_deg
    if centered and (close_by_distance or close_by_size):
        reason = "close"
        if close_by_distance:
            reason = f"distance {distance:.2f}m"
        elif close_by_size:
            reason = f"size {target.diameter_px:.0f}px"
        return True, reason

    if args.collect_bottom_enable:
        bottom_centered = abs(target.cx - frame_w * 0.5) <= frame_w * args.collect_bottom_center_fraction
        low_center = target.cy >= frame_h * args.collect_bottom_y_fraction
        low_edge = target.y2 >= frame_h * args.collect_bottom_edge_fraction
        if bottom_centered and low_center and low_edge:
            offset_px = target.cx - frame_w * 0.5
            return True, f"bottom centered offset {offset_px:+.0f}px"

    return False, "not reached"


def focal_from_hfov(width_px: int, hfov_deg: float) -> float:
    hfov_rad = math.radians(max(20.0, min(170.0, hfov_deg)))
    return (width_px * 0.5) / math.tan(hfov_rad * 0.5)


def parse_csv_set(text: str) -> set[str]:
    return {part.strip().lower() for part in text.split(",") if part.strip()}


def parse_signs(text: str) -> list[int]:
    parts = [part for part in text.replace(",", " ").split() if part]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected four signs, e.g. '1 -1 1 -1'")
    values = [int(part) for part in parts]
    if any(value not in (-1, 1) for value in values):
        raise argparse.ArgumentTypeError("motor signs must be -1 or 1")
    return values


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


def load_onnx(model_path: str):
    require_vision_modules()
    return cv2.dnn.readNetFromONNX(model_path)


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
        if not yolo_box_allowed(x1, y1, x2, y2, args, frame.shape[0]):
            continue
        color, ratio = classify_ball_color(frame, x1, y1, x2, y2)
        if args.require_ball_color and ratio < args.min_color_ratio:
            continue
        if not target_color_allowed(color, args):
            continue
        detections.append(Detection("yolo", label, float(conf), x1, y1, x2, y2, color))
    return detections


def run_onnx_yolo(net, frame, args: argparse.Namespace) -> list[Detection]:
    require_vision_modules()
    h, w = frame.shape[:2]
    size = int(args.imgsz)
    blob = cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (size, size), swapRB=True, crop=False)
    net.setInput(blob)
    output = net.forward()
    preds = np.asarray(output)
    if preds.ndim == 3:
        preds = preds[0]
    if preds.ndim == 2 and preds.shape[0] in (5, 6, 84) and preds.shape[1] > preds.shape[0]:
        preds = preds.T

    class_ids = parse_int_set(args.yolo_class_ids)
    allowed_names = parse_csv_set(args.yolo_class_names)
    detections: list[Detection] = []
    boxes: list[list[int]] = []
    confs: list[float] = []
    metas: list[tuple[str, int]] = []
    sx = w / float(size)
    sy = h / float(size)

    for row in preds:
        if len(row) < 5:
            continue
        values = row.astype(float)
        if len(values) == 6:
            a, b, c, d, conf, cls_value = values[:6]
            cls_id = int(round(cls_value))
            if args.onnx_box_format == "xywh" or (
                args.onnx_box_format == "auto" and (c <= a or d <= b)
            ):
                x1, y1, x2, y2 = a - c * 0.5, b - d * 0.5, a + c * 0.5, b + d * 0.5
            else:
                x1, y1, x2, y2 = a, b, c, d
        else:
            x, y, bw, bh = values[:4]
            scores = values[4:]
            cls_id = int(np.argmax(scores))
            conf = float(scores[cls_id])
            x1, y1, x2, y2 = x - bw * 0.5, y - bh * 0.5, x + bw * 0.5, y + bh * 0.5
        if conf < args.yolo_conf:
            continue
        label = "ping pong ball" if cls_id == 0 else f"class {cls_id}"
        if class_ids and cls_id not in class_ids:
            continue
        if not args.accept_any_yolo_class and not class_ids and not label_allowed(label, allowed_names):
            continue
        ix1 = int(max(0, min(w - 1, x1 * sx)))
        iy1 = int(max(0, min(h - 1, y1 * sy)))
        ix2 = int(max(ix1 + 1, min(w, x2 * sx)))
        iy2 = int(max(iy1 + 1, min(h, y2 * sy)))
        if (ix2 - ix1) < 2 or (iy2 - iy1) < 2:
            continue
        if not yolo_box_allowed(ix1, iy1, ix2, iy2, args, h):
            continue
        boxes.append([ix1, iy1, ix2 - ix1, iy2 - iy1])
        confs.append(float(conf))
        metas.append((label, cls_id))

    keep = cv2.dnn.NMSBoxes(boxes, confs, args.yolo_conf, args.onnx_nms_threshold)
    if len(keep) == 0:
        return detections
    for idx in np.array(keep).reshape(-1):
        x, y, bw, bh = boxes[int(idx)]
        label, _cls_id = metas[int(idx)]
        color, ratio = classify_ball_color(frame, x, y, x + bw, y + bh)
        if args.require_ball_color and ratio < args.min_color_ratio:
            continue
        if not target_color_allowed(color, args):
            continue
        detections.append(
            Detection("onnx", label, float(confs[int(idx)]), float(x), float(y), float(x + bw), float(y + bh), color)
        )
    return detections


def target_color_allowed(color: str, args: argparse.Namespace) -> bool:
    allowed_colors = parse_csv_set(args.target_colors)
    return not allowed_colors or color in allowed_colors


def yolo_box_allowed(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    args: argparse.Namespace,
    frame_h: Optional[int] = None,
) -> bool:
    width = max(1.0, float(x2) - float(x1))
    height = max(1.0, float(y2) - float(y1))
    diameter = max(width, height)
    if diameter < args.yolo_min_diameter_px or diameter > args.yolo_max_diameter_px:
        return False
    aspect = width / height
    if aspect < args.yolo_min_aspect or aspect > args.yolo_max_aspect:
        return False
    if frame_h is not None and frame_h > 0:
        cy = (float(y1) + float(y2)) * 0.5
        roi_y1 = max(0.0, min(1.0, args.roi_y_min_fraction)) * frame_h
        roi_y2 = max(0.0, min(1.0, args.roi_y_max_fraction)) * frame_h
        if roi_y2 <= roi_y1:
            roi_y1, roi_y2 = 0.0, float(frame_h)
        if cy < roi_y1 or cy > roi_y2:
            return False
    return True


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
    orange = cv2.inRange(hsv, np.array([5, 70, 70]), np.array([18, 255, 255]))
    yellow = cv2.inRange(hsv, np.array([18, 55, 80]), np.array([40, 255, 255]))
    white = cv2.inRange(hsv, np.array([0, 0, 145]), np.array([179, 78, 255]))
    total = float(crop.shape[0] * crop.shape[1])
    ratios = {
        "orange": float(cv2.countNonZero(orange)) / total,
        "yellow": float(cv2.countNonZero(yellow)) / total,
        "white": float(cv2.countNonZero(white)) / total,
    }
    return max(ratios.items(), key=lambda item: item[1])


def color_blob_detections(frame, args: argparse.Namespace) -> list[Detection]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, np.array([5, 70, 70]), np.array([18, 255, 255]))
    yellow = cv2.inRange(hsv, np.array([18, 55, 80]), np.array([40, 255, 255]))
    white = cv2.inRange(hsv, np.array([0, 0, 145]), np.array([179, 70, 255]))
    kernel = np.ones((5, 5), np.uint8)
    masks = {
        "orange": cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel),
        "yellow": cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel),
        "white": cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel),
    }
    detections: list[Detection] = []
    frame_h, frame_w = frame.shape[:2]
    frame_area = frame_h * frame_w
    roi_y1 = int(max(0.0, min(1.0, args.roi_y_min_fraction)) * frame_h)
    roi_y2 = int(max(0.0, min(1.0, args.roi_y_max_fraction)) * frame_h)
    if roi_y2 <= roi_y1:
        roi_y1, roi_y2 = 0, frame_h

    for color, mask in masks.items():
        if not target_color_allowed(color, args):
            continue
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
            if y < roi_y1 or (y + h) > roi_y2:
                continue
            if (
                x <= args.color_edge_margin_px
                or y <= args.color_edge_margin_px
                or x + w >= frame_w - args.color_edge_margin_px
                or y + h >= frame_h - args.color_edge_margin_px
            ):
                continue
            diameter = max(w, h)
            if diameter < args.color_min_diameter_px or diameter > args.color_max_diameter_px:
                continue
            aspect = w / max(1.0, h)
            if aspect < args.color_min_aspect or aspect > args.color_max_aspect:
                continue
            fill = area / max(1.0, w * h)
            if fill < args.color_min_fill:
                continue
            conf = max(0.05, min(0.95, circularity * 0.55 + fill * 0.45))
            detections.append(
                Detection("color", f"{color} ball", conf, float(x), float(y), float(x + w), float(y + h), color)
            )
    return detections


def target_clearance_m(lidar: LidarSnapshot, angle_deg: Optional[float], args: argparse.Namespace) -> Optional[float]:
    if angle_deg is None:
        return lidar.front_min_m
    if angle_deg <= -args.target_side_angle_deg:
        return min_present((lidar.front_left_m, lidar.left_m))
    if angle_deg >= args.target_side_angle_deg:
        return min_present((lidar.front_right_m, lidar.right_m))
    return lidar.front_min_m


def target_wall_penalty(lidar: LidarSnapshot, angle_deg: Optional[float], args: argparse.Namespace) -> float:
    if angle_deg is None:
        return 0.0
    side_dist = lidar.left_m if angle_deg < 0.0 else lidar.right_m
    if side_dist is None:
        return 0.0
    if side_dist <= args.side_stop_m:
        return args.wall_target_penalty
    if side_dist <= args.side_slow_m:
        span = max(0.01, args.side_slow_m - args.side_stop_m)
        return args.wall_target_penalty * (args.side_slow_m - side_dist) / span
    return 0.0


def find_target(
    detections: list[Detection],
    frame_w: int,
    frame_h: int,
    focal_px: float,
    lidar: LidarSnapshot,
    args: argparse.Namespace,
) -> Optional[Detection]:
    if not detections:
        return None
    center_x = frame_w * 0.5
    preferred = args.prefer_color.strip().lower()

    def score(det: Detection) -> float:
        angle = estimate_angle_deg(det.cx, frame_w, focal_px, args.angle_model)
        clearance = target_clearance_m(lidar, angle, args)
        center_penalty = abs(det.cx - center_x) / max(1.0, frame_w)
        lower_bonus = det.cy / max(1.0, frame_h)
        clearance_bonus = min(1.0, (clearance or args.target_clear_m) / max(0.01, args.target_clear_m))
        wall_penalty = target_wall_penalty(lidar, angle, args)
        jitter = random.uniform(0.0, args.target_random_jitter) if args.target_random_jitter > 0 else 0.0
        color_bonus = 1.0
        if preferred and preferred != "any":
            color_bonus = 1.35 if det.color == preferred else 0.65
        return (
            det.area
            * (1.0 + 0.15 * lower_bonus)
            * (1.0 - 0.20 * center_penalty)
            * (0.35 + 0.65 * clearance_bonus)
            * max(0.10, 1.0 - wall_penalty)
            * color_bonus
            * (1.0 + jitter)
        )

    candidates: list[tuple[float, Detection]] = [(score(det), det) for det in detections]

    if args.target_cluster_enable and len(detections) >= args.target_cluster_min_count:
        ordered = sorted(detections, key=lambda det: det.cx)
        clusters: list[list[Detection]] = []
        current: list[Detection] = []
        last_cx: Optional[float] = None
        for det in ordered:
            if last_cx is None or det.cx - last_cx <= args.target_cluster_gap_px:
                current.append(det)
            else:
                clusters.append(current)
                current = [det]
            last_cx = det.cx
        if current:
            clusters.append(current)

        for members in clusters:
            if len(members) < args.target_cluster_min_count:
                continue
            weights = [max(1.0, math.sqrt(det.area) * max(0.05, det.conf)) for det in members]
            total_w = sum(weights)
            cx = sum(det.cx * w for det, w in zip(members, weights)) / total_w
            cy = sum(det.cy * w for det, w in zip(members, weights)) / total_w
            diameter = max(det.diameter_px for det in members)
            color_set = sorted({det.color for det in members if det.color})
            color = color_set[0] if len(color_set) == 1 else ("mixed" if color_set else "")
            cluster = Detection(
                "cluster",
                f"ball cluster x{len(members)}",
                min(0.99, max(det.conf for det in members) + 0.04 * (len(members) - 1)),
                cx - diameter * 0.5,
                cy - diameter * 0.5,
                cx + diameter * 0.5,
                cy + diameter * 0.5,
                color,
            )
            cluster_score = sum(score(det) for det in members) * (
                1.0 + args.target_cluster_bonus * (len(members) - 1)
            )
            candidates.append((cluster_score, cluster))

    return max(candidates, key=lambda item: item[0])[1]


def _target_center_gap_px(a: Detection, b: Detection) -> float:
    return math.hypot(a.cx - b.cx, a.cy - b.cy)


def _blend_detection(previous: Detection, current: Detection, alpha: float) -> Detection:
    alpha = max(0.0, min(1.0, alpha))
    beta = 1.0 - alpha
    label = current.label if current.source != "held" else previous.label
    color = current.color or previous.color
    return Detection(
        current.source,
        label,
        current.conf,
        previous.x1 * beta + current.x1 * alpha,
        previous.y1 * beta + current.y1 * alpha,
        previous.x2 * beta + current.x2 * alpha,
        previous.y2 * beta + current.y2 * alpha,
        color,
    )


def stabilize_target(
    target: Optional[Detection],
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
) -> Optional[Detection]:
    if target is not None:
        if (
            mem.held_target is not None
            and now - mem.held_target_s <= args.target_hold_s
        ):
            gap = _target_center_gap_px(mem.held_target, target)
            if gap <= args.target_smooth_max_jump_px:
                target = _blend_detection(mem.held_target, target, args.target_smooth_alpha)
                mem.locked_target_s = now
            elif now - mem.locked_target_s <= args.target_switch_lock_s:
                held = mem.held_target
                return Detection(
                    "held",
                    f"locked {held.label}",
                    max(0.01, held.conf * 0.95),
                    held.x1,
                    held.y1,
                    held.x2,
                    held.y2,
                    held.color,
                )
        mem.held_target = target
        mem.held_target_s = now
        mem.locked_target_s = now
        return target

    if mem.held_target is not None and now - mem.held_target_s <= args.target_hold_s:
        age = max(0.0, now - mem.held_target_s)
        held = mem.held_target
        return Detection(
            "held",
            f"held {held.label}",
            max(0.01, held.conf * (1.0 - age / max(0.01, args.target_hold_s))),
            held.x1,
            held.y1,
            held.x2,
            held.y2,
            held.color,
        )

    mem.held_target = None
    mem.held_target_s = 0.0
    mem.locked_target_s = 0.0
    return None


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


def orient_frame(frame, args: argparse.Namespace):
    rotate = int(args.camera_rotate) % 360
    if rotate == 90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotate == 180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    elif rotate == 270:
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if args.flip_horizontal:
        frame = cv2.flip(frame, 1)
    if args.flip_vertical:
        frame = cv2.flip(frame, 0)
    return frame


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


def command_expect(ser, command: str, ok_prefix: Optional[str], seconds: float) -> tuple[bool, list[str]]:
    send_line(ser, command)
    lines = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        lines.append(line)
        if ok_prefix is not None and line.startswith(ok_prefix):
            return True, lines
    return ok_prefix is None, lines


def preflight_stm32(ser, args: argparse.Namespace) -> None:
    ser.reset_input_buffer()
    drain_lines(ser, 0.20)
    commands = [
        ("STOP", "OK STOP", 0.60),
        ("ENABLE 0", "OK ENABLE 0", 0.60),
        ("TELEM 5", "OK TELEM 5", 0.60),
        ("PING", "OK PONG", 0.80),
        ("STATUS", "OK STATUS", 0.80),
        (f"LIMIT {args.limit}", f"OK LIMIT {args.limit}", 0.80),
        ("MOTOR_SIGN " + " ".join(str(v) for v in args.motor_sign), "OK MOTOR_SIGN", 0.80),
        (f"TELEM {args.telemetry_hz}", f"OK TELEM {args.telemetry_hz}", 0.80),
    ]
    for command, ok_prefix, seconds in commands:
        ok, _lines = command_expect(ser, command, ok_prefix, seconds)
        if ok:
            continue
        if args.allow_no_stm32_ack:
            print(f"WARN STM32 did not acknowledge {command}; continuing because --allow-no-stm32-ack is set")
        else:
            raise RuntimeError(f"STM32 did not acknowledge {command}")


def enable_live_stm32(ser, args: argparse.Namespace) -> None:
    ok, _lines = command_expect(ser, "ENABLE 1", "OK ENABLE 1", 0.80)
    if ok:
        return
    if args.allow_no_stm32_ack:
        print("WARN STM32 did not acknowledge ENABLE 1; continuing because --allow-no-stm32-ack is set")
    else:
        raise RuntimeError("STM32 did not acknowledge ENABLE 1")


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
        self.last_front_s = 0.0
        self.last_front_min_m: Optional[float] = None
        self.last_front_left_m: Optional[float] = None
        self.last_front_right_m: Optional[float] = None
        self.front_close_count = 0
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
        left = sector_stats(points, self.args.front_center, 270.0, self.args.side_width / 2.0)
        right = sector_stats(points, self.args.front_center, 90.0, self.args.side_width / 2.0)
        rear = sector_stats(points, self.args.front_center, 180.0, self.args.side_width / 2.0)
        front_value = choose(front, self.args.front_stat)
        front_left_value = choose(front_left, self.args.front_stat)
        front_right_value = choose(front_right, self.args.front_stat)
        front_values = [front_value, front_left_value, front_right_value]
        present = [v for v in front_values if v is not None]
        front_min = min(present) if present else None
        if front_min is not None:
            self.last_front_s = now
            self.last_front_min_m = front_min
            self.last_front_left_m = front_left_value
            self.last_front_right_m = front_right_value
            if front_min < self.args.lidar_stop_m:
                self.front_close_count += 1
            else:
                self.front_close_count = 0
            reason = "ok"
        else:
            held_age = now - self.last_front_s if self.last_front_s else 999.0
            if (
                self.args.lidar_front_hold_s > 0.0
                and self.last_front_min_m is not None
                and held_age <= self.args.lidar_front_hold_s
            ):
                front_min = self.last_front_min_m
                front_left_value = self.last_front_left_m
                front_right_value = self.last_front_right_m
                reason = f"front held {held_age:.2f}s"
            else:
                self.front_close_count = 0
                reason = "front unknown"
        stale = age > self.args.lidar_stale_s
        ok = not stale and (front_min is not None or self.args.allow_missing_front_lidar)
        if stale:
            reason = f"lidar stale {age:.2f}s"
        self.last_snapshot = LidarSnapshot(
            ok=ok,
            age_s=age,
            front_min_m=front_min,
            front_left_m=front_left_value,
            front_right_m=front_right_value,
            left_m=choose(left, self.args.side_stat),
            right_m=choose(right, self.args.side_stat),
            rear_m=choose(rear, self.args.side_stat),
            point_count=len(points),
            reason=reason,
            front_close_count=self.front_close_count,
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
        if args.allow_missing_front_lidar:
            return True, snapshot.reason or "front unknown", args.lidar_missing_front_scale
        return False, "no front lidar data", 0.0
    if front < args.lidar_stop_m:
        if snapshot.front_close_count < args.lidar_close_frames:
            scale = min(args.lidar_transient_scale, args.lidar_missing_front_scale)
            return True, f"front transient {front:.2f}m", scale
        return False, f"front {front:.2f}m < stop {args.lidar_stop_m:.2f}m", 0.0
    if front < args.lidar_slow_m:
        scale = max(0.25, (front - args.lidar_stop_m) / max(0.01, args.lidar_slow_m - args.lidar_stop_m))
        return True, f"front caution {front:.2f}m", scale
    return True, "clear", 1.0


def side_wall_state(lidar: LidarSnapshot, args: argparse.Namespace) -> tuple[str, float]:
    left = lidar.left_m
    right = lidar.right_m
    if left is not None and left <= args.side_stop_m:
        return "left-tight", 0.0
    if right is not None and right <= args.side_stop_m:
        return "right-tight", 0.0
    if left is not None and left <= args.side_slow_m:
        span = max(0.01, args.side_slow_m - args.side_stop_m)
        return "left-near", max(0.30, (left - args.side_stop_m) / span)
    if right is not None and right <= args.side_slow_m:
        span = max(0.01, args.side_slow_m - args.side_stop_m)
        return "right-near", max(0.30, (right - args.side_stop_m) / span)
    return "clear", 1.0


def wall_hold_strafe(lidar: LidarSnapshot, args: argparse.Namespace) -> tuple[str, int, float]:
    candidates: list[tuple[str, float, float]] = []
    if lidar.left_m is not None and lidar.left_m <= args.wall_track_m:
        candidates.append(("left", lidar.left_m, args.wall_distance_m - lidar.left_m))
    if lidar.right_m is not None and lidar.right_m <= args.wall_track_m:
        candidates.append(("right", lidar.right_m, lidar.right_m - args.wall_distance_m))
    if not candidates:
        return "clear", 0, 1.0

    side, distance, error = min(candidates, key=lambda item: item[1])
    if side == "left" and distance <= args.side_stop_m:
        state = "left-tight"
    elif side == "right" and distance <= args.side_stop_m:
        state = "right-tight"
    elif side == "left" and distance <= args.side_slow_m:
        state = "left-near"
    elif side == "right" and distance <= args.side_slow_m:
        state = "right-near"
    else:
        state = f"{side}-hold"

    if abs(error) <= args.wall_deadband_m:
        return state, 0, 1.0

    strafe = signed_pwm(error * args.wall_hold_gain, args.min_strafe_pwm, args.wall_strafe_pwm)
    if distance <= args.side_stop_m:
        scale = 0.0
    elif distance <= args.side_slow_m:
        span = max(0.01, args.side_slow_m - args.side_stop_m)
        scale = max(0.30, (distance - args.side_stop_m) / span)
    else:
        scale = 1.0
    return state, strafe, scale


def omni_wall_pwm(lidar: LidarSnapshot, args: argparse.Namespace, forward: int | float = 0) -> tuple[str, list[int], str]:
    state, strafe, scale = wall_hold_strafe(lidar, args)
    if state == "clear":
        strafe = clearer_strafe(lidar, args)
        state = "side-clearer"
    forward = clamp(forward * scale, args.limit)
    pwm = mecanum_pwm(forward, strafe, 0, args)
    return "OMNI_WALL", pwm, f"{state}; strafe {strafe:+d}, forward {forward:+d}"


def omni_escape_pwm(lidar: LidarSnapshot, args: argparse.Namespace, reason: str) -> tuple[str, list[int], str]:
    rear_clear = lidar.rear_m is None or lidar.rear_m >= args.rear_stop_m
    backward = -args.escape_pwm if rear_clear else 0
    state, strafe, _ = wall_hold_strafe(lidar, args)
    if strafe == 0:
        strafe = clearer_strafe(lidar, args)
    if backward == 0 and strafe == 0:
        return "STOP", [0, 0, 0, 0], f"escape blocked: {reason}"
    pwm = mecanum_pwm(backward, strafe, 0, args)
    return "OMNI_ESCAPE", pwm, f"{reason}; {state}; back {backward:+d}, strafe {strafe:+d}"


def pd_turn_command(angle_deg: float, mem: MissionMemory, args: argparse.Namespace, now: float) -> int:
    dt = max(0.02, min(0.30, now - mem.last_pd_s)) if mem.last_pd_s else 0.10
    derivative = (angle_deg - mem.prev_angle_error_deg) / dt
    mem.prev_angle_error_deg = angle_deg
    mem.last_pd_s = now
    turn = angle_deg * args.turn_gain + derivative * args.turn_kd
    return clamp(turn, args.turn_pwm)


def pd_strafe_command(angle_deg: float, mem: MissionMemory, args: argparse.Namespace, now: float) -> int:
    dt = max(0.02, min(0.30, now - mem.last_pd_s)) if mem.last_pd_s else 0.10
    derivative = (angle_deg - mem.prev_angle_error_deg) / dt
    mem.prev_angle_error_deg = angle_deg
    mem.last_pd_s = now
    strafe = angle_deg * args.camera_strafe_gain + derivative * args.camera_strafe_kd
    return clamp(strafe, args.strafe_pwm)


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


def clearer_turn_pwm(lidar: LidarSnapshot, args: argparse.Namespace) -> int:
    left = lidar.left_m if lidar.left_m is not None else 0.0
    right = lidar.right_m if lidar.right_m is not None else 0.0
    return -args.turn_pwm if left >= right else args.turn_pwm


def tank_escape_pwm(lidar: LidarSnapshot, args: argparse.Namespace, reason: str) -> tuple[str, list[int], str]:
    rear_clear = lidar.rear_m is None or lidar.rear_m >= args.rear_stop_m
    if rear_clear:
        pwm = [-args.escape_pwm, -args.escape_pwm, -args.escape_pwm, -args.escape_pwm]
        return "BACKUP", pwm, f"{reason}; backing up"
    turn = clearer_turn_pwm(lidar, args)
    return "TURN_CLEAR", mix_pwm(0, turn, args.limit), f"{reason}; rear blocked, turning"


def signed_angle_delta_deg(current: float, previous: float) -> float:
    return ((current - previous + 180.0) % 360.0) - 180.0


def tank_recovery_pwm(lidar: LidarSnapshot, args: argparse.Namespace, reason: str) -> tuple[str, list[int], str]:
    front_close = lidar.front_min_m is not None and lidar.front_min_m < args.lidar_slow_m
    rear_clear = lidar.rear_m is None or lidar.rear_m >= args.rear_stop_m
    left = lidar.left_m
    right = lidar.right_m

    if front_close and rear_clear:
        pwm = [-args.escape_pwm, -args.escape_pwm, -args.escape_pwm, -args.escape_pwm]
        return "STUCK_BACKUP", pwm, f"{reason}; front close, backing up"

    if left is not None or right is not None:
        if left is not None and (right is None or left < right):
            turn = args.turn_pwm
            side = "left wall"
        elif right is not None and (left is None or right < left):
            turn = -args.turn_pwm
            side = "right wall"
        else:
            turn = clearer_turn_pwm(lidar, args)
            side = "clearer side"
        return "STUCK_TURN_AWAY", mix_pwm(0, turn, args.limit), f"{reason}; turning away from {side}"

    if rear_clear:
        pwm = [-args.escape_pwm, -args.escape_pwm, -args.escape_pwm, -args.escape_pwm]
        return "STUCK_BACKUP", pwm, f"{reason}; backing up"

    turn = clearer_turn_pwm(lidar, args)
    return "STUCK_TURN_CLEAR", mix_pwm(0, turn, args.limit), f"{reason}; rear blocked, turning"


def reset_stuck_watch(mem: MissionMemory, now: float) -> None:
    mem.stuck_last_change_s = now
    mem.stuck_last_front_m = None
    mem.stuck_last_left_m = None
    mem.stuck_last_right_m = None
    mem.stuck_last_target_cx = None
    mem.stuck_last_target_diameter_px = None
    mem.stuck_last_yaw_deg = None


def _value_changed(current: Optional[float], previous: Optional[float], delta: float) -> tuple[bool, bool]:
    if current is None:
        return False, False
    if previous is None:
        return True, True
    return True, abs(current - previous) >= delta


def update_stuck_watch(
    mem: MissionMemory,
    target: Optional[Detection],
    lidar: LidarSnapshot,
    imu_sample,
    args: argparse.Namespace,
    now: float,
) -> bool:
    observed = False
    changed = False

    for name, current, delta in (
        ("front", lidar.front_min_m if lidar.ok else None, args.stuck_lidar_delta_m),
        ("left", lidar.left_m if lidar.ok else None, args.stuck_lidar_delta_m),
        ("right", lidar.right_m if lidar.ok else None, args.stuck_lidar_delta_m),
    ):
        previous = getattr(mem, f"stuck_last_{name}_m")
        present, moved = _value_changed(current, previous, delta)
        observed = observed or present
        changed = changed or moved
        if current is not None:
            setattr(mem, f"stuck_last_{name}_m", current)

    if target is not None and target.source != "held":
        observed = True
        if (
            mem.stuck_last_target_cx is None
            or abs(target.cx - mem.stuck_last_target_cx) >= args.stuck_target_delta_px
            or abs(target.diameter_px - (mem.stuck_last_target_diameter_px or target.diameter_px))
            >= args.stuck_target_diameter_delta_px
        ):
            changed = True
        mem.stuck_last_target_cx = target.cx
        mem.stuck_last_target_diameter_px = target.diameter_px

    yaw = getattr(imu_sample, "yaw_deg", None)
    if getattr(imu_sample, "enabled", False) and getattr(imu_sample, "ok", False) and yaw is not None:
        observed = True
        if mem.stuck_last_yaw_deg is None or abs(signed_angle_delta_deg(yaw, mem.stuck_last_yaw_deg)) >= args.stuck_yaw_delta_deg:
            changed = True
        mem.stuck_last_yaw_deg = yaw

    if changed or mem.stuck_last_change_s <= 0.0:
        mem.stuck_last_change_s = now
    return observed


def apply_stuck_recovery(
    action: str,
    pwm: list[int],
    reason: str,
    target: Optional[Detection],
    lidar: LidarSnapshot,
    imu_sample,
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
    live: bool,
) -> tuple[str, list[int], str]:
    if not live or not args.enable_stuck_recovery:
        return action, pwm, reason

    safety_ok, _safety_reason, _safety_scale = lidar_allows_motion(lidar, args, live)
    if not safety_ok:
        mem.recovery_until_s = 0.0
        reset_stuck_watch(mem, now)
        return action, pwm, reason

    if mem.recovery_until_s > now:
        return mem.recovery_action, list(mem.recovery_pwm), mem.recovery_reason

    moving = action not in ("STOP", "RETURN_HOME") and any(abs(value) >= args.stuck_min_pwm for value in pwm)
    if not moving:
        reset_stuck_watch(mem, now)
        return action, pwm, reason

    observed = update_stuck_watch(mem, target, lidar, imu_sample, args, now)
    if not observed:
        reset_stuck_watch(mem, now)
        return action, pwm, reason

    if now - mem.stuck_last_change_s < args.stuck_seconds:
        return action, pwm, reason

    clear_tank_step(mem)
    stuck_reason = f"stuck suspected for {now - mem.stuck_last_change_s:.1f}s"
    if args.drive_mode == "omni":
        recovery_action, recovery_pwm, recovery_reason = omni_escape_pwm(lidar, args, stuck_reason)
    else:
        recovery_action, recovery_pwm, recovery_reason = tank_recovery_pwm(lidar, args, stuck_reason)
    mem.recovery_until_s = now + args.stuck_recover_s
    mem.recovery_action = recovery_action
    mem.recovery_pwm = list(recovery_pwm)
    mem.recovery_reason = recovery_reason
    reset_stuck_watch(mem, now)
    return recovery_action, recovery_pwm, recovery_reason


def start_tank_step(
    mem: MissionMemory,
    now: float,
    action: str,
    pwm: list[int],
    duration_s: float,
    reason: str,
    args: argparse.Namespace,
) -> tuple[str, list[int], str]:
    duration_s = max(args.step_min_s, min(args.step_max_s, duration_s))
    mem.step_action = action
    mem.step_pwm = list(pwm)
    mem.step_until_s = now + duration_s
    mem.step_reason = reason
    return action, pwm, f"{reason}; step {duration_s:.2f}s"


def current_tank_step(mem: MissionMemory, now: float) -> Optional[tuple[str, list[int], str]]:
    if mem.step_until_s <= now or not mem.step_action:
        return None
    remaining = max(0.0, mem.step_until_s - now)
    return mem.step_action, list(mem.step_pwm), f"{mem.step_reason}; step remaining {remaining:.2f}s"


def clear_tank_step(mem: MissionMemory) -> None:
    mem.step_action = ""
    mem.step_pwm = [0, 0, 0, 0]
    mem.step_until_s = 0.0
    mem.step_reason = ""


def tank_step_turn_duration(angle_deg: float, args: argparse.Namespace) -> float:
    return args.step_turn_base_s + abs(angle_deg) * args.step_turn_s_per_deg


def imu_yaw_deg(imu_sample) -> Optional[float]:
    yaw = getattr(imu_sample, "yaw_deg", None)
    if getattr(imu_sample, "enabled", False) and getattr(imu_sample, "ok", False) and yaw is not None:
        return float(yaw)
    return None


def build_home_dock_args(args: argparse.Namespace) -> SimpleNamespace:
    return SimpleNamespace(
        target_tag_ids=args.home_tag_ids,
        no_lidar=args.no_lidar,
        require_lidar_in_dry_run=args.require_lidar_in_dry_run,
        target_hold_s=args.home_tag_hold_s,
        search_flip_s=args.home_search_flip_s,
        search_pwm=args.home_search_pwm,
        search_boost_pwm=args.home_search_boost_pwm,
        search_sweep_deg=args.home_search_sweep_deg,
        search_segment_deg=args.home_search_segment_deg,
        search_stall_s=args.home_search_stall_s,
        search_progress_epsilon_deg=args.home_search_progress_epsilon_deg,
        search_alternate=args.home_search_alternate,
        limit=args.limit,
        align_tolerance_deg=args.home_align_tolerance_deg,
        align_gain=args.home_align_gain,
        min_turn_pwm=args.home_min_turn_pwm,
        align_turn_pwm=args.home_align_turn_pwm,
        align_boost_pwm=args.home_align_boost_pwm,
        align_stall_s=args.home_align_stall_s,
        align_progress_epsilon_deg=args.home_align_progress_epsilon_deg,
        align_step_s=args.home_align_step_s,
        dock_approach_m=args.home_dock_approach_m,
        dock_distance_tolerance_m=args.home_dock_distance_tolerance_m,
        approach_front_stop_m=args.home_front_stop_m,
        approach_front_slow_m=args.home_front_slow_m,
        missing_front_scale=args.home_missing_front_scale,
        approach_pwm=args.home_approach_pwm,
        slow_approach_m=args.home_slow_approach_m,
        creep_pwm=args.home_creep_pwm,
        forward_turn_gain=args.home_forward_turn_gain,
        max_forward_turn_pwm=args.home_max_forward_turn_pwm,
        kick_pwm=args.home_kick_pwm,
        kick_s=args.home_kick_s,
        kick_cooldown_s=args.home_kick_cooldown_s,
        kick_min_front_scale=args.home_kick_min_front_scale,
        step_min_s=args.home_step_min_s,
        step_max_s=args.home_step_max_s,
        step_forward_s=args.home_step_forward_s,
        rear_preturn_stop_m=args.home_rear_preturn_stop_m,
        turn_180_deg=args.home_turn_180_deg,
        turn_tolerance_deg=args.home_turn_tolerance_deg,
        post_turn_pause_s=args.home_post_turn_pause_s,
        backup_timeout_s=args.home_backup_timeout_s,
        rear_dock_m=args.home_rear_dock_m,
        rear_min_settle_s=args.home_rear_min_settle_s,
        dock_confirm_s=args.home_dock_confirm_s,
        backup_pwm=args.home_backup_pwm,
        backup_force_pwm=args.home_backup_force_pwm,
        backup_force_s=args.home_backup_force_s,
        turn_direction=args.home_turn_direction,
        turn_180_pwm=args.home_turn_180_pwm,
        turn_boost_pwm=args.home_turn_boost_pwm,
        turn_stall_s=args.home_turn_stall_s,
        turn_progress_epsilon_deg=args.home_turn_progress_epsilon_deg,
        turn_slowdown_deg=args.home_turn_slowdown_deg,
    )


def adapt_home_lidar(lidar: LidarSnapshot):
    if AprilDockLidarSnapshot is None:
        return None
    return AprilDockLidarSnapshot(
        ok=lidar.ok,
        age_s=lidar.age_s,
        front_m=lidar.front_min_m,
        rear_m=lidar.rear_m,
        left_m=lidar.left_m,
        right_m=lidar.right_m,
        point_count=lidar.point_count,
        reason=lidar.reason,
    )


def detect_home_tags(frame, detector, target_ids: set[int], args: argparse.Namespace, focal_px: float):
    if detector is None:
        return [], [], None, 0
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=(focal_px, focal_px, frame.shape[1] / 2.0, frame.shape[0] / 2.0),
        tag_size=args.home_tag_size_m,
    )
    tags = [apriltag_to_dict(det, frame.shape[1], focal_px, args.home_tag_size_m) for det in detections]
    target, _components, distractor_count = choose_apriltag_target(tags, target_ids, frame.shape[1], focal_px)
    return detections, tags, target, distractor_count


def decide_home_dock_motion(
    home_target: Optional[dict],
    lidar: LidarSnapshot,
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
    live: bool,
    imu_sample=None,
) -> tuple[str, list[int], str, Optional[float], Optional[float], str]:
    if not args.home_dock_enable:
        return "STOP", [0, 0, 0, 0], "home docking disabled", None, None, "home disabled"
    if AprilDockMemory is None or decide_apriltag_dock_motion is None:
        mem.state = "HOME_DOCK_UNAVAILABLE"
        return "STOP", [0, 0, 0, 0], "AprilTag docking module unavailable", None, None, "apriltag unavailable"
    if mem.home_dock is None:
        mem.home_dock = AprilDockMemory()
    dock_lidar = adapt_home_lidar(lidar)
    if dock_lidar is None:
        mem.state = "HOME_DOCK_UNAVAILABLE"
        return "STOP", [0, 0, 0, 0], "AprilTag LiDAR adapter unavailable", None, None, "apriltag unavailable"

    dock_args = build_home_dock_args(args)
    action, pwm, reason = decide_apriltag_dock_motion(home_target, dock_lidar, imu_sample, mem.home_dock, dock_args, now, live)
    dock_state = getattr(mem.home_dock, "state", "UNKNOWN")
    mem.state = f"HOME_{dock_state}"
    distance = home_target.get("distance_m") if home_target is not None else None
    angle = home_target.get("angle_deg") if home_target is not None else None

    if dock_state == "DOCKED":
        if args.release_gate_after_dock and not mem.release_done:
            mem.state = "RELEASE_GATE"
            status = mem.release_status or "pending"
            return "STOP", [0, 0, 0, 0], f"docked; gate release {status}", distance, angle, "docked"
        mem.finished = True
        mem.state = "FINISHED"
        return "STOP", [0, 0, 0, 0], "home dock complete; release complete", distance, angle, "finished"

    prefixed_action = action if action == "STOP" else f"HOME_{action}"
    return prefixed_action, pwm, reason, distance, angle, dock_lidar.reason


def release_gate_once(mem: MissionMemory, args: argparse.Namespace, live: bool) -> None:
    if mem.release_done or mem.release_started_s > 0.0:
        return
    mem.release_started_s = time.monotonic()
    if not args.release_gate_after_dock:
        mem.release_done = True
        mem.release_status = "disabled"
        return
    if not args.release_live_servo or not live:
        mem.release_done = True
        mem.release_status = "dry-run; add --release-live-servo to move servo"
        return
    if make_gate is None or move_servo is None or bounded_pulse_us is None:
        mem.release_done = True
        mem.release_status = "servo module unavailable"
        return

    servo_args = SimpleNamespace(
        pin=args.release_servo_pin,
        backend=args.release_servo_backend,
        frequency_hz=args.release_servo_frequency_hz,
        home_us=args.release_servo_home_us,
        open_us=args.release_servo_open_us,
        min_us=args.release_servo_min_us,
        max_us=args.release_servo_max_us,
        move_s=args.release_servo_move_s,
        step_us=args.release_servo_step_us,
        home_hold_s=args.release_servo_home_hold_s,
        open_hold_s=args.release_servo_open_hold_s,
        detach_final=args.release_servo_detach_final,
    )
    servo_args.home_us = bounded_pulse_us(servo_args.home_us, servo_args)
    servo_args.open_us = bounded_pulse_us(servo_args.open_us, servo_args)
    gate = make_gate(servo_args)
    try:
        mem.release_status = "opening"
        gate.open()
        gate.write_us(servo_args.home_us)
        time.sleep(max(0.0, servo_args.home_hold_s))
        move_servo(gate, servo_args.home_us, servo_args.open_us, servo_args)
        time.sleep(max(0.0, servo_args.open_hold_s))
        if args.release_servo_return_home:
            mem.release_status = "closing"
            move_servo(gate, servo_args.open_us, servo_args.home_us, servo_args)
            time.sleep(max(0.0, servo_args.home_hold_s))
        mem.release_status = "done"
    except Exception as exc:
        mem.release_status = f"servo error: {exc}"
    finally:
        try:
            gate.close(servo_args.home_us, servo_args.home_hold_s, servo_args.detach_final)
        finally:
            mem.release_done = True


def tank_imu_search_turn(
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
    yaw: float,
    safety_reason: str,
) -> tuple[str, list[int], str, Optional[float], Optional[float], str]:
    if mem.search_recovery_until_s > now:
        remaining = max(0.0, mem.search_recovery_until_s - now)
        return "SEARCH_RECOVERY", list(mem.search_recovery_pwm), f"search yaw stalled; recovery {remaining:.2f}s", None, None, safety_reason

    if mem.search_yaw_start_deg is None:
        mem.search_yaw_start_deg = yaw
        mem.search_last_progress_deg = 0.0
        mem.search_last_progress_s = now
    turned = abs(signed_angle_delta_deg(yaw, mem.search_yaw_start_deg))
    if (
        mem.search_last_progress_s <= 0.0
        or turned - mem.search_last_progress_deg >= args.search_imu_progress_epsilon_deg
    ):
        mem.search_last_progress_deg = turned
        mem.search_last_progress_s = now
    if turned >= args.search_imu_turn_deg:
        mem.search_yaw_start_deg = yaw
        mem.search_last_progress_deg = 0.0
        mem.search_last_progress_s = now
        if args.search_imu_alternate:
            mem.search_turn_dir *= -1
        turned = 0.0
    if now - mem.search_last_progress_s >= args.search_imu_stall_s:
        mem.search_turn_dir *= -1
        mem.search_yaw_start_deg = yaw
        mem.search_last_progress_deg = 0.0
        mem.search_last_progress_s = now
        recovery_turn = mem.search_turn_dir * max(args.search_pwm, args.search_imu_recover_turn_pwm)
        reverse = -max(0, args.search_imu_recover_reverse_pwm)
        mem.search_recovery_pwm = mix_pwm(reverse, recovery_turn, args.limit)
        mem.search_recovery_until_s = now + args.search_imu_recover_s
        clear_tank_step(mem)
        return (
            "SEARCH_RECOVERY",
            list(mem.search_recovery_pwm),
            f"search yaw stalled at {turned:.0f}/{args.search_imu_turn_deg:.0f}deg; reversing sweep",
            None,
            None,
            safety_reason,
        )
    turn = mem.search_turn_dir * args.search_pwm
    pwm = mix_pwm(0, turn, args.limit)
    return (
        *start_tank_step(
            mem,
            now,
            "SEARCH_IMU_TURN",
            pwm,
            args.search_imu_step_s,
            f"no ball target; imu sweep {turned:.0f}/{args.search_imu_turn_deg:.0f}deg yaw {yaw:.1f}; {safety_reason}",
            args,
        ),
        None,
        None,
        safety_reason,
    )


def decide_motion_tank_step(
    target: Optional[Detection],
    frame_w: int,
    frame_h: int,
    focal_px: float,
    lidar: LidarSnapshot,
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
    live: bool,
    imu_sample=None,
) -> tuple[str, list[int], str, Optional[float], Optional[float], str]:
    safety_ok, safety_reason, safety_scale = lidar_allows_motion(lidar, args, live)
    mem.wall_state, _wall_scale = side_wall_state(lidar, args)
    if mem.finished:
        clear_tank_step(mem)
        return "STOP", [0, 0, 0, 0], "finished", None, None, safety_reason

    if not safety_ok:
        clear_tank_step(mem)
        if lidar.ok and lidar.front_min_m is not None and lidar.front_min_m < args.lidar_stop_m:
            action, pwm, reason = tank_escape_pwm(lidar, args, safety_reason)
            return action, pwm, reason, None, None, safety_reason
        return "STOP", [0, 0, 0, 0], f"blocked: {safety_reason}", None, None, safety_reason

    angle: Optional[float] = None
    distance: Optional[float] = None
    if target is None and mem.state == "COLLECT" and mem.collect_since_s > 0.0:
        clear_tank_step(mem)
        collect_elapsed = now - mem.collect_since_s
        if collect_elapsed < args.collect_drive_s:
            forward = clamp(args.collect_drive_pwm * safety_scale, args.limit)
            return (
                "COLLECT_FORWARD",
                [forward, forward, forward, forward],
                f"finishing collect drive blind {collect_elapsed:.2f}/{args.collect_drive_s:.2f}s",
                None,
                None,
                safety_reason,
            )
        mem.collected += 1
        mem.state = "SEARCH"
        mem.collect_since_s = 0.0
        return "STOP", [0, 0, 0, 0], f"ball counted {mem.collected}/{args.target_count}", None, None, safety_reason

    if target is not None:
        mem.last_target_s = now
        angle = estimate_angle_deg(target.cx, frame_w, focal_px, args.angle_model)
        distance = estimate_distance_m(target.diameter_px, focal_px, args.ball_diameter_m)
        collect_ok, collect_reason = collect_reached(target, frame_w, frame_h, angle, distance, args)
        if collect_ok:
            clear_tank_step(mem)
            if target.source == "held" and mem.state != "COLLECT":
                mem.collect_since_s = 0.0
                mem.state = "APPROACH"
                return "STOP", [0, 0, 0, 0], f"held target reached ({collect_reason}); waiting for live detection", distance, angle, safety_reason
            if mem.state != "COLLECT":
                mem.state = "COLLECT"
                mem.collect_since_s = now
            collect_elapsed = now - mem.collect_since_s
            if collect_elapsed < args.collect_drive_s:
                forward = clamp(args.collect_drive_pwm * safety_scale, args.limit)
                return (
                    "COLLECT_FORWARD",
                    [forward, forward, forward, forward],
                    f"collect drive-in {collect_elapsed:.2f}/{args.collect_drive_s:.2f}s ({collect_reason})",
                    distance,
                    angle,
                    safety_reason,
                )
            if collect_elapsed >= args.collect_drive_s + args.collect_hold_s:
                mem.collected += 1
                mem.state = "SEARCH"
                mem.collect_since_s = 0.0
                return "STOP", [0, 0, 0, 0], f"ball counted {mem.collected}/{args.target_count} ({collect_reason})", distance, angle, safety_reason
            return "STOP", [0, 0, 0, 0], f"collect drive complete; settling ({collect_reason})", distance, angle, safety_reason

    active = current_tank_step(mem, now)
    if active is not None:
        action, pwm, reason = active
        if action.startswith("SEARCH") and target is not None and target.source != "held":
            clear_tank_step(mem)
            mem.search_yaw_start_deg = None
        else:
            if target is not None and angle is not None:
                reason += f"; target angle now {angle:.1f}deg"
            return action, pwm, reason, distance, angle, safety_reason

    if target is not None and target.source != "held":
        mem.search_yaw_start_deg = None

    if target is None or angle is None:
        mem.state = "SEARCH"
        yaw = imu_yaw_deg(imu_sample) if args.search_use_imu else None
        if yaw is not None:
            return tank_imu_search_turn(mem, args, now, yaw, safety_reason)

        if target is not None and angle is not None:
            mem.search_yaw_start_deg = None
        search_phase = int(now / max(0.5, args.search_sweep_s * 0.5)) % 4
        if args.explore_forward_pwm > 0 and search_phase in (0, 2):
            forward = clamp(args.explore_forward_pwm * safety_scale, args.limit)
            pwm = [forward, forward, forward, forward]
            return (
                *start_tank_step(
                    mem,
                    now,
                    "SEARCH_FORWARD",
                    pwm,
                    args.step_forward_s,
                    "no ball target; roaming forward",
                    args,
                ),
                None,
                None,
                safety_reason,
            )
        search_dir = 1 if search_phase == 1 else -1
        pwm = mix_pwm(0, search_dir * args.search_pwm, args.limit)
        return (*start_tank_step(mem, now, "SEARCH_TURN", pwm, args.step_search_s, "no ball target; roaming sweep", args), None, None, safety_reason)

    mem.state = "APPROACH"
    abs_angle = abs(angle)
    turn_dir = 1 if angle > 0.0 else -1
    front_cautious = lidar.front_min_m is not None and lidar.front_min_m < args.lidar_slow_m

    if (
        args.kick_pwm > 0
        and args.kick_s > 0.0
        and not front_cautious
        and abs_angle < args.step_spin_angle_deg
        and now - mem.last_kick_s >= args.kick_cooldown_s
    ):
        kick_forward = clamp(args.kick_pwm * safety_scale, args.limit)
        if abs_angle > args.center_tolerance_deg:
            kick_turn = turn_dir * min(args.kick_turn_pwm, args.limit)
            pwm = mix_pwm(kick_forward, kick_turn, args.limit)
            reason = f"anti-stiction kick toward target {angle:.1f}deg"
        else:
            pwm = [kick_forward, kick_forward, kick_forward, kick_forward]
            reason = f"anti-stiction kick forward on centered target {angle:.1f}deg"
        mem.last_kick_s = now
        return (
            *start_tank_step(mem, now, "KICK_TO_BALL", pwm, args.kick_s, reason, args),
            distance,
            angle,
            safety_reason,
        )

    if abs_angle >= args.step_spin_angle_deg or front_cautious:
        turn_pwm = signed_pwm(angle * args.turn_gain, args.min_turn_pwm, args.turn_pwm)
        duration = tank_step_turn_duration(angle, args)
        pwm = mix_pwm(0, turn_pwm, args.limit)
        reason = f"turn to target {angle:.1f}deg"
        if front_cautious:
            reason += "; front caution, no forward arc"
        return (*start_tank_step(mem, now, "STEP_TURN_TO_BALL", pwm, duration, reason, args), distance, angle, safety_reason)

    if abs_angle > args.center_tolerance_deg:
        forward = args.step_arc_forward_pwm
        if distance is not None and distance < args.slow_distance_m:
            forward = min(forward, args.creep_pwm)
        forward = clamp(forward * safety_scale, args.limit)
        turn = turn_dir * args.step_arc_turn_pwm
        duration = args.step_arc_s
        pwm = mix_pwm(forward, turn, args.limit)
        side = "right" if turn_dir > 0 else "left"
        return (
            *start_tank_step(
                mem,
                now,
                "STEP_ARC_TO_BALL",
                pwm,
                duration,
                f"{side} arc toward target {angle:.1f}deg",
                args,
            ),
            distance,
            angle,
            safety_reason,
        )

    forward = args.approach_pwm
    if distance is not None and distance < args.slow_distance_m:
        forward = min(forward, args.creep_pwm)
    forward = clamp(forward * safety_scale, args.limit)
    pwm = [forward, forward, forward, forward]
    return (*start_tank_step(mem, now, "STEP_FORWARD", pwm, args.step_forward_s, f"centered target {target.label}", args), distance, angle, safety_reason)


def decide_motion_tank(
    target: Optional[Detection],
    frame_w: int,
    frame_h: int,
    focal_px: float,
    lidar: LidarSnapshot,
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
    live: bool,
) -> tuple[str, list[int], str, Optional[float], Optional[float], str]:
    safety_ok, safety_reason, safety_scale = lidar_allows_motion(lidar, args, live)
    mem.wall_state, _wall_scale = side_wall_state(lidar, args)
    if mem.finished:
        return "STOP", [0, 0, 0, 0], "finished", None, None, safety_reason
    if not safety_ok:
        if lidar.ok and lidar.front_min_m is not None and lidar.front_min_m < args.lidar_stop_m:
            action, pwm, reason = tank_escape_pwm(lidar, args, safety_reason)
            return action, pwm, reason, None, None, safety_reason
        return "STOP", [0, 0, 0, 0], f"blocked: {safety_reason}", None, None, safety_reason

    if target is None:
        mem.state = "SEARCH"
        turn = args.search_pwm if (int(now / max(1.0, args.search_sweep_s)) % 2 == 0) else -args.search_pwm
        return "SEARCH_TURN", mix_pwm(0, turn, args.limit), "no ball target; sweeping", None, None, safety_reason

    mem.last_target_s = now
    angle = estimate_angle_deg(target.cx, frame_w, focal_px, args.angle_model)
    distance = estimate_distance_m(target.diameter_px, focal_px, args.ball_diameter_m)
    collect_ok, collect_reason = collect_reached(target, frame_w, frame_h, angle, distance, args)

    if collect_ok:
        if target.source == "held":
            mem.collect_since_s = 0.0
            mem.state = "APPROACH"
            return "STOP", [0, 0, 0, 0], f"held target reached ({collect_reason}); waiting for live detection", distance, angle, safety_reason
        if mem.state != "COLLECT":
            mem.state = "COLLECT"
            mem.collect_since_s = now
        if now - mem.collect_since_s >= args.collect_hold_s:
            mem.collected += 1
            mem.state = "SEARCH"
            mem.collect_since_s = 0.0
            return "STOP", [0, 0, 0, 0], f"ball counted {mem.collected}/{args.target_count} ({collect_reason})", distance, angle, safety_reason
        return "STOP", [0, 0, 0, 0], f"target reached; holding ({collect_reason})", distance, angle, safety_reason

    mem.state = "APPROACH"
    if abs(angle) > args.center_tolerance_deg:
        turn = signed_pwm(angle * args.turn_gain, args.min_turn_pwm, args.turn_pwm)
        return "TURN_TO_BALL", mix_pwm(0, turn, args.limit), f"target angle {angle:.1f}deg", distance, angle, safety_reason

    forward = args.approach_pwm
    if distance is not None and distance < args.slow_distance_m:
        forward = min(forward, args.creep_pwm)
    forward = clamp(forward * safety_scale, args.limit)
    turn = clamp(angle * args.forward_turn_gain, args.turn_pwm)
    return "APPROACH_BALL", mix_pwm(forward, turn, args.limit), f"centered target {target.label}", distance, angle, safety_reason


def decide_motion(
    target: Optional[Detection],
    home_target: Optional[dict],
    frame_w: int,
    frame_h: int,
    focal_px: float,
    lidar: LidarSnapshot,
    mem: MissionMemory,
    args: argparse.Namespace,
    now: float,
    live: bool,
    imu_sample=None,
) -> tuple[str, list[int], str, Optional[float], Optional[float], str]:
    if args.home_dock_enable and (
        mem.collected >= args.target_count
        or mem.home_dock is not None
        or mem.state.startswith("HOME_")
        or mem.state == "RELEASE_GATE"
    ):
        clear_tank_step(mem)
        return decide_home_dock_motion(home_target, lidar, mem, args, now, live, imu_sample)

    if args.drive_mode == "tank_step":
        return decide_motion_tank_step(target, frame_w, frame_h, focal_px, lidar, mem, args, now, live, imu_sample)
    if args.drive_mode == "tank":
        return decide_motion_tank(target, frame_w, frame_h, focal_px, lidar, mem, args, now, live)

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

    wall_state, wall_strafe, wall_scale = wall_hold_strafe(lidar, args)
    mem.wall_state = wall_state
    if not safety_ok:
        if lidar.ok and lidar.front_min_m is not None and lidar.front_min_m < args.lidar_stop_m:
            action, pwm, escape_reason = omni_escape_pwm(lidar, args, safety_reason)
            return action, pwm, escape_reason, None, None, safety_reason
        return "STOP", [0, 0, 0, 0], f"search blocked: {safety_reason}", None, None, safety_reason

    if wall_state in ("left-tight", "right-tight"):
        action, pwm, wall_reason = omni_wall_pwm(lidar, args, forward=-args.escape_pwm)
        return action, pwm, wall_reason, None, None, safety_reason

    if target is None:
        mem.state = "SEARCH"
        forward = args.explore_forward_pwm
        if lidar.front_min_m is not None and lidar.front_min_m < args.lidar_slow_m:
            forward = 0
        sweep = args.search_strafe_pwm if (int(now / max(1.0, args.search_sweep_s)) % 2 == 0) else -args.search_strafe_pwm
        strafe = wall_strafe if wall_strafe != 0 else sweep
        pwm = mecanum_pwm(forward, strafe, 0, args)
        return "SEARCH_OMNI", pwm, f"no ball; forward {forward:+d}, strafe {strafe:+d}", None, None, safety_reason

    mem.last_target_s = now
    angle = estimate_angle_deg(target.cx, frame_w, focal_px, args.angle_model)
    distance = estimate_distance_m(target.diameter_px, focal_px, args.ball_diameter_m)
    target_clearance = target_clearance_m(lidar, angle, args)
    collect_ok, collect_reason = collect_reached(target, frame_w, frame_h, angle, distance, args)

    if collect_ok:
        if target.source == "held":
            mem.collect_since_s = 0.0
            mem.state = "APPROACH"
            return "STOP", [0, 0, 0, 0], f"held target reached ({collect_reason}); waiting for live detection", distance, angle, safety_reason
        if mem.state != "COLLECT":
            mem.state = "COLLECT"
            mem.collect_since_s = now
        if now - mem.collect_since_s >= args.collect_hold_s:
            mem.collected += 1
            mem.state = "SEARCH"
            mem.collect_since_s = 0.0
            return "STOP", [0, 0, 0, 0], f"ball counted {mem.collected}/{args.target_count} ({collect_reason})", distance, angle, safety_reason
        return "STOP", [0, 0, 0, 0], f"target reached; holding ({collect_reason})", distance, angle, safety_reason

    mem.state = "APPROACH"
    if not safety_ok:
        if lidar.ok and lidar.front_min_m is not None and lidar.front_min_m < args.lidar_stop_m:
            action, pwm, escape_reason = omni_escape_pwm(lidar, args, safety_reason)
            return action, pwm, escape_reason, distance, angle, safety_reason
        return "STOP", [0, 0, 0, 0], f"approach blocked: {safety_reason}", distance, angle, safety_reason

    wall_risk = target_wall_penalty(lidar, angle, args)
    if target_clearance is not None and target_clearance < args.target_clear_m:
        mem.state = "WALL_SKIRT"
        strafe = wall_strafe
        if strafe == 0:
            strafe = -args.wall_strafe_pwm if angle > 0.0 else args.wall_strafe_pwm
        forward = args.wall_follow_pwm
        if lidar.front_min_m is not None and lidar.front_min_m < args.lidar_slow_m:
            forward = -args.escape_pwm if (lidar.rear_m is None or lidar.rear_m >= args.rear_stop_m) else 0
        pwm = mecanum_pwm(forward, strafe, 0, args)
        return "WALL_SKIRT", pwm, f"target path tight {target_clearance:.2f}m; forward {forward:+d}, strafe {strafe:+d}", distance, angle, safety_reason

    if wall_state in ("left-near", "right-near") and wall_risk > 0.0:
        action, pwm, wall_reason = omni_wall_pwm(lidar, args, forward=args.wall_follow_pwm)
        return action, pwm, f"{wall_reason}; target near wall", distance, angle, safety_reason

    forward = args.approach_pwm
    if distance is not None and distance < args.slow_distance_m:
        forward = min(forward, args.creep_pwm)
    if abs(angle) > args.forward_angle_slow_deg:
        forward = min(forward, args.creep_pwm)
    forward = clamp(forward * min(safety_scale, wall_scale), args.limit)
    camera_strafe = pd_strafe_command(angle, mem, args, now)
    strafe = clamp(camera_strafe + wall_strafe, args.strafe_pwm)
    turn = clamp(angle * args.omni_turn_gain, args.max_turn_correction_pwm)
    pwm = mecanum_pwm(forward, strafe, turn, args)
    return "APPROACH_OMNI", pwm, f"target {target.label}; forward {forward:+d}, strafe {strafe:+d}, turn {turn:+d}", distance, angle, safety_reason


def draw_overlay(
    frame,
    detections: list[Detection],
    target: Optional[Detection],
    action: str,
    reason: str,
    collected: int,
    args: argparse.Namespace,
    focal_px: float,
    home_detections=None,
    home_tags: Optional[list[dict]] = None,
    home_target: Optional[dict] = None,
    home_target_ids: Optional[set[int]] = None,
) -> bytes:
    out = frame.copy()
    h, w = out.shape[:2]
    cv2.line(out, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
    cv2.line(out, (0, h // 2), (w, h // 2), (50, 50, 50), 1)
    collect_y = int(h * args.collect_bottom_y_fraction)
    collect_x = int(w * args.collect_bottom_center_fraction)
    cv2.line(out, (w // 2 - collect_x, collect_y), (w // 2 + collect_x, collect_y), (80, 130, 80), 1)
    for det in detections:
        color = (0, 220, 255) if det.color == "yellow" else (220, 220, 220)
        if det is target:
            color = (40, 220, 80)
        p1 = (int(det.x1), int(det.y1))
        p2 = (int(det.x2), int(det.y2))
        cv2.rectangle(out, p1, p2, color, 2)
        distance = estimate_distance_m(det.diameter_px, focal_px, args.ball_diameter_m)
        angle = estimate_angle_deg(det.cx, w, focal_px, args.angle_model)
        label = f"{det.label} {det.conf:.2f} {angle:.0f}deg"
        if distance is not None:
            label += f" {distance:.2f}m"
        cv2.putText(out, label, (p1[0], max(18, p1[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    if home_detections and home_tags:
        target_ids = home_target_ids or set()
        for det, info in zip(home_detections, home_tags):
            corners = np.asarray(det.corners, dtype=np.int32).reshape(4, 2)
            is_target = int(info["id"]) in target_ids
            color = (40, 220, 80) if is_target else (150, 150, 150)
            cv2.polylines(out, [corners], True, color, 2, cv2.LINE_AA)
            cx, cy = info["center"]
            cv2.circle(out, (int(cx), int(cy)), 4, (0, 220, 255) if is_target else color, -1, cv2.LINE_AA)
            label = f"tag {info['id']} {info['angle_deg']:.0f}deg"
            if info.get("distance_m") is not None:
                label += f" {info['distance_m']:.2f}m"
            x, y = int(corners[:, 0].min()), int(corners[:, 1].min())
            cv2.putText(out, label, (max(0, x), max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1)
        if home_target is not None:
            cx, cy = home_target["center"]
            cv2.drawMarker(out, (int(cx), int(cy)), (0, 255, 255), cv2.MARKER_CROSS, 26, 2)

    cv2.putText(out, f"{action} balls {collected}/{args.target_count}", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (60, 220, 120), 2)
    cv2.putText(out, reason[:80], (12, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
    ok, jpeg = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
    return jpeg.tobytes() if ok else b""


def safety_imu_state(safety_snapshot=None, imu_sample=None) -> dict:
    kill_active = None
    kill_raw = None
    safety_io = "disabled"
    led = "--"
    if safety_snapshot is not None:
        safety_io = safety_snapshot.reason
        if safety_snapshot.enabled:
            kill_active = safety_snapshot.kill_active
            kill_raw = safety_snapshot.raw_level
            led = ("R" if safety_snapshot.red_on else "-") + ("G" if safety_snapshot.green_on else "-")

    imu = "disabled"
    imu_ok = False
    imu_yaw_deg = None
    imu_gyro_z = None
    imu_age_s = None
    if imu_sample is not None:
        imu_ok = imu_sample.ok
        if imu_sample.enabled:
            if imu_sample.ok:
                addr = f"0x{imu_sample.address:02x}" if imu_sample.address is not None else "--"
                imu = f"{imu_sample.sensor} i2c-{imu_sample.bus}@{addr}"
            else:
                imu = imu_sample.reason
            imu_yaw_deg = round(imu_sample.yaw_deg, 2) if imu_sample.yaw_deg is not None else None
            imu_gyro_z = round(imu_sample.gyro_z_rad_s, 5) if imu_sample.gyro_z_rad_s is not None else None
            imu_age_s = round(imu_sample.age_s, 3)
        else:
            imu = imu_sample.reason

    return {
        "kill_active": kill_active,
        "kill_raw": kill_raw,
        "safety_io": safety_io,
        "led": led,
        "imu": imu,
        "imu_ok": imu_ok,
        "imu_yaw_deg": imu_yaw_deg,
        "imu_gyro_z_rad_s": imu_gyro_z,
        "imu_age_s": imu_age_s,
    }


def imu_is_unsafe(args: argparse.Namespace, imu_sample) -> bool:
    if not args.require_imu:
        return False
    return (not imu_sample.enabled) or (not imu_sample.ok) or (imu_sample.age_s > args.imu_stale_seconds)


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
    safety_snapshot=None,
    imu_sample=None,
) -> dict:
    target_json = target.to_json(frame_w, focal_px, args.ball_diameter_m, args.angle_model) if target is not None else None
    target_angle = target_json["angle_deg"] if target_json else None
    target_clearance = target_clearance_m(lidar, target_angle, args) if target_json else None
    home_target = mem.home_target
    home_target_label = None
    home_target_distance = None
    home_target_angle = None
    if home_target is not None:
        home_target_label = f"{home_target.get('id')} / {home_target.get('wall')}"
        home_target_distance = home_target.get("distance_m")
        home_target_angle = home_target.get("angle_deg")
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
        "detections": [det.to_json(frame_w, focal_px, args.ball_diameter_m, args.angle_model) for det in detections[:8]],
        "target_label": target_json["label"] if target_json else None,
        "target_distance_m": target_json["distance_m"] if target_json else None,
        "target_angle_deg": target_angle,
        "target_clearance_m": round(target_clearance, 3) if target_clearance is not None else None,
        "home_dock_enabled": args.home_dock_enable,
        "home_tag_count": len(mem.home_tags),
        "home_distractor_count": mem.home_distractor_count,
        "home_target_label": home_target_label,
        "home_target_distance_m": round(home_target_distance, 3) if home_target_distance is not None else None,
        "home_target_angle_deg": round(home_target_angle, 2) if home_target_angle is not None else None,
        "release_done": mem.release_done,
        "release_status": mem.release_status,
        "front_min_m": round(lidar.front_min_m, 3) if lidar.front_min_m is not None else None,
        "front_left_m": round(lidar.front_left_m, 3) if lidar.front_left_m is not None else None,
        "front_right_m": round(lidar.front_right_m, 3) if lidar.front_right_m is not None else None,
        "left_m": round(lidar.left_m, 3) if lidar.left_m is not None else None,
        "right_m": round(lidar.right_m, 3) if lidar.right_m is not None else None,
        "wall_state": mem.wall_state,
        "lidar_age_s": round(lidar.age_s, 3),
        "lidar_points": lidar.point_count,
        "last_stop": shared.last_stop_reason,
        **safety_imu_state(safety_snapshot, imu_sample),
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
    if args.drive_mode == "omni":
        max_pwm = max(
            args.approach_pwm + args.strafe_pwm + args.max_turn_correction_pwm,
            args.creep_pwm + args.strafe_pwm + args.max_turn_correction_pwm,
            args.explore_forward_pwm + args.search_strafe_pwm + args.max_turn_correction_pwm,
            args.escape_pwm + args.wall_strafe_pwm + args.max_turn_correction_pwm,
            args.wall_follow_pwm + args.wall_strafe_pwm + args.max_turn_correction_pwm,
        )
    elif args.drive_mode == "tank_step":
        max_pwm = max(
            args.approach_pwm,
            args.creep_pwm,
            args.collect_drive_pwm,
            args.explore_forward_pwm,
            args.search_pwm,
            args.search_imu_recover_turn_pwm + args.search_imu_recover_reverse_pwm,
            args.turn_pwm,
            args.escape_pwm,
            args.step_arc_forward_pwm + args.step_arc_turn_pwm,
        )
    else:
        max_pwm = max(
            args.approach_pwm + args.turn_pwm,
            args.creep_pwm + args.turn_pwm,
            args.search_pwm,
            args.turn_pwm,
            args.escape_pwm,
        )
    if args.home_dock_enable:
        max_pwm = max(
            max_pwm,
            args.home_search_pwm,
            args.home_search_boost_pwm,
            args.home_align_turn_pwm,
            args.home_align_boost_pwm,
            args.home_approach_pwm,
            args.home_creep_pwm,
            args.home_kick_pwm,
            args.home_turn_180_pwm,
            args.home_turn_boost_pwm,
            args.home_backup_pwm,
            args.home_backup_force_pwm,
        )
    if max_pwm > args.limit:
        raise SystemExit("PWM values must not exceed --limit")

    shared = SharedState()
    server = start_server(shared, args.host, args.http_port)
    print(f"Ping-pong tracker monitor: http://{args.host}:{args.http_port}")
    if args.host in ("0.0.0.0", "::"):
        print(f"Jetson URL from Mac: http://192.168.55.1:{args.http_port}")

    model = None
    if args.detector in ("yolo", "onnx", "hybrid"):
        model_path = resolve_model_path(args.model)
        if args.detector == "onnx" or model_path.lower().endswith(".onnx"):
            print(f"Loading ONNX YOLO model: {model_path}")
            model = ("onnx", load_onnx(model_path))
        else:
            print(f"Loading Ultralytics YOLO model: {model_path}")
            model = ("yolo", load_yolo(model_path))

    cap = None
    stm = None
    safety_io = None
    imu_reader = None
    lidar_reader = LidarReader(args)
    mem = MissionMemory()
    start_s = time.monotonic()
    home_detector = None
    home_target_ids: set[int] = set()

    try:
        safety_io = open_safety_io(args, live)
        imu_reader = open_imu(args, live)
        cap = open_camera(args)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        focal_px = focal_from_hfov(actual_w, args.hfov_deg)
        print(f"Camera {args.camera}: {actual_w}x{actual_h} @ {actual_fps:.1f}fps, focal~{focal_px:.1f}px")

        if args.home_dock_enable:
            if apriltag is None or parse_apriltag_ids is None:
                print("WARN AprilTag home docking requested, but dt_apriltags module is unavailable.")
            else:
                home_target_ids = parse_apriltag_ids(args.home_tag_ids)
                home_detector = apriltag.Detector(
                    families=args.home_tag_family,
                    nthreads=args.home_tag_threads,
                    quad_decimate=args.home_tag_quad_decimate,
                    quad_sigma=args.home_tag_quad_sigma,
                    refine_edges=True,
                    decode_sharpening=args.home_tag_decode_sharpening,
                )
                print(f"Home AprilTag docking enabled: ids={sorted(home_target_ids)} size={args.home_tag_size_m:.3f}m")

        lidar_reader.open()

        if live:
            import serial

            stm = serial.Serial(args.stm32_port, args.stm32_baud, timeout=0.08)
            preflight_stm32(stm, args)
            if not args.no_lidar and not lidar_reader.ser:
                raise RuntimeError("Live motion requires LiDAR safety unless --no-lidar is set")
            initial_safety = safety_io.read()
            safety_io.update_leds(kill=initial_safety.kill_active, running=False)
            if initial_safety.enabled and initial_safety.kill_active:
                raise RuntimeError("Kill switch is active before enabling motors")
            enable_live_stm32(stm, args)
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
            frame = orient_frame(frame, args)

            safety_snapshot = safety_io.read()
            imu_sample = imu_reader.read()

            detections: list[Detection] = []
            if model is not None and args.detector in ("yolo", "onnx", "hybrid"):
                kind, loaded_model = model
                if kind == "onnx":
                    detections.extend(run_onnx_yolo(loaded_model, frame, args))
                else:
                    detections.extend(run_yolo(loaded_model, frame, args))
            if args.detector in ("color", "hybrid") and (args.detector == "color" or not detections):
                detections.extend(color_blob_detections(frame, args))

            lidar_snapshot = lidar_reader.read()
            target = find_target(detections, frame.shape[1], frame.shape[0], focal_px, lidar_snapshot, args)
            target = stabilize_target(target, mem, args, now)
            home_detections = []
            home_tags = []
            home_target = None
            home_distractor_count = 0
            if args.home_dock_enable and (
                mem.collected >= args.target_count
                or mem.home_dock is not None
                or mem.state.startswith("HOME_")
                or mem.state == "RELEASE_GATE"
            ):
                home_detections, home_tags, home_target, home_distractor_count = detect_home_tags(
                    frame,
                    home_detector,
                    home_target_ids,
                    args,
                    focal_px,
                )
            mem.home_tags = home_tags
            mem.home_target = home_target
            mem.home_distractor_count = home_distractor_count
            display_detections = list(detections)
            if target is not None and not any(det is target for det in display_detections):
                display_detections.append(target)
            action, pwm, reason, distance, angle, safety = decide_motion(
                target,
                home_target,
                frame.shape[1],
                frame.shape[0],
                focal_px,
                lidar_snapshot,
                mem,
                args,
                now,
                live,
                imu_sample,
            )
            forced_live_stop = False
            if safety_snapshot.enabled and safety_snapshot.kill_active:
                action, pwm, reason, safety = "STOP", [0, 0, 0, 0], "kill switch active", "kill switch active"
                shared.last_stop_reason = reason
                forced_live_stop = live
            elif imu_is_unsafe(args, imu_sample):
                reason = f"imu unsafe: {imu_sample.reason}"
                action, pwm, safety = "STOP", [0, 0, 0, 0], reason
                shared.last_stop_reason = reason
                forced_live_stop = live
            home_action = action.startswith("HOME_") or mem.state.startswith("HOME_") or mem.state == "RELEASE_GATE"
            if not forced_live_stop and not home_action:
                action, pwm, reason = apply_stuck_recovery(
                    action,
                    pwm,
                    reason,
                    target,
                    lidar_snapshot,
                    imu_sample,
                    mem,
                    args,
                    now,
                    live,
                )

            if action not in ("STOP", "RETURN_HOME") and not action.startswith("HOME_"):
                record_motion(mem, pwm, now)
            elif action == "STOP":
                close_active_segment(mem, now)

            if forced_live_stop:
                send_motion(stm, "STOP", [0, 0, 0, 0], live)
                mem.last_command_s = now
                mem.last_pwm = [0, 0, 0, 0]
            elif now - mem.last_command_s >= args.command_period:
                send_motion(stm, action, pwm, live)
                mem.last_command_s = now
                mem.last_pwm = list(pwm)

            if mem.state == "RELEASE_GATE" and action == "STOP":
                release_gate_once(mem, args, live)

            safety_snapshot = safety_io.update_leds(
                kill=safety_snapshot.enabled and safety_snapshot.kill_active,
                running=action != "STOP",
                warning=action == "STOP" or (imu_sample.enabled and not imu_sample.ok),
            )

            jpeg = draw_overlay(
                frame,
                display_detections,
                target,
                action,
                reason,
                mem.collected,
                args,
                focal_px,
                home_detections,
                home_tags,
                home_target,
                home_target_ids,
            )
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
                    display_detections,
                    frame.shape[1],
                    focal_px,
                    lidar_snapshot,
                    args,
                    shared,
                    safety_snapshot,
                    imu_sample,
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
                f"det={len(detections)} shown={len(display_detections)} target={target_desc} action={action} pwm={pwm} {reason}",
                flush=True,
            )
            if forced_live_stop:
                shared.stop_event.set()
            time.sleep(args.loop_sleep)

    except KeyboardInterrupt:
        shared.last_stop_reason = "keyboard interrupt"
        print("\nInterrupted")
    finally:
        stop_and_disable(stm)
        if stm is not None:
            stm.close()
        lidar_reader.close()
        if safety_io is not None:
            safety_io.close()
        if imu_reader is not None:
            imu_reader.close()
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
    parser.add_argument("--camera-rotate", type=int, choices=[0, 90, 180, 270], default=0)
    parser.add_argument("--flip-horizontal", action="store_true")
    parser.add_argument("--flip-vertical", action="store_true")
    parser.add_argument("--hfov-deg", type=float, default=105.0, help="Approximate horizontal FOV; many 120 deg USB modules quote diagonal FOV")
    parser.add_argument("--angle-model", choices=["pinhole", "linear"], default="linear", help="Use linear for wide-angle lenses with edge distortion")
    parser.add_argument("--ball-diameter-m", type=float, default=0.040)
    parser.add_argument("--detector", choices=["yolo", "onnx", "color", "hybrid"], default="hybrid")
    parser.add_argument("--model", default="auto", help="'auto', a .pt/.engine path, or an Ultralytics model name")
    parser.add_argument("--device", default="auto", help="Ultralytics device, e.g. cuda, 0, cpu, or auto")
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--yolo-conf", type=float, default=0.10)
    parser.add_argument("--onnx-nms-threshold", type=float, default=0.45)
    parser.add_argument("--onnx-box-format", choices=["auto", "xyxy", "xywh"], default="auto")
    parser.add_argument("--yolo-min-diameter-px", type=float, default=4.0)
    parser.add_argument("--yolo-max-diameter-px", type=float, default=160.0)
    parser.add_argument("--yolo-min-aspect", type=float, default=0.45)
    parser.add_argument("--yolo-max-aspect", type=float, default=2.20)
    parser.add_argument("--yolo-class-ids", default="32", help="Optional comma list, e.g. 32 for COCO sports ball")
    parser.add_argument(
        "--yolo-class-names",
        default="sports ball,ball,ping pong ball,ping-pong ball,tennis ball",
        help="Accepted YOLO class names when --yolo-class-ids is empty",
    )
    parser.add_argument("--accept-any-yolo-class", action="store_true")
    parser.add_argument("--require-ball-color", action="store_true")
    parser.add_argument("--min-color-ratio", type=float, default=0.08)
    parser.add_argument("--target-colors", default="orange,yellow,white", help="Comma list for color detector: orange,yellow,white")
    parser.add_argument("--prefer-color", choices=["any", "orange", "yellow", "white"], default="any")
    parser.add_argument("--color-min-area", type=float, default=35.0)
    parser.add_argument("--color-max-area-fraction", type=float, default=0.18)
    parser.add_argument("--color-min-circularity", type=float, default=0.45)
    parser.add_argument("--color-min-fill", type=float, default=0.38)
    parser.add_argument("--color-min-aspect", type=float, default=0.55)
    parser.add_argument("--color-max-aspect", type=float, default=1.8)
    parser.add_argument("--color-min-diameter-px", type=float, default=6.0)
    parser.add_argument("--color-max-diameter-px", type=float, default=180.0)
    parser.add_argument("--color-edge-margin-px", type=int, default=3)
    parser.add_argument("--roi-y-min-fraction", type=float, default=0.0)
    parser.add_argument("--roi-y-max-fraction", type=float, default=1.0)
    parser.add_argument("--target-count", type=int, default=3)
    parser.add_argument("--collect-distance-m", type=float, default=0.30)
    parser.add_argument("--collect-diameter-px", type=float, default=94.0)
    parser.add_argument("--collect-bottom-enable", action="store_true", default=True)
    parser.add_argument("--collect-bottom-disable", action="store_false", dest="collect_bottom_enable")
    parser.add_argument("--collect-bottom-y-fraction", type=float, default=0.72)
    parser.add_argument("--collect-bottom-edge-fraction", type=float, default=0.88)
    parser.add_argument("--collect-bottom-center-fraction", type=float, default=0.18)
    parser.add_argument("--collect-hold-s", type=float, default=0.45)
    parser.add_argument("--collect-drive-pwm", type=int, default=34, help="Forward PWM used to physically drive into a close centered ball")
    parser.add_argument("--collect-drive-s", type=float, default=0.55, help="Forward drive-in time before counting a close centered ball")
    parser.add_argument("--target-hold-s", type=float, default=0.60, help="Keep the last target briefly across dropped detections")
    parser.add_argument("--target-smooth-alpha", type=float, default=0.65, help="New-measurement weight for target box smoothing")
    parser.add_argument("--target-smooth-max-jump-px", type=float, default=140.0)
    parser.add_argument("--target-switch-lock-s", type=float, default=1.0, help="Keep a far-jumping target locked briefly")
    parser.add_argument("--target-cluster-enable", action="store_true", default=True, help="Allow nearby balls to become one group target")
    parser.add_argument("--target-cluster-disable", action="store_false", dest="target_cluster_enable")
    parser.add_argument("--target-cluster-min-count", type=int, default=2)
    parser.add_argument("--target-cluster-gap-px", type=float, default=120.0)
    parser.add_argument("--target-cluster-bonus", type=float, default=0.30)
    parser.add_argument("--center-tolerance-deg", type=float, default=6.0)
    parser.add_argument("--turn-in-place-deg", type=float, default=18.0)
    parser.add_argument("--slow-distance-m", type=float, default=0.55)
    parser.add_argument("--drive-mode", choices=["omni", "tank", "tank_step"], default="omni")
    parser.add_argument("--motor-sign", type=parse_signs, default=[1, -1, 1, -1], help="STM32 MOTOR_SIGN values for FL FR BL BR")
    parser.add_argument("--strafe-sign", type=int, choices=[-1, 1], default=1, help="Flip if positive strafe moves left instead of right")
    parser.add_argument("--approach-pwm", type=int, default=20)
    parser.add_argument("--creep-pwm", type=int, default=12)
    parser.add_argument("--turn-pwm", type=int, default=16)
    parser.add_argument("--kick-pwm", type=int, default=0, help="Short forward PWM burst to overcome static friction")
    parser.add_argument("--kick-turn-pwm", type=int, default=6, help="Turn component during the anti-stiction kick")
    parser.add_argument("--kick-s", type=float, default=0.0, help="Duration of the anti-stiction kick")
    parser.add_argument("--kick-cooldown-s", type=float, default=1.4)
    parser.add_argument("--min-turn-pwm", type=int, default=10)
    parser.add_argument("--search-pwm", type=int, default=28)
    parser.add_argument("--strafe-pwm", type=int, default=18)
    parser.add_argument("--min-strafe-pwm", type=int, default=8)
    parser.add_argument("--search-strafe-pwm", type=int, default=10)
    parser.add_argument("--explore-forward-pwm", type=int, default=10)
    parser.add_argument("--escape-pwm", type=int, default=12)
    parser.add_argument("--camera-strafe-gain", type=float, default=0.55)
    parser.add_argument("--camera-strafe-kd", type=float, default=0.04)
    parser.add_argument("--omni-turn-gain", type=float, default=0.0)
    parser.add_argument("--max-turn-correction-pwm", type=int, default=0)
    parser.add_argument("--turn-gain", type=float, default=1.0)
    parser.add_argument("--turn-kd", type=float, default=0.10)
    parser.add_argument("--forward-turn-gain", type=float, default=0.35)
    parser.add_argument("--forward-angle-slow-deg", type=float, default=10.0)
    parser.add_argument("--search-sweep-s", type=float, default=4.0)
    parser.add_argument("--search-use-imu", action="store_true", default=True)
    parser.add_argument("--disable-search-imu", action="store_false", dest="search_use_imu")
    parser.add_argument("--search-imu-turn-deg", type=float, default=135.0)
    parser.add_argument("--search-imu-step-s", type=float, default=0.75)
    parser.add_argument("--search-imu-stall-s", type=float, default=2.0)
    parser.add_argument("--search-imu-progress-epsilon-deg", type=float, default=4.0)
    parser.add_argument("--search-imu-recover-s", type=float, default=0.45)
    parser.add_argument("--search-imu-recover-turn-pwm", type=int, default=30)
    parser.add_argument("--search-imu-recover-reverse-pwm", type=int, default=14)
    parser.add_argument("--search-imu-alternate", action="store_true", default=True)
    parser.add_argument("--search-imu-one-way", action="store_false", dest="search_imu_alternate")
    parser.add_argument("--allow-imu-search-without-front-lidar", action="store_true", default=True)
    parser.add_argument("--disallow-imu-search-without-front-lidar", action="store_false", dest="allow_imu_search_without_front_lidar")
    parser.add_argument("--step-min-s", type=float, default=0.20)
    parser.add_argument("--step-max-s", type=float, default=0.75)
    parser.add_argument("--step-search-s", type=float, default=0.50)
    parser.add_argument("--step-forward-s", type=float, default=0.45)
    parser.add_argument("--step-arc-s", type=float, default=0.50)
    parser.add_argument("--step-arc-forward-pwm", type=int, default=34)
    parser.add_argument("--step-arc-turn-pwm", type=int, default=11)
    parser.add_argument("--step-spin-angle-deg", type=float, default=55.0)
    parser.add_argument("--step-turn-base-s", type=float, default=0.22)
    parser.add_argument("--step-turn-s-per-deg", type=float, default=0.008)
    parser.add_argument("--wall-follow-pwm", type=int, default=10)
    parser.add_argument("--wall-follow-turn-pwm", type=int, default=12)
    parser.add_argument("--return-duration-scale", type=float, default=0.90)
    parser.add_argument("--return-segment-max-s", type=float, default=1.20)
    parser.add_argument("--return-max-segments", type=int, default=160)
    parser.add_argument("--home-dock-enable", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--home-tag-ids", default="20,21")
    parser.add_argument("--home-tag-family", default="tag36h11")
    parser.add_argument("--home-tag-size-m", type=float, default=0.10)
    parser.add_argument("--home-tag-hold-s", type=float, default=1.80)
    parser.add_argument("--home-tag-threads", type=int, default=2)
    parser.add_argument("--home-tag-quad-decimate", type=float, default=1.5)
    parser.add_argument("--home-tag-quad-sigma", type=float, default=0.0)
    parser.add_argument("--home-tag-decode-sharpening", type=float, default=0.25)
    parser.add_argument("--home-dock-approach-m", type=float, default=0.20)
    parser.add_argument("--home-dock-distance-tolerance-m", type=float, default=0.04)
    parser.add_argument("--home-align-tolerance-deg", type=float, default=4.0)
    parser.add_argument("--home-align-gain", type=float, default=1.2)
    parser.add_argument("--home-forward-turn-gain", type=float, default=0.25)
    parser.add_argument("--home-max-forward-turn-pwm", type=int, default=5)
    parser.add_argument("--home-slow-approach-m", type=float, default=0.28)
    parser.add_argument("--home-search-pwm", type=int, default=34)
    parser.add_argument("--home-search-boost-pwm", type=int, default=45)
    parser.add_argument("--home-search-flip-s", type=float, default=4.0)
    parser.add_argument("--home-search-sweep-deg", type=float, default=180.0)
    parser.add_argument("--home-search-segment-deg", type=float, default=45.0)
    parser.add_argument("--home-search-stall-s", type=float, default=0.90)
    parser.add_argument("--home-search-progress-epsilon-deg", type=float, default=3.0)
    parser.add_argument("--home-search-alternate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--home-align-turn-pwm", type=int, default=28)
    parser.add_argument("--home-align-boost-pwm", type=int, default=45)
    parser.add_argument("--home-align-stall-s", type=float, default=0.80)
    parser.add_argument("--home-align-progress-epsilon-deg", type=float, default=2.0)
    parser.add_argument("--home-align-step-s", type=float, default=0.25)
    parser.add_argument("--home-min-turn-pwm", type=int, default=22)
    parser.add_argument("--home-approach-pwm", type=int, default=32)
    parser.add_argument("--home-creep-pwm", type=int, default=24)
    parser.add_argument("--home-kick-pwm", type=int, default=45)
    parser.add_argument("--home-kick-s", type=float, default=0.25)
    parser.add_argument("--home-kick-cooldown-s", type=float, default=1.4)
    parser.add_argument("--home-kick-min-front-scale", type=float, default=0.75)
    parser.add_argument("--home-step-min-s", type=float, default=0.20)
    parser.add_argument("--home-step-max-s", type=float, default=0.65)
    parser.add_argument("--home-step-forward-s", type=float, default=0.35)
    parser.add_argument("--home-front-stop-m", type=float, default=0.16)
    parser.add_argument("--home-front-slow-m", type=float, default=0.34)
    parser.add_argument("--home-missing-front-scale", type=float, default=0.70)
    parser.add_argument("--home-turn-direction", choices=["left", "right"], default="right")
    parser.add_argument("--home-turn-180-deg", type=float, default=180.0)
    parser.add_argument("--home-turn-180-pwm", type=int, default=30)
    parser.add_argument("--home-turn-boost-pwm", type=int, default=42)
    parser.add_argument("--home-turn-stall-s", type=float, default=1.60)
    parser.add_argument("--home-turn-progress-epsilon-deg", type=float, default=2.0)
    parser.add_argument("--home-turn-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--home-turn-slowdown-deg", type=float, default=35.0)
    parser.add_argument("--home-post-turn-pause-s", type=float, default=0.35)
    parser.add_argument("--home-backup-pwm", type=int, default=18)
    parser.add_argument("--home-backup-force-pwm", type=int, default=24)
    parser.add_argument("--home-backup-force-s", type=float, default=1.20)
    parser.add_argument("--home-backup-timeout-s", type=float, default=4.5)
    parser.add_argument("--home-rear-dock-m", type=float, default=0.13)
    parser.add_argument("--home-rear-min-settle-s", type=float, default=0.80)
    parser.add_argument("--home-dock-confirm-s", type=float, default=0.35)
    parser.add_argument("--home-rear-preturn-stop-m", type=float, default=0.16)
    parser.add_argument("--release-gate-after-dock", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--release-live-servo", action="store_true")
    parser.add_argument("--release-servo-pin", type=int, default=32)
    parser.add_argument("--release-servo-backend", choices=["hardware-pwm", "gpio-bitbang"], default="hardware-pwm")
    parser.add_argument("--release-servo-frequency-hz", type=float, default=50.0)
    parser.add_argument("--release-servo-home-us", type=int, default=1500)
    parser.add_argument("--release-servo-open-us", type=int, default=2000)
    parser.add_argument("--release-servo-min-us", type=int, default=900)
    parser.add_argument("--release-servo-max-us", type=int, default=2100)
    parser.add_argument("--release-servo-move-s", type=float, default=0.60)
    parser.add_argument("--release-servo-step-us", type=int, default=10)
    parser.add_argument("--release-servo-open-hold-s", type=float, default=7.0)
    parser.add_argument("--release-servo-home-hold-s", type=float, default=0.80)
    parser.add_argument("--release-servo-return-home", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--release-servo-detach-final", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lidar-port", default=LIDAR_PORT)
    parser.add_argument("--lidar-baud", type=int, default=LIDAR_BAUD)
    parser.add_argument("--front-center", type=float, default=270.0)
    parser.add_argument("--front-width", type=float, default=36.0)
    parser.add_argument("--diagonal-width", type=float, default=42.0)
    parser.add_argument("--side-width", type=float, default=70.0)
    parser.add_argument("--front-stat", choices=["min", "p10", "median"], default="p10")
    parser.add_argument("--side-stat", choices=["min", "p10", "median"], default="median")
    parser.add_argument("--body-width-m", type=float, default=0.20)
    parser.add_argument("--wall-margin-m", type=float, default=0.04)
    parser.add_argument("--wall-distance-m", type=float, default=0.32)
    parser.add_argument("--wall-track-m", type=float, default=0.85)
    parser.add_argument("--wall-deadband-m", type=float, default=0.04)
    parser.add_argument("--wall-hold-gain", type=float, default=85.0)
    parser.add_argument("--wall-strafe-pwm", type=int, default=18)
    parser.add_argument("--side-stop-m", type=float, default=0.18)
    parser.add_argument("--side-slow-m", type=float, default=0.28)
    parser.add_argument("--rear-stop-m", type=float, default=0.18)
    parser.add_argument("--target-clear-m", type=float, default=0.30)
    parser.add_argument("--target-side-angle-deg", type=float, default=18.0)
    parser.add_argument("--wall-target-penalty", type=float, default=0.85)
    parser.add_argument("--target-random-jitter", type=float, default=0.03)
    parser.add_argument("--lidar-stop-m", type=float, default=0.24)
    parser.add_argument("--lidar-slow-m", type=float, default=0.42)
    parser.add_argument("--lidar-stale-s", type=float, default=0.50)
    parser.add_argument("--allow-missing-front-lidar", action="store_true", default=True)
    parser.add_argument("--disallow-missing-front-lidar", action="store_false", dest="allow_missing_front_lidar")
    parser.add_argument("--lidar-missing-front-scale", type=float, default=0.45)
    parser.add_argument("--lidar-front-hold-s", type=float, default=0.30)
    parser.add_argument("--lidar-close-frames", type=int, default=2)
    parser.add_argument("--lidar-transient-scale", type=float, default=0.35)
    parser.add_argument("--ignore-raw-arc", action="append", type=parse_angle_arc, default=[])
    parser.add_argument("--ignore-robot-arc", action="append", type=parse_angle_arc, default=[])
    parser.add_argument("--no-lidar", action="store_true", help="Disable LiDAR gating; not recommended for live floor tests")
    parser.add_argument("--require-lidar-in-dry-run", action="store_true")
    parser.add_argument("--enable-stuck-recovery", action="store_true", default=True)
    parser.add_argument("--disable-stuck-recovery", action="store_false", dest="enable_stuck_recovery")
    parser.add_argument("--stuck-seconds", type=float, default=2.6)
    parser.add_argument("--stuck-recover-s", type=float, default=0.65)
    parser.add_argument("--stuck-min-pwm", type=int, default=18)
    parser.add_argument("--stuck-lidar-delta-m", type=float, default=0.035)
    parser.add_argument("--stuck-target-delta-px", type=float, default=18.0)
    parser.add_argument("--stuck-target-diameter-delta-px", type=float, default=7.0)
    parser.add_argument("--stuck-yaw-delta-deg", type=float, default=5.0)
    parser.add_argument("--stm32-port", default=STM32_PORT)
    parser.add_argument("--stm32-baud", type=int, default=STM32_BAUD)
    parser.add_argument("--allow-no-stm32-ack", action="store_true", help="allow one-way STM32 UART when commands work but replies are missing")
    parser.add_argument("--telemetry-hz", type=int, default=10)
    parser.add_argument("--limit", type=int, default=45)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8770)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--command-period", type=float, default=0.10)
    parser.add_argument("--loop-sleep", type=float, default=0.02)
    parser.add_argument("--jpeg-quality", type=int, default=82)
    add_safety_imu_args(parser)
    parser.add_argument("--live-motors", action="store_true")
    parser.add_argument("--ground-test", action="store_true")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
