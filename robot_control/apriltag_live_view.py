#!/usr/bin/env python3
"""
AprilTag live viewer for Unibots arena fiducials.

This diagnostic only opens the camera and a local HTTP monitor. It never opens
the STM32 port and never sends motor commands.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import dt_apriltags as apriltag
import numpy as np


WALL_MAP = {
    **{i: "North" for i in range(0, 6)},
    **{i: "East" for i in range(6, 12)},
    **{i: "South" for i in range(12, 18)},
    **{i: "West" for i in range(18, 24)},
}


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AprilTag Live</title>
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
      grid-template-columns: minmax(0, 1fr) 340px;
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
    .tag {
      font-size: 24px;
      font-weight: 750;
      margin: 12px 0 2px;
      line-height: 1.1;
    }
    .reason {
      min-height: 42px;
      color: #cbd5e1;
      font-size: 13px;
      line-height: 1.45;
    }
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
    <h1>AprilTag Live</h1>
    <div class="row">
      <div class="status"><span id="dot" class="dot"></span><span id="status">connecting</span></div>
      <button id="stop">STOP</button>
    </div>
    <div id="tag" class="tag">--</div>
    <div id="reason" class="reason">--</div>
    <dl>
      <dt>Runtime</dt><dd id="runtime">--</dd>
      <dt>Target IDs</dt><dd id="targetIds">--</dd>
      <dt>Tags</dt><dd id="count">--</dd>
      <dt>Distractors</dt><dd id="distractors">--</dd>
      <dt>Facing wall</dt><dd id="facing">--</dd>
      <dt>Dock action</dt><dd id="dockAction">--</dd>
      <dt>Closest ID</dt><dd id="closest">--</dd>
      <dt>Target</dt><dd id="target">--</dd>
      <dt>Target distance</dt><dd id="targetDistance">--</dd>
      <dt>Target angle</dt><dd id="targetAngle">--</dd>
      <dt>Distance</dt><dd id="distance">--</dd>
      <dt>Angle</dt><dd id="angle">--</dd>
      <dt>Area</dt><dd id="area">--</dd>
      <dt>Camera</dt><dd id="camera">--</dd>
      <dt>Age</dt><dd id="age">--</dd>
    </dl>
  </aside>
<script>
const ids = ["frame", "dot", "status", "tag", "reason", "runtime", "count",
  "targetIds", "distractors", "facing", "dockAction", "closest", "target",
  "targetDistance", "targetAngle", "distance", "angle", "area", "camera",
  "age", "stop"];
const el = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));

function fmtM(v) { return v == null ? "--" : `${v.toFixed(2)} m`; }
function fmtDeg(v) { return v == null ? "--" : `${v.toFixed(1)} deg`; }
function fmtAge(v) { return v == null ? "--" : `${Math.round(v * 1000)} ms`; }

async function update() {
  try {
    const res = await fetch("/state", { cache: "no-store" });
    const data = await res.json();
    el.dot.classList.toggle("ok", data.live_camera && !data.stop_requested);
    el.status.textContent = data.live_camera ? "live" : "waiting";
    el.runtime.textContent = `${data.runtime_s.toFixed(1)} s`;
    el.targetIds.textContent = data.target_ids.join(", ");
    el.count.textContent = `${data.tag_count}`;
    el.distractors.textContent = `${data.distractor_count}`;
    el.facing.textContent = data.facing_wall || "--";
    el.dockAction.textContent = data.dock_action || "--";
    el.closest.textContent = data.closest ? `${data.closest.id}` : "--";
    el.target.textContent = data.target ? `${data.target.id} / ${data.target.wall}` : "--";
    el.targetDistance.textContent = data.target ? fmtM(data.target.distance_m) : "--";
    el.targetAngle.textContent = data.target ? fmtDeg(data.target.angle_deg) : "--";
    el.distance.textContent = data.closest ? fmtM(data.closest.distance_m) : "--";
    el.angle.textContent = data.closest ? fmtDeg(data.closest.angle_deg) : "--";
    el.area.textContent = data.closest ? `${Math.round(data.closest.area_px2)} px2` : "--";
    el.camera.textContent = data.camera || "--";
    el.age.textContent = fmtAge(data.frame_age_s);
    if (data.target) {
      el.tag.textContent = `TARGET ${data.target.id} / ${data.target.wall}`;
      el.reason.textContent = data.dock_reason || data.target.note || "target detected";
    } else if (data.closest) {
      el.tag.textContent = `distractor ID ${data.closest.id} / ${data.closest.wall}`;
      el.reason.textContent = data.dock_reason || data.closest.note || "non-target detected";
    } else {
      el.tag.textContent = "--";
      el.reason.textContent = data.message || "no tag";
    }
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
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    stop_event: threading.Event = field(default_factory=threading.Event)
    latest: dict = field(default_factory=dict)
    jpeg: bytes = b""
    last_frame_s: float = 0.0

    def update(self, data: dict, jpeg: Optional[bytes] = None) -> None:
        with self.lock:
            self.latest = data
            self.last_frame_s = time.monotonic()
            if jpeg is not None:
                self.jpeg = jpeg

    def data(self) -> dict:
        with self.lock:
            return dict(self.latest)

    def frame(self) -> bytes:
        with self.lock:
            return bytes(self.jpeg)


def camera_arg(text: str):
    return int(text) if text.isdigit() else text


def parse_tag_ids(text: str) -> set[int]:
    values = set()
    for part in text.replace(",", " ").split():
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start = int(start_s)
            end = int(end_s)
            lo, hi = sorted((start, end))
            values.update(range(lo, hi + 1))
        else:
            values.add(int(part))
    return values


def focal_from_hfov(width_px: int, hfov_deg: float) -> float:
    hfov_rad = math.radians(max(20.0, min(170.0, hfov_deg)))
    return (width_px * 0.5) / math.tan(hfov_rad * 0.5)


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


def polygon_area(points: np.ndarray) -> float:
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    x = pts[:, 0]
    y = pts[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) * 0.5)


def tag_edge_px(corners: np.ndarray) -> float:
    pts = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    edges = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
    return float(sum(edges) / max(1, len(edges)))


def pose_distance(det, focal_px: float, tag_size_m: float) -> Optional[float]:
    pose_t = getattr(det, "pose_t", None)
    if pose_t is not None:
        try:
            z = float(np.asarray(pose_t).reshape(-1)[2])
            if 0.02 <= z <= 10.0:
                return z
        except Exception:
            pass
    edge = tag_edge_px(det.corners)
    if edge <= 1.0:
        return None
    return tag_size_m * focal_px / edge


def tag_to_dict(det, frame_w: int, focal_px: float, tag_size_m: float) -> dict:
    tag_id = int(det.tag_id)
    center = np.asarray(det.center, dtype=np.float32).reshape(2)
    angle = math.degrees(math.atan2(float(center[0]) - frame_w * 0.5, focal_px))
    distance = pose_distance(det, focal_px, tag_size_m)
    wall = WALL_MAP.get(tag_id, "Unknown")
    return {
        "id": tag_id,
        "wall": wall,
        "angle_deg": round(angle, 2),
        "distance_m": round(distance, 3) if distance is not None else None,
        "center": [round(float(center[0]), 1), round(float(center[1]), 1)],
        "area_px2": round(polygon_area(det.corners), 1),
        "note": f"tag36h11 size {tag_size_m * 1000:.0f} mm",
    }


def choose_target(tags: list[dict], target_ids: set[int], frame_w: int, focal_px: float) -> tuple[Optional[dict], list[dict], int]:
    target_tags = [tag for tag in tags if int(tag["id"]) in target_ids]
    distractor_count = max(0, len(tags) - len(target_tags))
    if not target_tags:
        return None, [], distractor_count

    by_id = {int(tag["id"]): tag for tag in target_tags}
    if 20 in by_id and 21 in by_id:
        pair = [by_id[20], by_id[21]]
        cx = sum(tag["center"][0] for tag in pair) * 0.5
        cy = sum(tag["center"][1] for tag in pair) * 0.5
        distances = [tag["distance_m"] for tag in pair if tag["distance_m"] is not None]
        distance = sum(distances) / len(distances) if distances else None
        angle = math.degrees(math.atan2(cx - frame_w * 0.5, focal_px))
        return (
            {
                "id": "20+21",
                "wall": "West",
                "angle_deg": round(angle, 2),
                "distance_m": round(distance, 3) if distance is not None else None,
                "center": [round(cx, 1), round(cy, 1)],
                "area_px2": round(sum(tag["area_px2"] for tag in pair), 1),
                "note": "midpoint between target tags 20 and 21",
            },
            pair,
            distractor_count,
        )

    visible = min(
        target_tags,
        key=lambda tag: tag["distance_m"] if tag["distance_m"] is not None else 99.0,
    )
    return visible, [visible], distractor_count


def dock_guidance(target: Optional[dict], args: argparse.Namespace) -> tuple[str, str]:
    if target is None:
        return "SEARCH_TARGET_TAG", f"target ids {args.target_tag_ids} not visible"
    angle = target.get("angle_deg")
    distance = target.get("distance_m")
    if angle is not None and abs(angle) > args.align_tolerance_deg:
        side = "RIGHT" if angle > 0.0 else "LEFT"
        return f"TURN_{side}_TO_TAG", f"target angle {angle:.1f}deg; align to 0deg first"
    if distance is None:
        return "HOLD_DISTANCE_UNKNOWN", "target aligned but distance is unavailable"
    low = args.dock_approach_m - args.dock_distance_tolerance_m
    high = args.dock_approach_m + args.dock_distance_tolerance_m
    if distance > high:
        return "APPROACH_TAG", f"target {distance:.2f}m away; approach to {args.dock_approach_m:.2f}m"
    if distance < low:
        return "BACK_UP_FROM_TAG", f"target {distance:.2f}m away; back up to {args.dock_approach_m:.2f}m"
    return "READY_FOR_180", f"aligned at {distance:.2f}m; next phase is IMU 180deg turn"


def draw_tags(frame, detections: list, tags: list[dict], target_ids: set[int]) -> bytes:
    for det, info in zip(detections, tags):
        corners = np.asarray(det.corners, dtype=np.int32).reshape(4, 2)
        is_target = int(info["id"]) in target_ids
        color = (0, 255, 0) if is_target else (160, 160, 160)
        cv2.polylines(frame, [corners], True, color, 2, cv2.LINE_AA)
        center = tuple(np.asarray(det.center, dtype=np.int32).reshape(2))
        cv2.circle(frame, center, 4, (0, 200, 255) if is_target else (130, 130, 130), -1, cv2.LINE_AA)
        label = f"ID {info['id']} {info['wall']}"
        if info["distance_m"] is not None:
            label += f" {info['distance_m']:.2f}m"
        label += f" {info['angle_deg']:.0f}deg"
        x, y = int(corners[:, 0].min()), int(corners[:, 1].min())
        cv2.putText(frame, label, (max(0, x), max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    return encoded.tobytes() if ok else b""


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
    shared = SharedState()
    target_ids = parse_tag_ids(args.target_tag_ids)
    server = start_server(shared, args.host, args.http_port)
    detector = apriltag.Detector(
        families=args.family,
        nthreads=max(1, args.threads),
        quad_decimate=args.quad_decimate,
        quad_sigma=args.quad_sigma,
        refine_edges=1,
        decode_sharpening=args.decode_sharpening,
    )
    cap = None
    start_s = time.monotonic()
    last_print_s = 0.0
    try:
        cap = open_camera(args)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
        focal_px = focal_from_hfov(actual_w, args.hfov_deg)
        camera_params = (focal_px, focal_px, actual_w * 0.5, actual_h * 0.5)
        print(f"AprilTag monitor: http://{args.host}:{args.http_port}")
        print(f"Jetson URL from Mac: http://192.168.55.1:{args.http_port}")
        print(f"Camera {args.camera}: {actual_w}x{actual_h}, family={args.family}, tag_size={args.tag_size_m:.3f}m")
        while not shared.stop_event.is_set():
            ok, frame = cap.read()
            now = time.monotonic()
            if not ok:
                time.sleep(0.05)
                continue
            frame = orient_frame(frame, args)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=args.tag_size_m,
            )
            tags = [tag_to_dict(det, frame.shape[1], focal_px, args.tag_size_m) for det in detections]
            tags.sort(key=lambda item: item["distance_m"] if item["distance_m"] is not None else 99.0)
            target, target_components, distractor_count = choose_target(tags, target_ids, frame.shape[1], focal_px)
            dock_action, dock_reason = dock_guidance(target, args)
            closest = tags[0] if tags else None
            facing = target["wall"] if target else (closest["wall"] if closest else None)
            data = {
                "runtime_s": round(now - start_s, 3),
                "live_camera": True,
                "stop_requested": shared.stop_event.is_set(),
                "camera": str(args.camera),
                "target_ids": sorted(target_ids),
                "tag_count": len(tags),
                "distractor_count": distractor_count,
                "tags": tags,
                "target": target,
                "target_components": target_components,
                "closest": closest,
                "facing_wall": facing,
                "dock_action": dock_action,
                "dock_reason": dock_reason,
                "frame_age_s": 0.0,
                "message": "show an AprilTag to the camera",
            }
            shared.update(data, draw_tags(frame, detections, [tag_to_dict(det, frame.shape[1], focal_px, args.tag_size_m) for det in detections], target_ids))
            if now - last_print_s >= args.print_period_s:
                if target:
                    print(
                        f"t={now - start_s:05.1f}s tags={len(tags)} target={target['id']} "
                        f"{target['wall']} {target['distance_m']}m {target['angle_deg']}deg "
                        f"action={dock_action} distractors={distractor_count}",
                        flush=True,
                    )
                elif closest:
                    print(
                        f"t={now - start_s:05.1f}s tags={len(tags)} "
                        f"closest_distractor=ID {closest['id']} {closest['wall']} "
                        f"{closest['distance_m']}m {closest['angle_deg']}deg action={dock_action}",
                        flush=True,
                    )
                else:
                    print(f"t={now - start_s:05.1f}s tags=0", flush=True)
                last_print_s = now
            time.sleep(args.loop_sleep)
    finally:
        if cap is not None:
            cap.release()
        server.shutdown()
        server.server_close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live AprilTag camera monitor. No motor output.")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--camera-rotate", type=int, choices=[0, 90, 180, 270], default=180)
    parser.add_argument("--flip-horizontal", action="store_true")
    parser.add_argument("--flip-vertical", action="store_true")
    parser.add_argument("--hfov-deg", type=float, default=105.0)
    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--tag-size-m", type=float, default=0.10)
    parser.add_argument("--target-tag-ids", default="20,21")
    parser.add_argument("--dock-approach-m", type=float, default=0.20)
    parser.add_argument("--dock-distance-tolerance-m", type=float, default=0.04)
    parser.add_argument("--align-tolerance-deg", type=float, default=4.0)
    parser.add_argument("--quad-decimate", type=float, default=1.5)
    parser.add_argument("--quad-sigma", type=float, default=0.0)
    parser.add_argument("--decode-sharpening", type=float, default=0.25)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8772)
    parser.add_argument("--loop-sleep", type=float, default=0.02)
    parser.add_argument("--print-period-s", type=float, default=0.8)
    return parser


if __name__ == "__main__":
    raise SystemExit(run(build_parser().parse_args()))
