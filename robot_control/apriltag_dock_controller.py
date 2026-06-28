#!/usr/bin/env python3
"""
AprilTag target-wall docking controller.

Default mode is dry-run: camera, AprilTag, LiDAR, IMU and the web monitor run,
but the STM32 is not enabled and no motor commands are sent. Live floor motion
requires both --live-motors and --ground-test.

Docking sequence:
  1. find target tags 20/21 among distractors
  2. align to the midpoint of 20 and 21, or the visible one
  3. approach to about 20 cm from the tag wall
  4. turn 180 degrees using IMU yaw
  5. reverse slowly until the rear LiDAR sector says the robot is close to wall
  6. stop and disable motors
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
from typing import Iterable, Optional

import cv2
import dt_apriltags as apriltag
import numpy as np

from apriltag_live_view import (
    choose_target,
    focal_from_hfov,
    open_camera,
    orient_frame,
    parse_tag_ids,
    tag_to_dict,
)
from jetson_safety_imu import add_safety_imu_args, open_imu, open_safety_io


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
  <title>AprilTag Dock</title>
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
      grid-template-columns: minmax(0, 1fr) 370px;
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
      max-width: 220px;
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
    <h1>AprilTag Dock</h1>
    <div class="row">
      <div class="status"><span id="dot" class="dot"></span><span id="status">connecting</span></div>
      <button id="stop">STOP</button>
    </div>
    <div id="action" class="action">--</div>
    <div id="reason" class="reason">--</div>
    <dl>
      <dt>Mode</dt><dd id="mode">--</dd>
      <dt>Runtime</dt><dd id="runtime">--</dd>
      <dt>State</dt><dd id="state">--</dd>
      <dt>Target IDs</dt><dd id="targetIds">--</dd>
      <dt>Tags</dt><dd id="tags">--</dd>
      <dt>Distractors</dt><dd id="distractors">--</dd>
      <dt>Target</dt><dd id="target">--</dd>
      <dt>Target distance</dt><dd id="targetDistance">--</dd>
      <dt>Target angle</dt><dd id="targetAngle">--</dd>
      <dt>LiDAR front</dt><dd id="front">--</dd>
      <dt>LiDAR rear</dt><dd id="rear">--</dd>
      <dt>LiDAR age</dt><dd id="lidarAge">--</dd>
      <dt>IMU</dt><dd id="imu">--</dd>
      <dt>Yaw</dt><dd id="yaw">--</dd>
      <dt>Turn progress</dt><dd id="turnProgress">--</dd>
      <dt>PWM</dt><dd id="pwm">--</dd>
      <dt>Last STOP</dt><dd id="lastStop">--</dd>
    </dl>
  </aside>
<script>
const ids = ["frame", "dot", "status", "action", "reason", "mode", "runtime",
  "state", "targetIds", "tags", "distractors", "target", "targetDistance",
  "targetAngle", "front", "rear", "lidarAge", "imu", "yaw", "turnProgress",
  "pwm", "lastStop", "stop"];
const el = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));

function fmtM(v) { return v == null ? "--" : `${v.toFixed(2)} m`; }
function fmtDeg(v) { return v == null ? "--" : `${v.toFixed(1)} deg`; }
function fmtMs(v) { return v == null ? "--" : `${Math.round(v * 1000)} ms`; }

async function update() {
  try {
    const res = await fetch("/state", { cache: "no-store" });
    const data = await res.json();
    el.dot.classList.toggle("ok", data.live_camera && !data.stop_requested);
    el.status.textContent = data.live_camera ? "live" : "waiting";
    el.action.textContent = data.action || "--";
    el.reason.textContent = data.reason || "--";
    el.mode.textContent = data.live_motors ? "LIVE" : "dry-run";
    el.runtime.textContent = `${data.runtime_s.toFixed(1)} s`;
    el.state.textContent = data.dock_state || "--";
    el.targetIds.textContent = data.target_ids.join(", ");
    el.tags.textContent = `${data.tag_count}`;
    el.distractors.textContent = `${data.distractor_count}`;
    el.target.textContent = data.target ? `${data.target.id} / ${data.target.wall}` : "--";
    el.targetDistance.textContent = data.target ? fmtM(data.target.distance_m) : "--";
    el.targetAngle.textContent = data.target ? fmtDeg(data.target.angle_deg) : "--";
    el.front.textContent = fmtM(data.front_m);
    el.rear.textContent = fmtM(data.rear_m);
    el.lidarAge.textContent = fmtMs(data.lidar_age_s);
    el.imu.textContent = data.imu || "--";
    el.yaw.textContent = fmtDeg(data.yaw_deg);
    el.turnProgress.textContent = data.turn_progress_deg == null ? "--" : `${data.turn_progress_deg.toFixed(1)} / ${data.turn_target_deg.toFixed(0)} deg`;
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
class LidarSnapshot:
    ok: bool = False
    age_s: float = 999.0
    front_m: Optional[float] = None
    rear_m: Optional[float] = None
    left_m: Optional[float] = None
    right_m: Optional[float] = None
    point_count: int = 0
    reason: str = "lidar not opened"


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
class DockMemory:
    state: str = "SEARCH_TAG"
    last_target: Optional[dict] = None
    last_target_s: float = 0.0
    search_dir: int = 1
    search_flip_s: float = 0.0
    search_yaw_start_deg: Optional[float] = None
    search_progress_deg: float = 0.0
    search_last_progress_deg: float = 0.0
    search_last_progress_s: float = 0.0
    yaw_start_deg: Optional[float] = None
    turn_progress_deg: float = 0.0
    turn_last_progress_deg: float = 0.0
    turn_last_progress_s: float = 0.0
    align_yaw_start_deg: Optional[float] = None
    align_progress_deg: float = 0.0
    align_last_progress_deg: float = 0.0
    align_last_progress_s: float = 0.0
    back_start_s: float = 0.0
    dock_close_since_s: float = 0.0
    phase_wait_until_s: float = 0.0
    last_pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    last_command_s: float = 0.0
    step_action: str = ""
    step_pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    step_until_s: float = 0.0
    step_reason: str = ""
    last_kick_s: float = 0.0


def clamp(value: int | float, limit: int) -> int:
    return max(-limit, min(limit, int(round(value))))


def signed_pwm(value: float, minimum: int, limit: int) -> int:
    pwm = clamp(value, limit)
    if pwm == 0:
        return 0
    if abs(pwm) < minimum:
        return minimum if pwm > 0 else -minimum
    return pwm


def mix_pwm(forward: int | float, turn_right: int | float, limit: int) -> list[int]:
    return [
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
    ]


def parse_signs(text: str) -> list[int]:
    parts = [part for part in text.replace(",", " ").split() if part]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected four signs, e.g. '1 -1 1 -1'")
    values = [int(part) for part in parts]
    if any(value not in (-1, 1) for value in values):
        raise argparse.ArgumentTypeError("motor signs must be -1 or 1")
    return values


def signed_delta_deg(start: float, current: float) -> float:
    return (current - start + 540.0) % 360.0 - 180.0


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


def sector_value(
    points: list[tuple[float, float, int]],
    front_center: float,
    center: float,
    half_width: float,
    stat: str,
) -> Optional[float]:
    values = sorted(
        dist
        for raw_angle, dist, _ in points
        if in_arc(robot_angle(raw_angle, front_center), center, half_width)
    )
    if not values:
        return None
    if stat == "min":
        return values[0]
    if stat == "median":
        return values[len(values) // 2]
    idx = max(0, min(len(values) - 1, int(round((len(values) - 1) * 0.10))))
    return values[idx]


class LidarReader:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.ser = None
        self.last_scan_s = 0.0
        self.last_snapshot = LidarSnapshot(reason="lidar disabled")

    def open(self) -> None:
        if self.args.no_lidar:
            return
        import serial

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
        now = time.monotonic()
        if raw:
            points = parse_points(raw)
            if points:
                self.last_scan_s = now
            age = now - self.last_scan_s if self.last_scan_s else 999.0
            front = sector_value(points, self.args.front_center, 0.0, self.args.front_width / 2.0, self.args.lidar_stat)
            rear = sector_value(points, self.args.front_center, 180.0, self.args.rear_width / 2.0, self.args.lidar_stat)
            left = sector_value(points, self.args.front_center, 270.0, self.args.side_width / 2.0, self.args.side_stat)
            right = sector_value(points, self.args.front_center, 90.0, self.args.side_width / 2.0, self.args.side_stat)
            ok = age <= self.args.lidar_stale_s
            self.last_snapshot = LidarSnapshot(
                ok=ok,
                age_s=age,
                front_m=front,
                rear_m=rear,
                left_m=left,
                right_m=right,
                point_count=len(points),
                reason="ok" if ok else f"lidar stale {age:.2f}s",
            )
            return self.last_snapshot
        age = now - self.last_scan_s if self.last_scan_s else 999.0
        self.last_snapshot.age_s = age
        self.last_snapshot.ok = age <= self.args.lidar_stale_s
        self.last_snapshot.reason = "ok" if self.last_snapshot.ok else f"lidar stale {age:.2f}s"
        return self.last_snapshot


def send_line(ser, line: str) -> None:
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()


def drain_lines(ser, seconds: float, show_raw: bool = False) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        lines.append(line)
        if show_raw:
            print("STM32", line)
    return lines


def command_expect(ser, command: str, ok_prefix: str, seconds: float, show_raw: bool = False) -> tuple[bool, list[str]]:
    send_line(ser, command)
    lines: list[str] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        lines.append(line)
        if show_raw:
            print("STM32", line)
        if line.startswith(ok_prefix):
            return True, lines
    return False, lines


def open_serial(port: str, baud: int):
    import serial

    return serial.Serial(port, baud, timeout=0.05)


def preflight_stm32(ser, args: argparse.Namespace) -> None:
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    drain_lines(ser, 0.20, args.show_raw)
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
        ok, lines = command_expect(ser, command, ok_prefix, seconds, args.show_raw)
        if ok:
            continue
        if args.allow_no_stm32_ack:
            print(f"WARN STM32 did not acknowledge {command}; replies={lines[-5:]}")
        else:
            raise RuntimeError(f"STM32 did not acknowledge {command}; replies={lines[-5:]}")


def stop_and_disable(ser) -> None:
    if ser is None:
        return
    try:
        send_line(ser, "STOP")
        send_line(ser, "ENABLE 0")
    except Exception:
        pass


def enable_live_stm32(ser, args: argparse.Namespace) -> None:
    ok, lines = command_expect(ser, "ENABLE 1", "OK ENABLE 1", 0.80, args.show_raw)
    if ok:
        return
    if args.allow_no_stm32_ack:
        print(f"WARN STM32 did not acknowledge ENABLE 1; replies={lines[-5:]}")
    else:
        raise RuntimeError(f"STM32 did not acknowledge ENABLE 1; replies={lines[-5:]}")


def send_motion(ser, action: str, pwm: list[int], live: bool, show_raw: bool = False) -> None:
    if not live:
        return
    if action == "STOP":
        send_line(ser, "STOP")
    else:
        send_line(ser, "PWM " + " ".join(str(int(v)) for v in pwm))
    drain_lines(ser, 0.015, show_raw)


def held_target(target: Optional[dict], mem: DockMemory, args: argparse.Namespace, now: float) -> Optional[dict]:
    if target is not None:
        mem.last_target = dict(target)
        mem.last_target_s = now
        return target
    if mem.last_target is not None and now - mem.last_target_s <= args.target_hold_s:
        held = dict(mem.last_target)
        held["id"] = f"held {held['id']}"
        return held
    return None


def lidar_live_ok(lidar: LidarSnapshot, args: argparse.Namespace, live: bool) -> tuple[bool, str]:
    if args.no_lidar:
        return True, "lidar disabled"
    if not lidar.ok:
        return (not live and not args.require_lidar_in_dry_run), lidar.reason
    return True, "ok"


def front_allows_approach(lidar: LidarSnapshot, args: argparse.Namespace) -> tuple[bool, str, float]:
    if args.no_lidar:
        return True, "lidar disabled", 1.0
    if lidar.front_m is None:
        return True, "front unknown", args.missing_front_scale
    if lidar.front_m < args.approach_front_stop_m:
        return False, f"front {lidar.front_m:.2f}m < hard {args.approach_front_stop_m:.2f}m", 0.0
    if lidar.front_m < args.approach_front_slow_m:
        span = max(0.01, args.approach_front_slow_m - args.approach_front_stop_m)
        return True, f"front caution {lidar.front_m:.2f}m", max(0.30, (lidar.front_m - args.approach_front_stop_m) / span)
    return True, "front clear", 1.0


def rear_allows_reverse(lidar: LidarSnapshot, args: argparse.Namespace) -> tuple[bool, str]:
    if args.no_lidar:
        return False, "rear LiDAR required for final reverse dock"
    if lidar.rear_m is None:
        return False, "rear unknown"
    if lidar.rear_m <= args.rear_dock_m:
        return False, f"rear dock distance {lidar.rear_m:.2f}m"
    return True, f"rear clear {lidar.rear_m:.2f}m"


def turn_pwm(direction: str, pwm: int, limit: int) -> list[int]:
    if direction == "left":
        return mix_pwm(0, -pwm, limit)
    return mix_pwm(0, pwm, limit)


def start_step(
    mem: DockMemory,
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


def current_step(mem: DockMemory, now: float) -> Optional[tuple[str, list[int], str]]:
    if not mem.step_action or mem.step_until_s <= now:
        return None
    remaining = max(0.0, mem.step_until_s - now)
    return mem.step_action, list(mem.step_pwm), f"{mem.step_reason}; step remaining {remaining:.2f}s"


def clear_step(mem: DockMemory) -> None:
    mem.step_action = ""
    mem.step_pwm = [0, 0, 0, 0]
    mem.step_until_s = 0.0
    mem.step_reason = ""


def reset_search_sweep(mem: DockMemory, yaw: Optional[float], now: float) -> None:
    mem.search_yaw_start_deg = yaw
    mem.search_progress_deg = 0.0
    mem.search_last_progress_deg = 0.0
    mem.search_last_progress_s = now
    mem.turn_progress_deg = 0.0


def reset_align_watch(mem: DockMemory, yaw: Optional[float], now: float) -> None:
    mem.align_yaw_start_deg = yaw
    mem.align_progress_deg = 0.0
    mem.align_last_progress_deg = 0.0
    mem.align_last_progress_s = now
    mem.turn_progress_deg = 0.0


def align_tag_motion(
    target: dict,
    imu_sample,
    mem: DockMemory,
    args: argparse.Namespace,
    now: float,
) -> tuple[str, list[int], str]:
    angle = float(target["angle_deg"])
    direction = 1 if angle > 0.0 else -1
    turn = signed_pwm(angle * args.align_gain, args.min_turn_pwm, args.align_turn_pwm)
    stalled = False
    progress = 0.0

    if getattr(imu_sample, "ok", False) and imu_sample.yaw_deg is not None:
        yaw = float(imu_sample.yaw_deg)
        if mem.state != "ALIGN_TAG" or mem.align_yaw_start_deg is None:
            reset_align_watch(mem, yaw, now)
        progress = abs(signed_delta_deg(mem.align_yaw_start_deg, yaw))
        mem.align_progress_deg = progress
        mem.turn_progress_deg = progress
        if progress - mem.align_last_progress_deg >= args.align_progress_epsilon_deg:
            mem.align_last_progress_deg = progress
            mem.align_last_progress_s = now
        stalled = now - mem.align_last_progress_s >= args.align_stall_s
        if stalled:
            turn = direction * max(abs(turn), args.align_boost_pwm)
            turn = clamp(turn, args.limit)
    else:
        mem.align_yaw_start_deg = None
        mem.align_progress_deg = 0.0
        mem.turn_progress_deg = 0.0

    mem.state = "ALIGN_TAG"
    pwm = mix_pwm(0, turn, args.limit)
    stall_note = f"; boost, yaw moved only {progress:.1f}deg" if stalled else ""
    return start_step(
        mem,
        now,
        "ALIGN_TAG",
        pwm,
        args.align_step_s,
        f"target {target['id']} angle {angle:+.1f}deg{stall_note}",
        args,
    )


def decide_search_tag_motion(
    lidar: LidarSnapshot,
    imu_sample,
    mem: DockMemory,
    args: argparse.Namespace,
    now: float,
) -> tuple[str, list[int], str]:
    clear_step(mem)
    mem.state = "SEARCH_TAG"
    if getattr(imu_sample, "ok", False) and imu_sample.yaw_deg is not None:
        yaw = float(imu_sample.yaw_deg)
        if mem.search_yaw_start_deg is None:
            reset_search_sweep(mem, yaw, now)
        progress = abs(signed_delta_deg(mem.search_yaw_start_deg, yaw))
        mem.search_progress_deg = progress
        mem.turn_progress_deg = progress
        if progress - mem.search_last_progress_deg >= args.search_progress_epsilon_deg:
            mem.search_last_progress_deg = progress
            mem.search_last_progress_s = now
        if progress >= max(args.search_segment_deg, args.search_sweep_deg):
            if args.search_alternate:
                mem.search_dir *= -1
            reset_search_sweep(mem, yaw, now)
            progress = 0.0

        stalled = now - mem.search_last_progress_s >= args.search_stall_s
        pwm_value = args.search_pwm
        if stalled:
            pwm_value = max(args.search_pwm, args.search_boost_pwm)
        direction = "right" if mem.search_dir > 0 else "left"
        segment = int(progress // max(1.0, args.search_segment_deg)) + 1
        segment_count = max(1, int(math.ceil(args.search_sweep_deg / max(1.0, args.search_segment_deg))))
        stall_note = "; boost, wheel/ground may be stuck" if stalled else ""
        pwm = turn_pwm(direction, pwm_value, args.limit)
        return (
            "SEARCH_TAG",
            pwm,
            f"target ids {args.target_tag_ids} not visible; {direction} imu sweep "
            f"{progress:.0f}/{args.search_sweep_deg:.0f}deg segment {segment}/{segment_count}{stall_note}",
        )

    mem.search_yaw_start_deg = None
    mem.turn_progress_deg = 0.0
    if now - mem.search_flip_s >= args.search_flip_s:
        mem.search_dir *= -1
        mem.search_flip_s = now
    direction = "right" if mem.search_dir > 0 else "left"
    pwm = turn_pwm(direction, args.search_pwm, args.limit)
    return "SEARCH_TAG", pwm, f"target ids {args.target_tag_ids} not visible; timed {direction} sweep, IMU unavailable"


def decide_motion(
    target: Optional[dict],
    lidar: LidarSnapshot,
    imu_sample,
    mem: DockMemory,
    args: argparse.Namespace,
    now: float,
    live: bool,
) -> tuple[str, list[int], str]:
    lidar_ok, lidar_reason = lidar_live_ok(lidar, args, live)
    if not lidar_ok:
        clear_step(mem)
        return "STOP", [0, 0, 0, 0], f"blocked: {lidar_reason}"

    if mem.state == "DOCKED":
        clear_step(mem)
        return "STOP", [0, 0, 0, 0], "dock complete"

    active = current_step(mem, now)
    if active is not None:
        action, pwm, reason = active
        if any(value > 0 for value in pwm):
            ok, front_reason, _scale = front_allows_approach(lidar, args)
            if not ok:
                clear_step(mem)
                return "STOP", [0, 0, 0, 0], front_reason
        return action, pwm, reason

    if mem.state == "TURN_180":
        clear_step(mem)
        if not getattr(imu_sample, "ok", False) or imu_sample.yaw_deg is None:
            return "STOP", [0, 0, 0, 0], f"waiting for IMU yaw: {getattr(imu_sample, 'reason', 'imu missing')}"
        if mem.yaw_start_deg is None:
            mem.yaw_start_deg = float(imu_sample.yaw_deg)
            mem.turn_progress_deg = 0.0
            mem.turn_last_progress_deg = 0.0
            mem.turn_last_progress_s = now
        delta = signed_delta_deg(mem.yaw_start_deg, float(imu_sample.yaw_deg))
        mem.turn_progress_deg = abs(delta)
        if (
            mem.turn_last_progress_s <= 0.0
            or mem.turn_progress_deg - mem.turn_last_progress_deg >= args.turn_progress_epsilon_deg
        ):
            mem.turn_last_progress_deg = mem.turn_progress_deg
            mem.turn_last_progress_s = now
        if mem.turn_progress_deg >= max(0.0, args.turn_180_deg - args.turn_tolerance_deg):
            mem.state = "BACK_TO_WALL"
            mem.back_start_s = now
            mem.phase_wait_until_s = now + args.post_turn_pause_s
            return "STOP", [0, 0, 0, 0], f"180 turn done {mem.turn_progress_deg:.1f}deg; preparing reverse dock"
        remaining = max(0.0, args.turn_180_deg - mem.turn_progress_deg)
        pwm = args.turn_180_pwm
        if remaining <= args.turn_slowdown_deg:
            pwm = max(args.min_turn_pwm, int(args.turn_180_pwm * 0.58))
        stalled = now - mem.turn_last_progress_s >= args.turn_stall_s
        if stalled and remaining > args.turn_tolerance_deg:
            pwm = max(pwm, args.turn_boost_pwm)
        stall_note = "; boost" if stalled else ""
        return "TURN_180", turn_pwm(args.turn_direction, pwm, args.limit), f"turning {args.turn_direction} {mem.turn_progress_deg:.1f}/{args.turn_180_deg:.0f}deg{stall_note}"

    if mem.state == "BACK_TO_WALL":
        clear_step(mem)
        if now < mem.phase_wait_until_s:
            return "STOP", [0, 0, 0, 0], "post-turn settle before reversing"
        reverse_elapsed = max(0.0, now - mem.phase_wait_until_s)
        if args.backup_force_s > 0.0 and reverse_elapsed < args.backup_force_s:
            pwm_value = max(args.backup_pwm, args.backup_force_pwm)
            pwm = [-pwm_value, -pwm_value, -pwm_value, -pwm_value]
            return (
                "BACK_TO_WALL_FORCE",
                pwm,
                f"forcing rear dock {reverse_elapsed:.1f}/{args.backup_force_s:.1f}s; "
                f"ignoring rear LiDAR={lidar.rear_m}",
            )
        if args.backup_force_s > 0.0:
            if mem.dock_close_since_s <= 0.0:
                mem.dock_close_since_s = now
                return (
                    "STOP",
                    [0, 0, 0, 0],
                    f"forced reverse complete; settling against wall rear={lidar.rear_m}",
                )
            if now - mem.dock_close_since_s >= args.dock_confirm_s:
                mem.state = "DOCKED"
                return "STOP", [0, 0, 0, 0], f"docked after forced reverse rear={lidar.rear_m}"
            return "STOP", [0, 0, 0, 0], f"confirming forced rear dock rear={lidar.rear_m}"
        if now - mem.back_start_s > args.backup_timeout_s:
            mem.state = "DOCKED"
            return "STOP", [0, 0, 0, 0], f"reverse dock timeout; stopped with rear={lidar.rear_m}"
        if lidar.rear_m is not None and lidar.rear_m <= args.rear_dock_m:
            elapsed = now - mem.back_start_s
            if elapsed < args.rear_min_settle_s:
                mem.dock_close_since_s = 0.0
                return (
                    "STOP",
                    [0, 0, 0, 0],
                    f"rear close {lidar.rear_m:.2f}m; settling {elapsed:.1f}/{args.rear_min_settle_s:.1f}s",
                )
            if mem.dock_close_since_s <= 0.0:
                mem.dock_close_since_s = now
            if now - mem.dock_close_since_s >= args.dock_confirm_s:
                mem.state = "DOCKED"
                return "STOP", [0, 0, 0, 0], f"docked: rear {lidar.rear_m:.2f}m <= {args.rear_dock_m:.2f}m"
            return "STOP", [0, 0, 0, 0], f"confirming rear contact {lidar.rear_m:.2f}m"
        mem.dock_close_since_s = 0.0
        ok, reason = rear_allows_reverse(lidar, args)
        if not ok:
            return "STOP", [0, 0, 0, 0], f"reverse blocked: {reason}"
        pwm = [-args.backup_pwm, -args.backup_pwm, -args.backup_pwm, -args.backup_pwm]
        return "BACK_TO_WALL", pwm, reason

    target = held_target(target, mem, args, now)
    if target is None:
        mem.align_yaw_start_deg = None
        mem.align_progress_deg = 0.0
        return decide_search_tag_motion(lidar, imu_sample, mem, args, now)

    if mem.state == "SEARCH_TAG":
        mem.search_yaw_start_deg = None
        mem.search_progress_deg = 0.0
        mem.search_last_progress_deg = 0.0
        mem.search_last_progress_s = now

    angle = target.get("angle_deg")
    distance = target.get("distance_m")
    if angle is None:
        clear_step(mem)
        return "STOP", [0, 0, 0, 0], "target angle unavailable"

    if abs(float(angle)) > args.align_tolerance_deg:
        return align_tag_motion(target, imu_sample, mem, args, now)

    mem.align_yaw_start_deg = None
    mem.align_progress_deg = 0.0
    if distance is None:
        clear_step(mem)
        return "STOP", [0, 0, 0, 0], "target aligned but distance unavailable"

    low = args.dock_approach_m - args.dock_distance_tolerance_m
    high = args.dock_approach_m + args.dock_distance_tolerance_m
    if float(distance) > high:
        mem.state = "APPROACH_TAG"
        ok, reason, scale = front_allows_approach(lidar, args)
        if not ok:
            clear_step(mem)
            return "STOP", [0, 0, 0, 0], reason
        forward = args.approach_pwm
        if float(distance) < args.slow_approach_m:
            forward = args.creep_pwm
        turn = clamp(float(angle) * args.forward_turn_gain, args.max_forward_turn_pwm)
        if (
            args.kick_pwm > 0
            and args.kick_s > 0.0
            and float(distance) >= args.slow_approach_m
            and now - mem.last_kick_s >= args.kick_cooldown_s
            and scale >= args.kick_min_front_scale
        ):
            kick_forward = clamp(args.kick_pwm * scale, args.limit)
            pwm = mix_pwm(kick_forward, turn, args.limit)
            mem.last_kick_s = now
            return start_step(
                mem,
                now,
                "KICK_APPROACH_TAG",
                pwm,
                args.kick_s,
                f"anti-stiction kick toward target {target['id']} {float(distance):.2f}m; {reason}",
                args,
            )
        pwm = mix_pwm(clamp(forward * scale, args.limit), turn, args.limit)
        return start_step(
            mem,
            now,
            "APPROACH_TAG",
            pwm,
            args.step_forward_s,
            f"target {target['id']} {float(distance):.2f}m; {reason}",
            args,
        )

    if float(distance) < low:
        clear_step(mem)
        mem.state = "BACK_UP_FROM_TAG"
        if lidar.rear_m is not None and lidar.rear_m < args.rear_preturn_stop_m:
            return "STOP", [0, 0, 0, 0], f"too close to tag but rear blocked {lidar.rear_m:.2f}m"
        pwm = [-args.creep_pwm, -args.creep_pwm, -args.creep_pwm, -args.creep_pwm]
        return "BACK_UP_FROM_TAG", pwm, f"target too close {float(distance):.2f}m; backing to {args.dock_approach_m:.2f}m"

    if not getattr(imu_sample, "ok", False) or imu_sample.yaw_deg is None:
        clear_step(mem)
        return "STOP", [0, 0, 0, 0], f"ready for 180 but IMU unavailable: {getattr(imu_sample, 'reason', 'imu missing')}"
    clear_step(mem)
    mem.state = "TURN_180"
    mem.yaw_start_deg = float(imu_sample.yaw_deg)
    mem.turn_progress_deg = 0.0
    mem.turn_last_progress_deg = 0.0
    mem.turn_last_progress_s = now
    return "STOP", [0, 0, 0, 0], f"aligned at {float(distance):.2f}m; starting 180 turn"


def draw_overlay(frame, detections: list, tags: list[dict], target: Optional[dict], action: str, reason: str, target_ids: set[int]) -> bytes:
    for det, info in zip(detections, tags):
        corners = np.asarray(det.corners, dtype=np.int32).reshape(4, 2)
        is_target = int(info["id"]) in target_ids
        color = (0, 255, 0) if is_target else (150, 150, 150)
        cv2.polylines(frame, [corners], True, color, 2, cv2.LINE_AA)
        center = tuple(np.asarray(det.center, dtype=np.int32).reshape(2))
        cv2.circle(frame, center, 4, (0, 200, 255) if is_target else (130, 130, 130), -1, cv2.LINE_AA)
        label = f"ID {info['id']} {info['wall']} {info['angle_deg']:.0f}deg"
        if info.get("distance_m") is not None:
            label += f" {info['distance_m']:.2f}m"
        x, y = int(corners[:, 0].min()), int(corners[:, 1].min())
        cv2.putText(frame, label, (max(0, x), max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)

    if target is not None:
        cx, cy = target["center"]
        cv2.drawMarker(frame, (int(cx), int(cy)), (0, 255, 255), cv2.MARKER_CROSS, 28, 2)
        cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (80, 120, 160), 1)
    cv2.putText(frame, action, (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(frame, reason[:90], (18, frame.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    return encoded.tobytes() if ok else b""


def build_state(
    start_s: float,
    action: str,
    reason: str,
    pwm: list[int],
    mem: DockMemory,
    target_ids: set[int],
    tags: list[dict],
    target: Optional[dict],
    distractor_count: int,
    lidar: LidarSnapshot,
    imu_sample,
    live: bool,
    shared: SharedState,
) -> dict:
    return {
        "runtime_s": round(time.monotonic() - start_s, 3),
        "live_camera": True,
        "live_motors": live,
        "stop_requested": shared.stop_event.is_set(),
        "dock_state": mem.state,
        "action": action,
        "reason": reason,
        "target_ids": sorted(target_ids),
        "tag_count": len(tags),
        "distractor_count": distractor_count,
        "tags": tags,
        "target": target,
        "front_m": round(lidar.front_m, 3) if lidar.front_m is not None else None,
        "rear_m": round(lidar.rear_m, 3) if lidar.rear_m is not None else None,
        "lidar_age_s": round(lidar.age_s, 3),
        "lidar_reason": lidar.reason,
        "imu": getattr(imu_sample, "sensor", "") or getattr(imu_sample, "reason", "imu disabled"),
        "yaw_deg": round(imu_sample.yaw_deg, 2) if getattr(imu_sample, "yaw_deg", None) is not None else None,
        "turn_progress_deg": round(mem.turn_progress_deg, 2),
        "turn_target_deg": 180.0,
        "pwm": pwm,
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
    live = args.live_motors and args.ground_test
    if args.live_motors != args.ground_test:
        raise SystemExit("Live movement requires both --live-motors and --ground-test")
    if live:
        args.require_imu = True
        args.enable_imu = True
    if max(abs(args.search_pwm), abs(args.search_boost_pwm), abs(args.align_turn_pwm), abs(args.align_boost_pwm), abs(args.approach_pwm), abs(args.creep_pwm), abs(args.kick_pwm), abs(args.turn_180_pwm), abs(args.turn_boost_pwm), abs(args.backup_pwm), abs(args.backup_force_pwm)) > args.limit:
        raise SystemExit("PWM parameters must not exceed --limit")

    target_ids = parse_tag_ids(args.target_tag_ids)
    shared = SharedState()
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
    lidar_reader = LidarReader(args)
    safety_io = None
    imu_reader = None
    stm = None
    mem = DockMemory()
    start_s = time.monotonic()
    last_print_s = 0.0
    action = "STOP"
    reason = "starting"
    pwm = [0, 0, 0, 0]
    try:
        cap = open_camera(args)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
        focal_px = focal_from_hfov(actual_w, args.hfov_deg)
        camera_params = (focal_px, focal_px, actual_w * 0.5, actual_h * 0.5)

        lidar_reader.open()
        safety_io = open_safety_io(args, live)
        imu_reader = open_imu(args, live)

        if live:
            stm = open_serial(args.stm32_port, args.stm32_baud)
            preflight_stm32(stm, args)
            first_safety = safety_io.read()
            if first_safety.enabled and first_safety.kill_active:
                raise RuntimeError("Kill switch active before enabling motors")
            enable_live_stm32(stm, args)

        print(f"AprilTag dock monitor: http://{args.host}:{args.http_port}")
        print(f"Jetson URL from Mac: http://192.168.55.1:{args.http_port}")
        print(
            f"mode={'LIVE GROUND MOTION' if live else 'DRY RUN'} camera={args.camera} "
            f"targets={sorted(target_ids)} tag_size={args.tag_size_m:.3f}m"
        )

        while not shared.stop_event.is_set():
            now = time.monotonic()
            if args.duration > 0 and now - start_s >= args.duration:
                shared.last_stop_reason = "duration elapsed"
                break

            ok, frame = cap.read()
            if not ok:
                action, pwm, reason = "STOP", [0, 0, 0, 0], "camera read failed"
                send_motion(stm, action, pwm, live, args.show_raw)
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
            target, _target_components, distractor_count = choose_target(tags, target_ids, frame.shape[1], focal_px)
            lidar = lidar_reader.read()
            imu_sample = imu_reader.read() if imu_reader is not None else type("ImuDisabled", (), {"ok": False, "yaw_deg": None, "sensor": "", "reason": "imu disabled"})()
            safety = safety_io.read() if safety_io is not None else None

            action, pwm, reason = decide_motion(target, lidar, imu_sample, mem, args, now, live)
            if safety is not None and safety.enabled and safety.kill_active:
                action, pwm, reason = "STOP", [0, 0, 0, 0], "kill switch active"
                shared.last_stop_reason = reason
                shared.stop_event.set()

            if now - mem.last_command_s >= args.command_period or pwm != mem.last_pwm:
                send_motion(stm, action, pwm, live, args.show_raw)
                mem.last_command_s = now
                mem.last_pwm = list(pwm)

            if safety_io is not None:
                safety_io.update_leds(
                    kill=safety is not None and safety.enabled and safety.kill_active,
                    running=live and action != "STOP",
                    warning=action == "STOP" or (getattr(imu_sample, "enabled", False) and not getattr(imu_sample, "ok", False)),
                )

            jpeg = draw_overlay(frame, detections, tags, target, action, reason, target_ids)
            shared.update(
                build_state(start_s, action, reason, pwm, mem, target_ids, tags, target, distractor_count, lidar, imu_sample, live, shared),
                jpeg,
            )

            if now - last_print_s >= args.print_period_s:
                target_desc = "none"
                if target is not None:
                    target_desc = f"{target['id']} {target['distance_m']}m {target['angle_deg']}deg"
                print(
                    f"t={now-start_s:05.1f}s state={mem.state} tags={len(tags)} target={target_desc} "
                    f"front={lidar.front_m} rear={lidar.rear_m} yaw={getattr(imu_sample, 'yaw_deg', None)} "
                    f"action={action} pwm={pwm} {reason}",
                    flush=True,
                )
                last_print_s = now
            time.sleep(args.loop_sleep)
    except KeyboardInterrupt:
        shared.last_stop_reason = "keyboard interrupt"
        return 130
    finally:
        if live:
            stop_and_disable(stm)
            print("Sent STOP and ENABLE 0 on exit")
        if stm is not None:
            stm.close()
        if imu_reader is not None:
            imu_reader.close()
        if safety_io is not None:
            safety_io.close()
        lidar_reader.close()
        if cap is not None:
            cap.release()
        server.shutdown()
        server.server_close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
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
    parser.add_argument("--target-hold-s", type=float, default=0.70)

    parser.add_argument("--dock-approach-m", type=float, default=0.20)
    parser.add_argument("--dock-distance-tolerance-m", type=float, default=0.04)
    parser.add_argument("--align-tolerance-deg", type=float, default=4.0)
    parser.add_argument("--align-gain", type=float, default=1.15)
    parser.add_argument("--forward-turn-gain", type=float, default=0.25)
    parser.add_argument("--max-forward-turn-pwm", type=int, default=5)
    parser.add_argument("--slow-approach-m", type=float, default=0.28)

    parser.add_argument("--search-pwm", type=int, default=34)
    parser.add_argument("--search-boost-pwm", type=int, default=45)
    parser.add_argument("--search-flip-s", type=float, default=4.0)
    parser.add_argument("--search-sweep-deg", type=float, default=180.0)
    parser.add_argument("--search-segment-deg", type=float, default=45.0)
    parser.add_argument("--search-stall-s", type=float, default=0.90)
    parser.add_argument("--search-progress-epsilon-deg", type=float, default=3.0)
    parser.add_argument("--search-alternate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--align-turn-pwm", type=int, default=32)
    parser.add_argument("--align-boost-pwm", type=int, default=45)
    parser.add_argument("--align-stall-s", type=float, default=0.80)
    parser.add_argument("--align-progress-epsilon-deg", type=float, default=2.0)
    parser.add_argument("--align-step-s", type=float, default=0.25)
    parser.add_argument("--min-turn-pwm", type=int, default=24)
    parser.add_argument("--approach-pwm", type=int, default=36)
    parser.add_argument("--creep-pwm", type=int, default=28)
    parser.add_argument("--kick-pwm", type=int, default=42)
    parser.add_argument("--kick-s", type=float, default=0.22)
    parser.add_argument("--kick-cooldown-s", type=float, default=1.60)
    parser.add_argument("--kick-min-front-scale", type=float, default=0.75)
    parser.add_argument("--step-min-s", type=float, default=0.20)
    parser.add_argument("--step-max-s", type=float, default=0.65)
    parser.add_argument("--step-forward-s", type=float, default=0.45)
    parser.add_argument("--turn-180-pwm", type=int, default=28)
    parser.add_argument("--turn-boost-pwm", type=int, default=42)
    parser.add_argument("--turn-stall-s", type=float, default=1.60)
    parser.add_argument("--turn-progress-epsilon-deg", type=float, default=2.0)
    parser.add_argument("--turn-direction", choices=["left", "right"], default="right")
    parser.add_argument("--turn-180-deg", type=float, default=180.0)
    parser.add_argument("--turn-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--turn-slowdown-deg", type=float, default=35.0)
    parser.add_argument("--post-turn-pause-s", type=float, default=0.35)
    parser.add_argument("--backup-pwm", type=int, default=16)
    parser.add_argument("--backup-force-pwm", type=int, default=24)
    parser.add_argument("--backup-force-s", type=float, default=1.20)
    parser.add_argument("--backup-timeout-s", type=float, default=4.5)
    parser.add_argument("--rear-dock-m", type=float, default=0.13)
    parser.add_argument("--rear-min-settle-s", type=float, default=0.80)
    parser.add_argument("--dock-confirm-s", type=float, default=0.35)
    parser.add_argument("--rear-preturn-stop-m", type=float, default=0.16)

    parser.add_argument("--lidar-port", default=LIDAR_PORT)
    parser.add_argument("--lidar-baud", type=int, default=LIDAR_BAUD)
    parser.add_argument("--front-center", type=float, default=270.0)
    parser.add_argument("--front-width", type=float, default=36.0)
    parser.add_argument("--rear-width", type=float, default=70.0)
    parser.add_argument("--side-width", type=float, default=70.0)
    parser.add_argument("--lidar-stat", choices=["min", "p10", "median"], default="median")
    parser.add_argument("--side-stat", choices=["min", "p10", "median"], default="median")
    parser.add_argument("--lidar-stale-s", type=float, default=0.60)
    parser.add_argument("--approach-front-stop-m", type=float, default=0.12)
    parser.add_argument("--approach-front-slow-m", type=float, default=0.30)
    parser.add_argument("--missing-front-scale", type=float, default=0.70)
    parser.add_argument("--no-lidar", action="store_true")
    parser.add_argument("--require-lidar-in-dry-run", action="store_true")

    parser.add_argument("--stm32-port", default=STM32_PORT)
    parser.add_argument("--stm32-baud", type=int, default=STM32_BAUD)
    parser.add_argument("--allow-no-stm32-ack", action="store_true")
    parser.add_argument("--telemetry-hz", type=int, default=10)
    parser.add_argument("--motor-sign", type=parse_signs, default=[1, -1, 1, -1])
    parser.add_argument("--limit", type=int, default=45)
    parser.add_argument("--show-raw", action="store_true")

    parser.add_argument("--quad-decimate", type=float, default=1.5)
    parser.add_argument("--quad-sigma", type=float, default=0.0)
    parser.add_argument("--decode-sharpening", type=float, default=0.25)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8773)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--command-period", type=float, default=0.10)
    parser.add_argument("--loop-sleep", type=float, default=0.02)
    parser.add_argument("--print-period-s", type=float, default=0.50)
    add_safety_imu_args(parser)
    parser.add_argument("--live-motors", action="store_true")
    parser.add_argument("--ground-test", action="store_true")
    return parser


if __name__ == "__main__":
    raise SystemExit(run(build_parser().parse_args()))
