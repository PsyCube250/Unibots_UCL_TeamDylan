#!/usr/bin/env python3
"""
Open-loop LiDAR exploration controller with a browser monitor.

Default mode is monitor/preflight only. Motor output requires both
--live-motors and --ground-test. On exit or monitor STOP it sends STOP and
ENABLE 0.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Iterable, Optional

import serial

from lidar_motor_dry_run import (
    BAUD as LIDAR_BAUD,
    LIDAR_PORT,
    choose,
    filter_points,
    parse_angle_arc,
    parse_points,
    sector_stats,
)
from jetson_safety_imu import add_safety_imu_args, open_imu, open_safety_io

STM32_PORT = os.environ.get("UNIBOTS_STM32_PORT", "/dev/ttyTHS1")
STM32_BAUD = 115200


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Explore Monitor</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0b1020;
      color: #e5e7eb;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 360px;
      background: #0b1020;
    }
    main {
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 14px;
    }
    canvas {
      width: min(92vmin, 100%);
      height: min(92vmin, 100%);
      max-height: calc(100vh - 28px);
      aspect-ratio: 1;
      background: #0f172a;
      border: 1px solid #1f2937;
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
    dd { margin: 0; font-variant-numeric: tabular-nums; text-align: right; }
    .action {
      font-size: 26px;
      font-weight: 750;
      margin: 12px 0 2px;
      line-height: 1.1;
    }
    .reason {
      min-height: 38px;
      color: #cbd5e1;
      font-size: 13px;
      line-height: 1.45;
    }
    .bar {
      height: 8px;
      background: #1f2937;
      margin-top: 8px;
      overflow: hidden;
    }
    .fill { height: 100%; width: 0%; background: #22c55e; }
    .warn { color: #f59e0b; }
    .bad { color: #ef4444; }
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
  <main><canvas id="scan" width="900" height="900"></canvas></main>
  <aside>
    <h1>Explore Monitor</h1>
    <div class="row">
      <div class="status"><span id="dot" class="dot"></span><span id="status">connecting</span></div>
      <button id="stop">STOP</button>
    </div>
    <div id="action" class="action">--</div>
    <div id="reason" class="reason">--</div>
    <div class="bar"><div id="marginFill" class="fill"></div></div>
    <dl>
      <dt>Mode</dt><dd id="mode">--</dd>
      <dt>Runtime</dt><dd id="runtime">--</dd>
      <dt>Points</dt><dd id="points">0</dd>
      <dt>Front min</dt><dd id="triad">--</dd>
      <dt>Front</dt><dd id="front">--</dd>
      <dt>Front left</dt><dd id="frontLeft">--</dd>
      <dt>Front right</dt><dd id="frontRight">--</dd>
      <dt>Left</dt><dd id="left">--</dd>
      <dt>Right</dt><dd id="right">--</dd>
      <dt>Rear</dt><dd id="rear">--</dd>
      <dt>PWM</dt><dd id="pwm">--</dd>
      <dt>LiDAR age</dt><dd id="age">--</dd>
      <dt>Kill</dt><dd id="kill">--</dd>
      <dt>LED</dt><dd id="led">--</dd>
      <dt>IMU</dt><dd id="imu">--</dd>
      <dt>Yaw</dt><dd id="yaw">--</dd>
      <dt>Gyro Z</dt><dd id="gyroZ">--</dd>
      <dt>Last STOP</dt><dd id="lastStop">--</dd>
    </dl>
  </aside>
  <script>
    const canvas = document.getElementById("scan");
    const ctx = canvas.getContext("2d");
    const ids = ["mode", "runtime", "points", "triad", "front", "frontLeft", "frontRight", "left", "right", "rear", "pwm", "age", "kill", "led", "imu", "yaw", "gyroZ", "lastStop"];
    const el = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));
    const actionEl = document.getElementById("action");
    const reasonEl = document.getElementById("reason");
    const statusEl = document.getElementById("status");
    const dotEl = document.getElementById("dot");
    const fillEl = document.getElementById("marginFill");
    const maxRange = 2.0;
    let lastSeen = 0;

    function fmt(v) { return v == null ? "--" : `${v.toFixed(3)} m`; }
    function fmtMs(v) { return v == null ? "--" : `${Math.round(v * 1000)} ms`; }

    function drawGrid() {
      const w = canvas.width, h = canvas.height, cx = w / 2, cy = h / 2, r = 380;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#0f172a";
      ctx.fillRect(0, 0, w, h);
      ctx.strokeStyle = "#334155";
      ctx.lineWidth = 1;
      ctx.font = "14px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillStyle = "#94a3b8";
      [0.1, 0.22, 0.45, 0.8, 1.2, 1.6, 2.0].forEach(m => {
        const rr = r * m / maxRange;
        ctx.beginPath();
        ctx.arc(cx, cy, rr, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillText(`${m.toFixed(2)}m`, cx + 7, cy - rr + 13);
      });
      [[0, "front"], [90, "right"], [180, "back"], [270, "left"]].forEach(([deg, label]) => {
        const rad = deg * Math.PI / 180;
        ctx.strokeStyle = "#475569";
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.sin(rad) * r, cy - Math.cos(rad) * r);
        ctx.stroke();
        ctx.fillStyle = "#e5e7eb";
        ctx.textAlign = "center";
        ctx.fillText(label, cx + Math.sin(rad) * (r + 33), cy - Math.cos(rad) * (r + 33));
      });
      const sectors = [[-18, 18, "#ef4444"], [18, 55, "#f59e0b"], [-55, -18, "#f59e0b"]];
      sectors.forEach(([a0, a1, color]) => {
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.55;
        ctx.beginPath();
        ctx.arc(cx, cy, r * 0.45 / maxRange, (a0 - 90) * Math.PI / 180, (a1 - 90) * Math.PI / 180);
        ctx.stroke();
        ctx.globalAlpha = 1;
      });
      ctx.fillStyle = "#38bdf8";
      ctx.beginPath();
      ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      ctx.fill();
    }

    function draw(data) {
      drawGrid();
      const cx = canvas.width / 2, cy = canvas.height / 2, scale = 380 / maxRange;
      for (const p of data.points || []) {
        const d = p[1];
        if (d > maxRange) continue;
        const rad = p[0] * Math.PI / 180;
        const x = cx + Math.sin(rad) * d * scale;
        const y = cy - Math.cos(rad) * d * scale;
        ctx.fillStyle = d < 0.10 ? "#b91c1c" : (d < 0.22 ? "#ef4444" : (d < 0.45 ? "#f59e0b" : "#22c55e"));
        ctx.globalAlpha = 0.78;
        ctx.beginPath();
        ctx.arc(x, y, 1.8, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.globalAlpha = 1;
    }

    function update(data) {
      draw(data);
      const s = data.sectors || {};
      actionEl.textContent = data.action || "--";
      reasonEl.textContent = data.reason || "--";
      el.mode.textContent = data.live ? "live" : "monitor";
      el.runtime.textContent = `${(data.runtime_s || 0).toFixed(1)} s`;
      el.points.textContent = (data.points || []).length;
      el.triad.textContent = fmt(data.front_min_m);
      el.front.textContent = fmt(s.front);
      el.frontLeft.textContent = fmt(s.front_left);
      el.frontRight.textContent = fmt(s.front_right);
      el.left.textContent = fmt(s.left);
      el.right.textContent = fmt(s.right);
      el.rear.textContent = fmt(s.rear);
      el.pwm.textContent = `[${(data.pwm || []).join(", ")}]`;
      el.age.textContent = fmtMs(data.lidar_age_s);
      el.kill.textContent = data.kill_active == null ? "--" : (data.kill_active ? "KILL" : "OK");
      el.led.textContent = data.led || "--";
      el.imu.textContent = data.imu || "--";
      el.yaw.textContent = data.imu_yaw_deg == null ? "--" : `${data.imu_yaw_deg.toFixed(1)} deg`;
      el.gyroZ.textContent = data.imu_gyro_z_rad_s == null ? "--" : `${data.imu_gyro_z_rad_s.toFixed(3)} rad/s`;
      el.lastStop.textContent = data.last_stop || "--";
      const margin = data.front_min_m == null ? 0 : Math.max(0, Math.min(1, data.front_min_m / 0.45));
      fillEl.style.width = `${margin * 100}%`;
      fillEl.style.background = margin < 0.25 ? "#ef4444" : (margin < 0.55 ? "#f59e0b" : "#22c55e");
      actionEl.className = "action";
      if ((data.front_min_m || 9) < 0.10) actionEl.classList.add("bad");
      else if ((data.front_min_m || 9) < 0.22) actionEl.classList.add("warn");
    }

    drawGrid();
    const events = new EventSource("/events");
    events.onmessage = ev => {
      lastSeen = Date.now();
      dotEl.classList.add("ok");
      statusEl.textContent = "live";
      update(JSON.parse(ev.data));
    };
    events.onerror = () => {
      dotEl.classList.remove("ok");
      statusEl.textContent = "reconnecting";
    };
    document.getElementById("stop").addEventListener("click", () => {
      fetch("/api/stop", {method: "POST"}).catch(() => {});
    });
    setInterval(() => {
      if (lastSeen && Date.now() - lastSeen > 1200) {
        dotEl.classList.remove("ok");
        statusEl.textContent = "stale";
      }
    }, 300);
  </script>
</body>
</html>
"""


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    stop_event: threading.Event = field(default_factory=threading.Event)
    snapshot: dict = field(default_factory=lambda: {"action": "STARTING", "points": []})
    last_stop_reason: str = ""

    def update(self, data: dict) -> None:
        with self.lock:
            self.snapshot = data

    def data(self) -> dict:
        with self.lock:
            return dict(self.snapshot)


@dataclass
class ControlMemory:
    action: str = "STOP"
    action_since_s: float = 0.0
    hold_action: str = ""
    hold_until_s: float = 0.0
    queued_action: str = ""
    queued_seconds: float = 0.0
    last_command_s: float = 0.0
    last_lidar_s: float = 0.0
    last_wander_s: float = 0.0
    wall_side: str = ""
    preferred_turn: str = ""
    preferred_turn_until_s: float = 0.0
    dfs_action: str = ""
    dfs_until_s: float = 0.0


def send_line(ser: serial.Serial, line: str) -> None:
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()


def drain_lines(ser: serial.Serial, seconds: float) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        lines.append(raw.decode("ascii", errors="replace").strip())
    return lines


def has_ok(lines: Iterable[str], prefix: str) -> bool:
    return any(line.startswith(prefix) for line in lines)


def preflight_stm32(ser: serial.Serial, args: argparse.Namespace) -> None:
    ser.reset_input_buffer()
    for command in ("STOP", "ENABLE 0", "PING", "STATUS", f"LIMIT {args.limit}", f"TELEM {args.telemetry_hz}"):
        send_line(ser, command)
        lines = drain_lines(ser, 0.35)
        if command == "PING" and not has_ok(lines, "OK PONG"):
            raise RuntimeError("STM32 did not answer PING")
        if command.startswith("LIMIT") and not has_ok(lines, f"OK LIMIT {args.limit}"):
            raise RuntimeError(f"STM32 did not accept LIMIT {args.limit}")


def read_scan(ser: serial.Serial, seconds: float) -> list[tuple[float, float, int]]:
    raw = bytearray()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw.extend(ser.read(4096))
    return parse_points(raw)


def stat_value(stats, name: str) -> Optional[float]:
    return choose(stats, name)


def min_present(values: Iterable[Optional[float]]) -> Optional[float]:
    present = [v for v in values if v is not None]
    return min(present) if present else None


def clamp(value: int, limit: int) -> int:
    return max(-limit, min(limit, int(value)))


def mix_pwm(forward: int, turn_right: int, limit: int) -> list[int]:
    return [
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
        clamp(forward + turn_right, limit),
        clamp(forward - turn_right, limit),
    ]


def turn_pwm(action: str, args: argparse.Namespace) -> list[int]:
    turn = args.turn_pwm if action == "TURN_RIGHT" else -args.turn_pwm
    return mix_pwm(0, turn, args.limit)


def action_pwm(action: str, args: argparse.Namespace) -> list[int]:
    if action == "BACKUP":
        return [-args.backup_pwm, -args.backup_pwm, -args.backup_pwm, -args.backup_pwm]
    if action == "BACKUP_LEFT":
        return mix_pwm(-args.backup_pwm, -args.turn_pwm, args.limit)
    if action == "BACKUP_RIGHT":
        return mix_pwm(-args.backup_pwm, args.turn_pwm, args.limit)
    if action in ("TURN_LEFT", "TURN_RIGHT"):
        return turn_pwm(action, args)
    return [0, 0, 0, 0]


def choose_turn(front_left: Optional[float], front_right: Optional[float], left: Optional[float], right: Optional[float]) -> str:
    fl = front_left if front_left is not None else 9.0
    fr = front_right if front_right is not None else 9.0
    if fl + 0.05 < fr:
        return "TURN_RIGHT"
    if fr + 0.05 < fl:
        return "TURN_LEFT"
    left_v = left if left is not None else 0.0
    right_v = right if right is not None else 0.0
    return "TURN_LEFT" if left_v >= right_v else "TURN_RIGHT"


def choose_escape_turn(
    mem: ControlMemory,
    front_left: Optional[float],
    front_right: Optional[float],
    left: Optional[float],
    right: Optional[float],
    args: argparse.Namespace,
    now: float,
) -> str:
    if mem.preferred_turn and now < mem.preferred_turn_until_s:
        return mem.preferred_turn
    action = choose_turn(front_left, front_right, left, right)
    mem.preferred_turn = action
    mem.preferred_turn_until_s = now + args.turn_stick_seconds
    return action


def hold(
    mem: ControlMemory,
    action: str,
    now: float,
    seconds: float,
    queued_action: str = "",
    queued_seconds: float = 0.0,
) -> None:
    mem.hold_action = action
    mem.hold_until_s = now + seconds
    mem.queued_action = queued_action
    mem.queued_seconds = queued_seconds


def front_guard_value(stats: dict, confirm_m: float) -> Optional[float]:
    guards = []
    for name in ("front", "front_left", "front_right"):
        sector = stats[name]
        if sector.min_m is not None and sector.p10_m is not None and sector.p10_m <= confirm_m:
            guards.append(sector.min_m)
    return min(guards) if guards else None


def update_wall_side(mem: ControlMemory, left: Optional[float], right: Optional[float], args: argparse.Namespace) -> None:
    if mem.wall_side == "left":
        if left is not None and left <= args.wall_lost_m:
            return
        mem.wall_side = ""
    if mem.wall_side == "right":
        if right is not None and right <= args.wall_lost_m:
            return
        mem.wall_side = ""

    candidates = []
    if left is not None and args.wall_min_m <= left <= args.wall_max_m:
        candidates.append(("left", abs(left - args.wall_target_m)))
    if right is not None and args.wall_min_m <= right <= args.wall_max_m:
        candidates.append(("right", abs(right - args.wall_target_m)))
    if candidates:
        mem.wall_side = min(candidates, key=lambda item: item[1])[0]


def dfs_pwm(action: str, args: argparse.Namespace) -> list[int]:
    if action == "DFS_LEFT":
        return mix_pwm(args.dfs_pwm, -args.dfs_turn_pwm, args.limit)
    if action == "DFS_RIGHT":
        return mix_pwm(args.dfs_pwm, args.dfs_turn_pwm, args.limit)
    return [args.dfs_pwm, args.dfs_pwm, args.dfs_pwm, args.dfs_pwm]


def choose_dfs_action(
    mem: ControlMemory,
    front: Optional[float],
    front_left: Optional[float],
    front_right: Optional[float],
    left: Optional[float],
    right: Optional[float],
    args: argparse.Namespace,
    now: float,
) -> Optional[tuple[str, list[int], str]]:
    front_clear = min_present((front, front_left, front_right))
    if front_clear is None or front_clear < args.dfs_min_forward_m:
        mem.dfs_action = ""
        return None

    if mem.dfs_action and now < mem.dfs_until_s:
        return mem.dfs_action, dfs_pwm(mem.dfs_action, args), f"dfs commit {mem.dfs_action.lower()}"

    straight_score = front_clear
    left_score = max(front_left or 0.0, (left or 0.0) * args.dfs_side_weight)
    right_score = max(front_right or 0.0, (right or 0.0) * args.dfs_side_weight)

    if straight_score >= args.dfs_straight_m:
        mem.dfs_action = "DFS_FORWARD"
        mem.dfs_until_s = now + args.dfs_forward_seconds
        return mem.dfs_action, dfs_pwm(mem.dfs_action, args), f"dfs forward corridor {straight_score:.2f}m"

    best_side = "DFS_LEFT" if left_score >= right_score else "DFS_RIGHT"
    best_side_score = max(left_score, right_score)
    if best_side_score >= front + args.dfs_branch_margin_m and best_side_score >= args.dfs_branch_m:
        mem.dfs_action = best_side
        mem.dfs_until_s = now + args.dfs_branch_seconds
        return mem.dfs_action, dfs_pwm(mem.dfs_action, args), f"dfs branch {best_side.lower()} score {best_side_score:.2f}m"

    mem.dfs_action = "DFS_FORWARD"
    mem.dfs_until_s = now + args.dfs_forward_seconds
    return mem.dfs_action, dfs_pwm(mem.dfs_action, args), f"dfs forward {straight_score:.2f}m"


def decide(stats: dict, mem: ControlMemory, args: argparse.Namespace, now: float, lidar_age_s: float) -> tuple[str, list[int], str]:
    front = stat_value(stats["front"], args.front_stat)
    fl = stat_value(stats["front_left"], args.front_stat)
    fr = stat_value(stats["front_right"], args.front_stat)
    left = stat_value(stats["left"], args.side_stat)
    right = stat_value(stats["right"], args.side_stat)
    front_decision = min_present((front, fl, fr))
    front_guard = front_guard_value(stats, args.guard_confirm_m)
    front_risk = min_present((front_guard, front_decision))

    if lidar_age_s > args.stale_seconds:
        return "STOP", [0, 0, 0, 0], f"lidar stale {lidar_age_s:.2f}s"
    if front_risk is None:
        return "STOP", [0, 0, 0, 0], "no front data"

    if mem.hold_action and now < mem.hold_until_s:
        action = mem.hold_action
        return action, action_pwm(action, args), f"held {action.lower()}"
    if mem.hold_action:
        mem.hold_action = ""

    escape_turn = choose_escape_turn(mem, fl, fr, left, right, args, now)
    if front_guard is not None and front_guard < args.min_clear_m:
        hold(mem, "BACKUP", now, args.backup_seconds, escape_turn, args.post_backup_turn_seconds)
        mem.dfs_action = ""
        return "BACKUP", action_pwm("BACKUP", args), f"front guard {front_guard:.2f}m < {args.min_clear_m:.2f}m"

    if front_risk < args.hard_stop_m:
        hold(mem, escape_turn, now, args.escape_turn_seconds)
        mem.dfs_action = ""
        return escape_turn, turn_pwm(escape_turn, args), f"front risk {front_risk:.2f}m < hard {args.hard_stop_m:.2f}m"

    if front_decision is not None and front_decision < args.creep_m:
        hold(mem, escape_turn, now, args.avoid_turn_seconds)
        mem.dfs_action = ""
        return escape_turn, turn_pwm(escape_turn, args), f"front p10 {front_decision:.2f}m < creep {args.creep_m:.2f}m"

    if front_decision is not None and front_decision < args.avoid_m:
        turn = args.creep_turn_pwm if escape_turn == "TURN_RIGHT" else -args.creep_turn_pwm
        action = "CREEP_RIGHT" if escape_turn == "TURN_RIGHT" else "CREEP_LEFT"
        return action, mix_pwm(args.creep_pwm, turn, args.limit), f"front p10 {front_decision:.2f}m, creeping {escape_turn.lower()}"

    if mem.queued_action:
        action = mem.queued_action
        seconds = mem.queued_seconds
        mem.queued_action = ""
        mem.queued_seconds = 0.0
        hold(mem, action, now, seconds)
        return action, action_pwm(action, args), f"post-recovery {action.lower()}"

    if left is not None and left < args.side_guard_m:
        mem.wall_side = ""
        return "SIDE_RIGHT", mix_pwm(args.side_push_pwm, args.side_push_turn_pwm, args.limit), f"left side {left:.2f}m < guard {args.side_guard_m:.2f}m"
    if right is not None and right < args.side_guard_m:
        mem.wall_side = ""
        return "SIDE_LEFT", mix_pwm(args.side_push_pwm, -args.side_push_turn_pwm, args.limit), f"right side {right:.2f}m < guard {args.side_guard_m:.2f}m"

    dfs_decision = choose_dfs_action(mem, front, fl, fr, left, right, args, now)
    if dfs_decision is not None:
        return dfs_decision

    update_wall_side(mem, left, right, args)

    forward = args.kick_pwm if now - mem.action_since_s < args.kick_seconds else args.cruise_pwm
    if mem.wall_side == "left" and left is not None:
        error = args.wall_target_m - left
        turn = clamp(round(error * args.wall_gain), args.trim_pwm)
        return "WALL_LEFT", mix_pwm(forward, turn, args.limit), f"left wall {left:.2f}m target {args.wall_target_m:.2f}m"
    if mem.wall_side == "right" and right is not None:
        error = right - args.wall_target_m
        turn = clamp(round(error * args.wall_gain), args.trim_pwm)
        return "WALL_RIGHT", mix_pwm(forward, turn, args.limit), f"right wall {right:.2f}m target {args.wall_target_m:.2f}m"

    if now - mem.last_wander_s > args.wander_period_s:
        mem.last_wander_s = now
        action = "TURN_RIGHT" if (int(now) // int(max(1, args.wander_period_s))) % 2 == 0 else "TURN_LEFT"
        hold(mem, action, now, args.wander_turn_seconds)
        return action, turn_pwm(action, args), "wander turn"

    return "FORWARD", [forward, forward, forward, forward], f"front clear {front_decision:.2f}m"


def send_motion(stm: serial.Serial, action: str, pwm: list[int], live: bool) -> None:
    if not live:
        return
    if action == "STOP":
        send_line(stm, "STOP")
    else:
        send_line(stm, "PWM " + " ".join(str(v) for v in pwm))
    drain_lines(stm, 0.02)


def stop_and_disable(stm: Optional[serial.Serial]) -> None:
    if stm is None:
        return
    try:
        send_line(stm, "STOP")
        send_line(stm, "ENABLE 0")
    except Exception:
        pass


def make_handler(shared: SharedState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path == "/" or self.path == "/index.html":
                body = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                last_seq = None
                while not shared.stop_event.is_set():
                    data = shared.data()
                    seq = data.get("seq")
                    if seq != last_seq:
                        payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
                        try:
                            self.wfile.write(b"data: " + payload + b"\n\n")
                            self.wfile.flush()
                        except BrokenPipeError:
                            break
                        last_seq = seq
                    time.sleep(0.08)
                return
            if self.path == "/api/state":
                body = json.dumps(shared.data()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_error(404)

        def do_POST(self):
            if self.path == "/api/stop":
                shared.last_stop_reason = "browser stop"
                shared.stop_event.set()
                self.send_response(204)
                self.end_headers()
                return
            self.send_error(404)

    return Handler


def start_server(shared: SharedState, host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), make_handler(shared))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


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


def build_snapshot(
    seq: int,
    start_s: float,
    live: bool,
    front_center: float,
    points: list[tuple[float, float, int]],
    stats: dict,
    action: str,
    pwm: list[int],
    reason: str,
    lidar_age_s: float,
    front_min: Optional[float],
    shared: SharedState,
    safety_snapshot=None,
    imu_sample=None,
) -> dict:
    robot_points = []
    for raw_angle, dist, _ in points[:5000]:
        robot_points.append([round((raw_angle - front_center) % 360.0, 2), round(dist, 3)])
    sectors = {
        "front": stat_value(stats["front"], "p10"),
        "front_left": stat_value(stats["front_left"], "p10"),
        "front_right": stat_value(stats["front_right"], "p10"),
        "left": stat_value(stats["left"], "median"),
        "right": stat_value(stats["right"], "median"),
        "rear": stat_value(stats["rear"], "median"),
    }
    return {
        "seq": seq,
        "live": live,
        "runtime_s": round(time.monotonic() - start_s, 2),
        "points": robot_points,
        "sectors": {k: (round(v, 3) if v is not None else None) for k, v in sectors.items()},
        "front_min_m": round(front_min, 3) if front_min is not None else None,
        "action": action,
        "pwm": pwm,
        "reason": reason,
        "lidar_age_s": round(lidar_age_s, 3),
        "last_stop": shared.last_stop_reason,
        **safety_imu_state(safety_snapshot, imu_sample),
    }


def run(args: argparse.Namespace) -> int:
    live = args.live_motors and args.ground_test
    if args.live_motors != args.ground_test:
        raise SystemExit("Ground motion requires both --live-motors and --ground-test")
    max_mixed_pwm = max(
        args.cruise_pwm + args.trim_pwm,
        args.creep_pwm + args.creep_turn_pwm,
        args.side_push_pwm + args.side_push_turn_pwm,
        args.dfs_pwm + args.dfs_turn_pwm,
        args.turn_pwm,
        args.backup_pwm,
        args.kick_pwm,
    )
    if max_mixed_pwm > args.limit:
        raise SystemExit("PWM values must not exceed --limit")

    shared = SharedState()
    server = start_server(shared, args.host, args.http_port)
    print(f"Explore monitor: http://{args.host}:{args.http_port}")
    if args.host in ("0.0.0.0", "::"):
        print(f"Jetson URL from Mac: http://192.168.55.1:{args.http_port}")

    lidar: Optional[serial.Serial] = None
    stm: Optional[serial.Serial] = None
    safety_io = None
    imu_reader = None
    mem = ControlMemory(action="STOP", action_since_s=time.monotonic(), last_wander_s=time.monotonic())
    seq = 0
    start_s = time.monotonic()

    try:
        safety_io = open_safety_io(args, live)
        imu_reader = open_imu(args, live)
        lidar = serial.Serial(args.lidar_port, args.lidar_baud, timeout=0.03)
        stm = serial.Serial(args.stm32_port, args.stm32_baud, timeout=0.08)
        preflight_stm32(stm, args)
        if live:
            initial_safety = safety_io.read()
            safety_io.update_leds(kill=initial_safety.kill_active, running=False)
            if initial_safety.enabled and initial_safety.kill_active:
                raise RuntimeError("Kill switch is active before enabling motors")
            send_line(stm, "ENABLE 1")
            drain_lines(stm, 0.25)
            print("LIVE motors enabled. Browser STOP or Ctrl-C stops.")
        else:
            print("Monitor only. Motors not enabled.")

        while not shared.stop_event.is_set():
            if args.duration > 0 and time.monotonic() - start_s >= args.duration:
                shared.last_stop_reason = "duration elapsed"
                break
            raw_points = read_scan(lidar, args.scan_seconds)
            now = time.monotonic()
            if raw_points:
                mem.last_lidar_s = now
            lidar_age_s = now - mem.last_lidar_s if mem.last_lidar_s else 999.0
            safety_snapshot = safety_io.read()
            imu_sample = imu_reader.read()
            points = filter_points(raw_points, args.front_center, args.ignore_raw_arc, args.ignore_robot_arc)

            stats = {
                "front": sector_stats(points, args.front_center, 0.0, args.front_width / 2.0),
                "front_left": sector_stats(points, args.front_center, 315.0, args.diagonal_width / 2.0),
                "front_right": sector_stats(points, args.front_center, 45.0, args.diagonal_width / 2.0),
                "left": sector_stats(points, args.front_center, 270.0, args.side_width / 2.0),
                "right": sector_stats(points, args.front_center, 90.0, args.side_width / 2.0),
                "rear": sector_stats(points, args.front_center, 180.0, args.side_width / 2.0),
            }
            front_guard = front_guard_value(stats, args.guard_confirm_m)
            front_decision = min_present((
                stat_value(stats["front"], args.front_stat),
                stat_value(stats["front_left"], args.front_stat),
                stat_value(stats["front_right"], args.front_stat),
            ))
            front_min = min_present((front_guard, front_decision))
            action, pwm, reason = decide(stats, mem, args, now, lidar_age_s)
            forced_live_stop = False
            if safety_snapshot.enabled and safety_snapshot.kill_active:
                action, pwm, reason = "STOP", [0, 0, 0, 0], "kill switch active"
                shared.last_stop_reason = reason
                forced_live_stop = live
            elif imu_is_unsafe(args, imu_sample):
                action, pwm, reason = "STOP", [0, 0, 0, 0], f"imu unsafe: {imu_sample.reason}"
                shared.last_stop_reason = reason
                forced_live_stop = live

            if action != mem.action:
                mem.action = action
                mem.action_since_s = now
            if forced_live_stop:
                send_motion(stm, "STOP", [0, 0, 0, 0], live)
                mem.last_command_s = now
            elif now - mem.last_command_s >= args.command_period:
                send_motion(stm, action, pwm, live)
                mem.last_command_s = now

            safety_snapshot = safety_io.update_leds(
                kill=safety_snapshot.enabled and safety_snapshot.kill_active,
                running=action != "STOP",
                warning=action == "STOP" or (imu_sample.enabled and not imu_sample.ok),
            )

            seq += 1
            shared.update(build_snapshot(
                seq,
                start_s,
                live,
                args.front_center,
                points,
                stats,
                action,
                pwm,
                reason,
                lidar_age_s,
                front_min,
                shared,
                safety_snapshot,
                imu_sample,
            ))
            print(
                f"t={time.monotonic() - start_s:05.2f}s front_min={front_min if front_min is not None else -1:.3f} "
                f"action={action} pwm={pwm} {reason}",
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
        if lidar is not None:
            lidar.close()
        if safety_io is not None:
            safety_io.close()
        if imu_reader is not None:
            imu_reader.close()
        shared.last_stop_reason = shared.last_stop_reason or "program exit"
        data = shared.data()
        data["action"] = "STOP"
        data["pwm"] = [0, 0, 0, 0]
        data["last_stop"] = shared.last_stop_reason
        shared.update(data)
        server.shutdown()
        server.server_close()
        print("Sent STOP and ENABLE 0 on exit")

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lidar-port", default=LIDAR_PORT)
    parser.add_argument("--lidar-baud", type=int, default=LIDAR_BAUD)
    parser.add_argument("--stm32-port", default=STM32_PORT)
    parser.add_argument("--stm32-baud", type=int, default=STM32_BAUD)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--front-center", type=float, default=270.0)
    parser.add_argument("--front-width", type=float, default=36.0)
    parser.add_argument("--diagonal-width", type=float, default=42.0)
    parser.add_argument("--side-width", type=float, default=70.0)
    parser.add_argument("--front-stat", choices=["min", "p10", "median"], default="p10")
    parser.add_argument("--side-stat", choices=["min", "p10", "median"], default="median")
    parser.add_argument("--min-clear-m", type=float, default=0.12)
    parser.add_argument("--guard-confirm-m", type=float, default=0.18)
    parser.add_argument("--hard-stop-m", type=float, default=0.25)
    parser.add_argument("--avoid-m", type=float, default=0.50)
    parser.add_argument("--creep-m", type=float, default=0.38)
    parser.add_argument("--wall-target-m", type=float, default=0.28)
    parser.add_argument("--wall-min-m", type=float, default=0.16)
    parser.add_argument("--wall-max-m", type=float, default=0.62)
    parser.add_argument("--wall-lost-m", type=float, default=0.90)
    parser.add_argument("--wall-gain", type=float, default=30.0)
    parser.add_argument("--side-guard-m", type=float, default=0.12)
    parser.add_argument("--dfs-min-forward-m", type=float, default=0.58)
    parser.add_argument("--dfs-straight-m", type=float, default=0.68)
    parser.add_argument("--dfs-branch-m", type=float, default=0.72)
    parser.add_argument("--dfs-branch-margin-m", type=float, default=0.16)
    parser.add_argument("--dfs-side-weight", type=float, default=0.75)
    parser.add_argument("--stale-seconds", type=float, default=0.50)
    parser.add_argument("--scan-seconds", type=float, default=0.10)
    parser.add_argument("--loop-sleep", type=float, default=0.025)
    parser.add_argument("--command-period", type=float, default=0.10)
    parser.add_argument("--telemetry-hz", type=int, default=10)
    parser.add_argument("--limit", type=int, default=45)
    parser.add_argument("--cruise-pwm", type=int, default=33)
    parser.add_argument("--creep-pwm", type=int, default=32)
    parser.add_argument("--creep-turn-pwm", type=int, default=8)
    parser.add_argument("--side-push-pwm", type=int, default=24)
    parser.add_argument("--side-push-turn-pwm", type=int, default=18)
    parser.add_argument("--dfs-pwm", type=int, default=40)
    parser.add_argument("--dfs-turn-pwm", type=int, default=4)
    parser.add_argument("--turn-pwm", type=int, default=32)
    parser.add_argument("--backup-pwm", type=int, default=30)
    parser.add_argument("--trim-pwm", type=int, default=8)
    parser.add_argument("--kick-pwm", type=int, default=45)
    parser.add_argument("--kick-seconds", type=float, default=0.22)
    parser.add_argument("--backup-seconds", type=float, default=0.75)
    parser.add_argument("--post-backup-turn-seconds", type=float, default=0.55)
    parser.add_argument("--escape-turn-seconds", type=float, default=0.85)
    parser.add_argument("--avoid-turn-seconds", type=float, default=0.45)
    parser.add_argument("--turn-stick-seconds", type=float, default=1.20)
    parser.add_argument("--dfs-forward-seconds", type=float, default=1.80)
    parser.add_argument("--dfs-branch-seconds", type=float, default=1.40)
    parser.add_argument("--wander-period-s", type=float, default=20.0)
    parser.add_argument("--wander-turn-seconds", type=float, default=0.35)
    parser.add_argument("--ignore-raw-arc", action="append", type=parse_angle_arc, default=[])
    parser.add_argument("--ignore-robot-arc", action="append", type=parse_angle_arc, default=[])
    add_safety_imu_args(parser)
    parser.add_argument("--live-motors", action="store_true")
    parser.add_argument("--ground-test", action="store_true")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
