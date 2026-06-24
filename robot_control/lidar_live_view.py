#!/usr/bin/env python3
"""
Continuous STL-27L LiDAR live viewer.

The server only reads the LiDAR serial port. It never opens the STM32 port and
never writes motor commands. Open the printed URL in a browser to see a live
robot-frame polar view.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Iterable, Optional

import serial

DEFAULT_PORT = os.environ.get("UNIBOTS_LIDAR_PORT", "/dev/cu.usbserial-0001")
DEFAULT_BAUD = 921600
DEFAULT_FRONT_CENTER = 270.0


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LiDAR Live</title>
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
      grid-template-columns: minmax(0, 1fr) 320px;
      background: #0b1020;
    }
    main {
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 16px;
    }
    canvas {
      width: min(92vmin, 100%);
      height: min(92vmin, 100%);
      max-height: calc(100vh - 32px);
      aspect-ratio: 1;
      background: #0f172a;
      border: 1px solid #1f2937;
    }
    aside {
      border-left: 1px solid #1f2937;
      padding: 18px 16px;
      background: #111827;
      min-height: 100vh;
    }
    h1 {
      font-size: 18px;
      margin: 0 0 16px;
      font-weight: 650;
    }
    .status {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      font-size: 13px;
      padding: 6px 9px;
      border: 1px solid #374151;
      margin-bottom: 18px;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 999px;
      background: #ef4444;
    }
    .dot.ok { background: #22c55e; }
    dl {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px 12px;
      margin: 0;
      font-size: 14px;
    }
    dt { color: #9ca3af; }
    dd { margin: 0; font-variant-numeric: tabular-nums; }
    .small {
      margin-top: 18px;
      color: #9ca3af;
      font-size: 12px;
      line-height: 1.45;
    }
    @media (max-width: 820px) {
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
  <main>
    <canvas id="scan" width="900" height="900"></canvas>
  </main>
  <aside>
    <h1>LiDAR Live</h1>
    <div class="status"><span id="dot" class="dot"></span><span id="status">connecting</span></div>
    <dl>
      <dt>Points</dt><dd id="points">0</dd>
      <dt>Front</dt><dd id="front">--</dd>
      <dt>Front left</dt><dd id="frontLeft">--</dd>
      <dt>Front right</dt><dd id="frontRight">--</dd>
      <dt>Left</dt><dd id="left">--</dd>
      <dt>Right</dt><dd id="right">--</dd>
      <dt>Back</dt><dd id="back">--</dd>
      <dt>Decision</dt><dd id="decision">--</dd>
      <dt>Age</dt><dd id="age">--</dd>
    </dl>
    <p class="small">
      Robot frame: front is up, right is clockwise 90 degrees. Red points are under
      0.4 m, amber under 0.8 m, green is clear. This viewer only reads LiDAR.
    </p>
  </aside>
  <script>
    const canvas = document.getElementById("scan");
    const ctx = canvas.getContext("2d");
    const ids = ["points", "front", "frontLeft", "frontRight", "left", "right", "back", "decision", "age"];
    const els = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));
    const statusEl = document.getElementById("status");
    const dotEl = document.getElementById("dot");
    const maxRange = 2.0;
    let latest = null;
    let lastSeen = 0;

    function fmt(v) {
      return v == null ? "--" : `${v.toFixed(3)} m`;
    }

    function drawGrid() {
      const w = canvas.width;
      const h = canvas.height;
      const cx = w / 2;
      const cy = h / 2;
      const r = 380;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#0f172a";
      ctx.fillRect(0, 0, w, h);
      ctx.strokeStyle = "#334155";
      ctx.lineWidth = 1;
      ctx.font = "14px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillStyle = "#94a3b8";
      [0.2, 0.4, 0.8, 1.2, 1.6, 2.0].forEach(m => {
        const rr = r * m / maxRange;
        ctx.beginPath();
        ctx.arc(cx, cy, rr, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillText(`${m.toFixed(1)}m`, cx + 8, cy - rr + 14);
      });
      const labels = [
        [0, "front"],
        [90, "right"],
        [180, "back"],
        [270, "left"],
      ];
      ctx.strokeStyle = "#475569";
      ctx.fillStyle = "#e5e7eb";
      ctx.font = "16px ui-monospace, SFMono-Regular, Menlo, monospace";
      labels.forEach(([deg, label]) => {
        const rad = deg * Math.PI / 180;
        const x = cx + Math.sin(rad) * r;
        const y = cy - Math.cos(rad) * r;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(x, y);
        ctx.stroke();
        ctx.textAlign = "center";
        ctx.fillText(label, cx + Math.sin(rad) * (r + 32), cy - Math.cos(rad) * (r + 32));
      });
      ctx.fillStyle = "#38bdf8";
      ctx.beginPath();
      ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      ctx.fill();
    }

    function drawScan(data) {
      drawGrid();
      const cx = canvas.width / 2;
      const cy = canvas.height / 2;
      const scale = 380 / maxRange;
      for (const p of data.points) {
        const d = p[1];
        if (d > maxRange) continue;
        const rad = p[0] * Math.PI / 180;
        const x = cx + Math.sin(rad) * d * scale;
        const y = cy - Math.cos(rad) * d * scale;
        ctx.fillStyle = d < 0.4 ? "#ef4444" : (d < 0.8 ? "#f59e0b" : "#22c55e");
        ctx.globalAlpha = 0.78;
        ctx.beginPath();
        ctx.arc(x, y, 1.7, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.globalAlpha = 1;
    }

    function updateText(data) {
      els.points.textContent = data.points.length;
      els.front.textContent = fmt(data.sectors.front);
      els.frontLeft.textContent = fmt(data.sectors.front_left);
      els.frontRight.textContent = fmt(data.sectors.front_right);
      els.left.textContent = fmt(data.sectors.left);
      els.right.textContent = fmt(data.sectors.right);
      els.back.textContent = fmt(data.sectors.back);
      els.decision.textContent = data.decision;
      els.age.textContent = `${data.age_ms} ms`;
    }

    drawGrid();
    const events = new EventSource("/events");
    events.onmessage = ev => {
      latest = JSON.parse(ev.data);
      lastSeen = Date.now();
      dotEl.classList.add("ok");
      statusEl.textContent = "live";
      drawScan(latest);
      updateText(latest);
    };
    events.onerror = () => {
      dotEl.classList.remove("ok");
      statusEl.textContent = "reconnecting";
    };
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


def parse_points(raw: bytes) -> list[tuple[float, float, int]]:
    points: list[tuple[float, float, int]] = []
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
                raw_angle = (start + step * p) % 360.0
                if 30 <= dist_mm <= 25000 and confidence >= 30:
                    points.append((raw_angle, dist_mm / 1000.0, confidence))
            i += 47
        else:
            i += 1
    return points


def robot_angle(raw_angle: float, front_center: float) -> float:
    return (raw_angle - front_center) % 360.0


def in_arc(angle: float, center: float, half_width: float) -> bool:
    diff = ((angle - center + 180.0) % 360.0) - 180.0
    return abs(diff) <= half_width


def sector_min(points: list[tuple[float, float]], center: float, half_width: float) -> Optional[float]:
    values = [dist for angle, dist in points if in_arc(angle, center, half_width)]
    return min(values) if values else None


def decide(front: Optional[float], emergency_m: float, caution_m: float) -> str:
    if front is None:
        return "STOP no-front"
    if front < emergency_m:
        return "STOP"
    if front < caution_m:
        return "CAUTION"
    return "CLEAR"


class LidarReader:
    def __init__(
        self,
        port: str,
        baud: int,
        front_center: float,
        hz: float,
        emergency_m: float,
        caution_m: float,
    ) -> None:
        self.port = port
        self.baud = baud
        self.front_center = front_center
        self.period = 1.0 / hz
        self.emergency_m = emergency_m
        self.caution_m = caution_m
        self.subscribers: list[queue.Queue[str]] = []
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def subscribe(self) -> queue.Queue[str]:
        q: queue.Queue[str] = queue.Queue(maxsize=3)
        with self.lock:
            self.subscribers.append(q)
        return q

    def unsubscribe(self, q: queue.Queue[str]) -> None:
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    def publish(self, payload: dict) -> None:
        text = json.dumps(payload, separators=(",", ":"))
        with self.lock:
            subscribers = list(self.subscribers)
        for q in subscribers:
            try:
                if q.full():
                    q.get_nowait()
                q.put_nowait(text)
            except queue.Full:
                pass

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                    ser.reset_input_buffer()
                    while not self.stop_event.is_set():
                        raw = bytearray()
                        deadline = time.monotonic() + self.period
                        while time.monotonic() < deadline:
                            raw.extend(ser.read(4096))
                        raw_points = parse_points(raw)
                        robot_points = [
                            (robot_angle(angle, self.front_center), dist)
                            for angle, dist, _ in raw_points
                        ]
                        sectors = {
                            "front": sector_min(robot_points, 0.0, 30.0),
                            "front_left": sector_min(robot_points, 315.0, 15.0),
                            "front_right": sector_min(robot_points, 45.0, 15.0),
                            "left": sector_min(robot_points, 270.0, 30.0),
                            "right": sector_min(robot_points, 90.0, 30.0),
                            "back": sector_min(robot_points, 180.0, 30.0),
                        }
                        payload = {
                            "ts": time.time(),
                            "age_ms": 0,
                            "front_center": self.front_center,
                            "points": [[round(a, 2), round(d, 3)] for a, d in robot_points],
                            "sectors": sectors,
                            "decision": decide(sectors["front"], self.emergency_m, self.caution_m),
                        }
                        self.publish(payload)
            except Exception as exc:
                self.publish(
                    {
                        "ts": time.time(),
                        "age_ms": 9999,
                        "front_center": self.front_center,
                        "points": [],
                        "sectors": {
                            "front": None,
                            "front_left": None,
                            "front_right": None,
                            "left": None,
                            "right": None,
                            "back": None,
                        },
                        "decision": f"ERROR {type(exc).__name__}",
                    }
                )
                time.sleep(1.0)


def make_handler(reader: LidarReader):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:
            return

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
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
                q = reader.subscribe()
                try:
                    while True:
                        text = q.get()
                        self.wfile.write(f"data: {text}\n\n".encode("utf-8"))
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    reader.unsubscribe(q)
                return

            self.send_error(404)

    return Handler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--front-center", type=float, default=DEFAULT_FRONT_CENTER)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--front-stop", type=float, default=0.40)
    parser.add_argument("--slow-distance", type=float, default=0.80)
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    reader = LidarReader(
        port=args.port,
        baud=args.baud,
        front_center=args.front_center,
        hz=args.hz,
        emergency_m=args.front_stop,
        caution_m=args.slow_distance,
    )
    reader.start()

    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(reader))
    print(f"LiDAR live viewer: http://{args.host}:{args.http_port}")
    print(
        f"Reading {args.port} at {args.baud}, front_center={args.front_center:g}; "
        "LiDAR read-only, no STM32/motor output."
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop_event.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
