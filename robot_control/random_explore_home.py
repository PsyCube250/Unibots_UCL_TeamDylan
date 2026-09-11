#!/usr/bin/env python3
"""
Random exploration with LiDAR obstacle avoidance and AprilTag homing.

The robot wanders randomly, avoids obstacles using the STL-27L LiDAR,
and returns to its home zone when it sees its target AprilTag(s).

Usage:
  python3 robot_control/random_explore_home.py --live-motors --ground-test

Without --live-motors it runs in dry-run mode (no motor output).
"""

from __future__ import annotations

import argparse
import math
import os
import random
import struct
import sys
import time
import threading
from dataclasses import dataclass
from enum import Enum, auto
from http.server import HTTPServer, BaseHTTPRequestHandler

import serial
import cv2
import numpy as np

try:
    import dt_apriltags as apriltag
except ImportError:
    apriltag = None

# ─── LiDAR constants ─────────────────────────────────────────────────────────

LIDAR_BAUD = 921600
LIDAR_STALE_S = 0.5

# ─── STM32 constants ─────────────────────────────────────────────────────────

STM32_BAUD = 115200
WATCHDOG_RESEND_S = 0.35
MOTOR_SIGN = (1, -1, 1, -1)

# ─── AprilTag constants ──────────────────────────────────────────────────────

TAG_SIZE_M = 0.10
HOME_TAG_IDS = {20, 21}

# ─── Web view ────────────────────────────────────────────────────────────────

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


# ─── State machine ───────────────────────────────────────────────────────────

class State(Enum):
    EXPLORE = auto()
    AVOID_TURN = auto()
    AVOID_BACKUP = auto()
    HOME_ALIGN = auto()
    HOME_APPROACH = auto()
    ARRIVED = auto()


# ─── LiDAR parsing ───────────────────────────────────────────────────────────

def parse_lidar_points(raw: bytes) -> list[tuple[float, float]]:
    """Parse STL-27L frames into (raw_angle_deg, distance_m) tuples."""
    points = []
    i = 0
    while i <= len(raw) - 47:
        if raw[i] == 0x54 and raw[i + 1] == 0x2C:
            frame = raw[i:i + 47]
            start = struct.unpack_from("<H", frame, 4)[0] / 100.0
            end = struct.unpack_from("<H", frame, 42)[0] / 100.0
            delta = (end - start) % 360.0
            step = delta / 11.0 if delta <= 180.0 else -(360.0 - delta) / 11.0
            for p in range(12):
                off = 6 + p * 3
                dist_mm = struct.unpack_from("<H", frame, off)[0]
                confidence = frame[off + 2]
                angle = (start + step * p) % 360.0
                if 30 <= dist_mm <= 12000 and confidence >= 30:
                    points.append((angle, dist_mm / 1000.0))
            i += 47
        else:
            i += 1
    return points


def robot_angle(raw_angle: float, front_center: float) -> float:
    """Convert raw clockwise LiDAR angle to robot-relative: 0=front, 90=right, 270=left."""
    return (front_center - raw_angle) % 360.0


def sector_min(points: list[tuple[float, float]], center_deg: float, half_width: float, front_center: float) -> float | None:
    """Get minimum distance in a robot-relative angular sector."""
    dists = []
    for raw_ang, dist in points:
        ra = robot_angle(raw_ang, front_center)
        diff = ((ra - center_deg + 180.0) % 360.0) - 180.0
        if abs(diff) <= half_width:
            dists.append(dist)
    return min(dists) if dists else None


# ─── Motor controller ────────────────────────────────────────────────────────

class MotorController:
    def __init__(self, port: str, live: bool, limit: int):
        self.live = live
        self.limit = limit
        self.ser = None
        self.last_cmd = ''
        self.last_send_t = 0.0

        if live:
            self.ser = serial.Serial(port, STM32_BAUD, timeout=0.08)
            time.sleep(0.1)
            self._send('STOP')
            self._send('ENABLE 0')
            time.sleep(0.05)
            self._send(f'LIMIT {limit}')
            self._send(f'MOTOR_SIGN {" ".join(str(s) for s in MOTOR_SIGN)}')
            self._send('ENABLE 1')
            print(f"[MOTOR] LIVE on {port}, limit={limit}")
        else:
            print(f"[MOTOR] DRY-RUN (no motor output)")

    def _send(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()
        self.last_send_t = time.time()

    def pwm(self, fl: int, fr: int, bl: int, br: int):
        values = [max(-self.limit, min(self.limit, v)) for v in (fl, fr, bl, br)]
        cmd = f'PWM {values[0]} {values[1]} {values[2]} {values[3]}'
        if cmd != self.last_cmd or time.time() - self.last_send_t > WATCHDOG_RESEND_S:
            self._send(cmd)
            self.last_cmd = cmd

    def stop(self):
        self._send('STOP')
        self.last_cmd = 'STOP'

    def mix(self, forward: int, turn_right: int):
        fl = forward + turn_right
        fr = forward - turn_right
        bl = forward + turn_right
        br = forward - turn_right
        self.pwm(fl, fr, bl, br)

    def close(self):
        self._send('STOP')
        self._send('ENABLE 0')
        if self.ser:
            self.ser.close()


# ─── AprilTag detector ───────────────────────────────────────────────────────

class TagDetector:
    def __init__(self, camera_path: str, rotate_180: bool, hfov_deg: float):
        self.cap = cv2.VideoCapture(camera_path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.rotate_180 = rotate_180

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fx = self.width / (2.0 * math.tan(math.radians(hfov_deg / 2.0)))
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        self.detector = None
        if apriltag is not None:
            self.detector = apriltag.Detector(
                families='tag36h11',
                nthreads=2,
                quad_decimate=2.0,
                refine_edges=1,
            )

        if not self.cap.isOpened():
            print("[TAG] WARNING: Camera not available")

    def detect(self) -> tuple[np.ndarray | None, list[dict]]:
        """Read frame, detect tags. Returns (frame, list of {id, angle_deg, distance_m})."""
        if not self.cap.isOpened() or self.detector is None:
            return None, []

        ret, frame = self.cap.read()
        if not ret:
            return None, []

        if self.rotate_180:
            frame = cv2.flip(frame, -1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy),
            tag_size=TAG_SIZE_M,
        )

        results = []
        for d in detections:
            distance_m = float(d.pose_t[2][0])
            cx_tag = float(d.center[0])
            offset = cx_tag - self.cx
            angle_deg = math.degrees(math.atan(offset / self.fx))
            results.append({
                'id': d.tag_id,
                'angle_deg': angle_deg,
                'distance_m': distance_m,
            })

        # Draw on frame
        for d in detections:
            corners = d.corners.astype(int)
            for j in range(4):
                cv2.line(frame, tuple(corners[j]), tuple(corners[(j+1) % 4]), (0, 255, 0), 2)
            cx_tag = int(d.center[0])
            cy_tag = int(d.center[1])
            cv2.putText(frame, f"ID:{d.tag_id}", (cx_tag - 20, cy_tag - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame, results

    def close(self):
        if self.cap.isOpened():
            self.cap.release()


# ─── Main controller ─────────────────────────────────────────────────────────

class Explorer:
    def __init__(self, args):
        self.args = args
        self.state = State.EXPLORE
        self.state_start = time.time()
        self.explore_action_until = 0.0
        self.explore_forward = True
        self.turn_direction = random.choice([-1, 1])

        # LiDAR
        self.lidar_ser = None
        self.lidar_points = []
        self.lidar_last_t = 0.0
        self._open_lidar()

        # Motors
        self.motor = MotorController(args.stm32_port, args.live_motors and args.ground_test, args.limit)

        # Camera + AprilTag
        self.tag_detector = TagDetector(args.camera, args.rotate_180, args.hfov_deg)

        # Home state
        self.home_tag = None
        self.home_last_seen = 0.0
        self.arrived_time = 0.0

        # Web view
        if args.http_port > 0:
            server = HTTPServer(('0.0.0.0', args.http_port), MJPEGHandler)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            print(f"[WEB] http://0.0.0.0:{args.http_port}")

    def _open_lidar(self):
        port = self.args.lidar_port
        if not os.path.exists(port):
            alt = '/dev/ttyUSB0'
            if os.path.exists(alt):
                port = alt
        try:
            self.lidar_ser = serial.Serial(port, LIDAR_BAUD, timeout=0.02)
            print(f"[LIDAR] Opened {port}")
        except Exception as e:
            print(f"[LIDAR] Failed to open {port}: {e}")

    def read_lidar(self):
        if self.lidar_ser is None or not self.lidar_ser.is_open:
            return
        try:
            raw = self.lidar_ser.read(self.lidar_ser.in_waiting or 1024)
            if raw:
                points = parse_lidar_points(raw)
                if points:
                    self.lidar_points = points
                    self.lidar_last_t = time.time()
        except Exception:
            pass

    def lidar_fresh(self) -> bool:
        return time.time() - self.lidar_last_t < LIDAR_STALE_S

    def front_distance(self) -> float | None:
        if not self.lidar_fresh():
            return None
        return sector_min(self.lidar_points, 0.0, self.args.front_width / 2.0, self.args.front_center)

    def left_distance(self) -> float | None:
        if not self.lidar_fresh():
            return None
        return sector_min(self.lidar_points, 270.0, 30.0, self.args.front_center)

    def right_distance(self) -> float | None:
        if not self.lidar_fresh():
            return None
        return sector_min(self.lidar_points, 90.0, 30.0, self.args.front_center)

    def rear_distance(self) -> float | None:
        if not self.lidar_fresh():
            return None
        return sector_min(self.lidar_points, 180.0, 30.0, self.args.front_center)

    def set_state(self, new_state: State):
        if new_state != self.state:
            print(f"[STATE] {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_start = time.time()

    def run(self):
        print(f"[RUN] Starting exploration. Duration={self.args.duration}s, home_tags={sorted(HOME_TAG_IDS)}")
        start_time = time.time()
        tag_check_interval = 0.2
        last_tag_check = 0.0

        try:
            while True:
                if self.args.duration > 0 and time.time() - start_time > self.args.duration:
                    print("[RUN] Duration expired")
                    break

                self.read_lidar()

                # Check AprilTags periodically
                now = time.time()
                if now - last_tag_check >= tag_check_interval:
                    last_tag_check = now
                    frame, tags = self.tag_detector.detect()
                    if frame is not None:
                        self._annotate_frame(frame)
                        with _frame_lock:
                            global _latest_frame
                            _latest_frame = frame

                    home_tags = [t for t in tags if t['id'] in HOME_TAG_IDS]
                    if home_tags:
                        self.home_tag = min(home_tags, key=lambda t: t['distance_m'])
                        self.home_last_seen = now

                # State machine
                if self.state == State.EXPLORE:
                    self._do_explore()
                elif self.state == State.AVOID_BACKUP:
                    self._do_avoid_backup()
                elif self.state == State.AVOID_TURN:
                    self._do_avoid_turn()
                elif self.state == State.HOME_ALIGN:
                    self._do_home_align()
                elif self.state == State.HOME_APPROACH:
                    self._do_home_approach()
                elif self.state == State.ARRIVED:
                    self._do_arrived()

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n[RUN] Interrupted")
        finally:
            self.motor.close()
            self.tag_detector.close()
            if self.lidar_ser:
                self.lidar_ser.close()

    def _annotate_frame(self, frame):
        front = self.front_distance()
        front_str = f"{front:.2f}m" if front else "---"
        cv2.putText(frame, f"State: {self.state.name}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Front: {front_str}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        if self.home_tag and time.time() - self.home_last_seen < 1.0:
            cv2.putText(frame, f"HOME tag {self.home_tag['id']}: {self.home_tag['distance_m']:.2f}m, {self.home_tag['angle_deg']:+.1f}deg",
                        (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def _home_tag_visible(self) -> bool:
        return self.home_tag is not None and time.time() - self.home_last_seen < 1.0

    def _do_explore(self):
        # If home tag visible, go home
        if self._home_tag_visible():
            self.set_state(State.HOME_ALIGN)
            return

        front = self.front_distance()

        # Obstacle too close -> avoid
        if front is not None and front < self.args.stop_m:
            self.set_state(State.AVOID_BACKUP)
            return

        now = time.time()
        if now >= self.explore_action_until:
            # Pick a new random action
            if front is not None and front < self.args.slow_m:
                # Getting close, turn
                self.explore_forward = False
                self.turn_direction = self._pick_turn_direction()
                self.explore_action_until = now + random.uniform(0.5, 1.5)
            else:
                # Mostly go forward, sometimes turn
                if random.random() < 0.7:
                    self.explore_forward = True
                    self.explore_action_until = now + random.uniform(1.0, 3.0)
                else:
                    self.explore_forward = False
                    self.turn_direction = random.choice([-1, 1])
                    self.explore_action_until = now + random.uniform(0.5, 1.5)

        if self.explore_forward:
            speed = self.args.forward_pwm
            if front is not None and front < self.args.slow_m:
                speed = self.args.creep_pwm
            self.motor.mix(speed, 0)
        else:
            self.motor.mix(0, self.turn_direction * self.args.turn_pwm)

    def _pick_turn_direction(self) -> int:
        left = self.left_distance()
        right = self.right_distance()
        if left is not None and right is not None:
            return -1 if left > right else 1
        return random.choice([-1, 1])

    def _do_avoid_backup(self):
        elapsed = time.time() - self.state_start
        if elapsed < 0.5:
            self.motor.mix(-self.args.creep_pwm, 0)
            return
        self.turn_direction = self._pick_turn_direction()
        self.set_state(State.AVOID_TURN)

    def _do_avoid_turn(self):
        elapsed = time.time() - self.state_start
        self.motor.mix(0, self.turn_direction * self.args.turn_pwm)
        front = self.front_distance()
        if elapsed > 0.3 and front is not None and front > self.args.slow_m:
            self.set_state(State.EXPLORE)
            return
        if elapsed > 2.0:
            self.turn_direction *= -1
            self.set_state(State.AVOID_TURN)

    def _do_home_align(self):
        if not self._home_tag_visible():
            # Lost the tag, go back to exploring
            if time.time() - self.home_last_seen > 2.0:
                self.set_state(State.EXPLORE)
            else:
                self.motor.stop()
            return

        angle = self.home_tag['angle_deg']
        distance = self.home_tag['distance_m']

        if abs(angle) < self.args.home_angle_tolerance:
            self.set_state(State.HOME_APPROACH)
            return

        turn = int(min(self.args.turn_pwm, max(self.args.min_turn_pwm, abs(angle) * 1.2)))
        if angle > 0:
            self.motor.mix(0, turn)
        else:
            self.motor.mix(0, -turn)

    def _do_home_approach(self):
        if not self._home_tag_visible():
            if time.time() - self.home_last_seen > 2.0:
                self.set_state(State.EXPLORE)
            else:
                self.motor.stop()
            return

        angle = self.home_tag['angle_deg']
        distance = self.home_tag['distance_m']

        # Check if arrived
        if distance < self.args.home_arrive_m:
            self.set_state(State.ARRIVED)
            self.arrived_time = time.time()
            return

        # Re-align if drifted
        if abs(angle) > self.args.home_angle_tolerance * 2:
            self.set_state(State.HOME_ALIGN)
            return

        # Check LiDAR for obstacles on approach
        front = self.front_distance()
        if front is not None and front < self.args.stop_m:
            self.motor.stop()
            return

        # Drive forward with slight correction
        speed = self.args.creep_pwm if distance < 0.4 else self.args.forward_pwm
        turn_correction = int(angle * 0.5)
        self.motor.mix(speed, turn_correction)

    def _do_arrived(self):
        self.motor.stop()
        elapsed = time.time() - self.arrived_time
        if elapsed > 5.0:
            # After 5 seconds at home, go explore again
            print("[HOME] Arrived at home, resuming exploration in 5s...")
            self.home_tag = None
            self.set_state(State.EXPLORE)


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Random explore with LiDAR avoidance + AprilTag homing")
    parser.add_argument('--live-motors', action='store_true', help='Enable motor output')
    parser.add_argument('--ground-test', action='store_true', help='Confirm ground test (required with --live-motors)')
    parser.add_argument('--duration', type=float, default=0, help='Run duration in seconds (0=infinite)')
    parser.add_argument('--stm32-port', default='/dev/ttyTHS1', help='STM32 UART port')
    parser.add_argument('--lidar-port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    parser.add_argument('--camera', default='/dev/video0', help='Camera device')
    parser.add_argument('--rotate-180', action='store_true', default=True, help='Rotate camera 180 degrees')
    parser.add_argument('--no-rotate', dest='rotate_180', action='store_false')
    parser.add_argument('--hfov-deg', type=float, default=120.0, help='Camera horizontal FOV')
    parser.add_argument('--front-center', type=float, default=270.0, help='LiDAR raw angle that points forward')
    parser.add_argument('--front-width', type=float, default=60.0, help='Front sector total width in degrees')
    parser.add_argument('--limit', type=int, default=45, help='PWM limit')
    parser.add_argument('--forward-pwm', type=int, default=35, help='Forward driving speed')
    parser.add_argument('--creep-pwm', type=int, default=25, help='Slow approach speed')
    parser.add_argument('--turn-pwm', type=int, default=30, help='Turning speed')
    parser.add_argument('--min-turn-pwm', type=int, default=18, help='Minimum turn PWM')
    parser.add_argument('--stop-m', type=float, default=0.25, help='Emergency stop distance')
    parser.add_argument('--slow-m', type=float, default=0.50, help='Slow down distance')
    parser.add_argument('--home-angle-tolerance', type=float, default=8.0, help='Angle tolerance for home alignment')
    parser.add_argument('--home-arrive-m', type=float, default=0.25, help='Distance to consider arrived home')
    parser.add_argument('--home-tag-ids', default='20,21', help='Comma-separated home AprilTag IDs')
    parser.add_argument('--http-port', type=int, default=8770, help='Web view port (0 to disable)')

    args = parser.parse_args()

    global HOME_TAG_IDS
    HOME_TAG_IDS = {int(x.strip()) for x in args.home_tag_ids.split(',')}

    if args.live_motors and not args.ground_test:
        print("ERROR: --live-motors requires --ground-test as confirmation")
        sys.exit(1)

    explorer = Explorer(args)
    explorer.run()


if __name__ == '__main__':
    main()
