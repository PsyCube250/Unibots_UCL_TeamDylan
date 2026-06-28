#!/usr/bin/env python3
"""
LiDAR-only obstacle dry run for the Jetson.

This script never opens the STM32 serial port and never writes motor commands.
It reads the STL-27L LiDAR, maps raw clockwise LiDAR angles into the robot frame,
and prints what a very small safety controller would do.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import serial

LIDAR_PORT = os.environ.get(
    "UNIBOTS_LIDAR_PORT",
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
)
if not os.path.exists(LIDAR_PORT):
    LIDAR_PORT = "/dev/ttyUSB0"

BAUD = 921600
STALE_S = 0.50


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
    """Return clockwise robot-relative angle: 0=front, 90=right, 180=back."""
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


def decide(
    front: Optional[float],
    left: Optional[float],
    right: Optional[float],
    age: float,
    emergency_m: float,
    caution_m: float,
) -> tuple[str, str, list[int]]:
    if age > STALE_S or front is None:
        return "STOP", "lidar stale/no front data", [0, 0, 0, 0]
    if front < emergency_m:
        return "STOP", f"front {front:.2f}m < {emergency_m:.2f}m", [0, 0, 0, 0]
    if front < caution_m:
        if (left or 0.0) > (right or 0.0):
            return "TURN_LEFT", f"front caution {front:.2f}m, left clearer", [-10, 10, -10, 10]
        return "TURN_RIGHT", f"front caution {front:.2f}m, right clearer", [10, -10, 10, -10]
    return "FORWARD", f"front clear {front:.2f}m", [10, 10, 10, 10]


def fmt_stats(name: str, stats: SectorStats) -> str:
    if stats.count == 0:
        return f"{name}=none"
    return (
        f"{name}=min:{stats.min_m:.3f}/p10:{stats.p10_m:.3f}/"
        f"med:{stats.median_m:.3f}/n:{stats.count}"
    )


def read_scan(ser: serial.Serial, duration: float) -> list[tuple[float, float, int]]:
    raw = bytearray()
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        raw.extend(ser.read(4096))
    return parse_points(raw)


def run(args: argparse.Namespace) -> int:
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print(
        "DRY RUN ONLY: reading "
        f"{args.port}, front_center={args.front_center:.1f} raw deg, "
        "not opening STM32, not writing motor commands"
    )
    print(
        "Robot frame: 0=front, 90=right, 180=back, 270=left. "
        f"Decision uses {args.decision_stat}."
    )
    if args.ignore_raw_arc or args.ignore_robot_arc:
        raw_desc = ", ".join(f"{a.start:g}:{a.end:g}" for a in args.ignore_raw_arc) or "none"
        robot_desc = ", ".join(f"{a.start:g}:{a.end:g}" for a in args.ignore_robot_arc) or "none"
        print(f"Ignoring raw arcs: {raw_desc}; robot arcs: {robot_desc}")

    last_scan = 0.0
    try:
        for k in range(args.iterations):
            raw_points = read_scan(ser, args.scan_seconds)
            points = filter_points(
                raw_points,
                args.front_center,
                args.ignore_raw_arc,
                args.ignore_robot_arc,
            )
            now = time.monotonic()
            if points:
                last_scan = now
            age = now - last_scan if last_scan else 999.0

            front = sector_stats(points, args.front_center, 0.0, args.front_width / 2.0)
            front_left = sector_stats(points, args.front_center, 315.0, 15.0)
            front_right = sector_stats(points, args.front_center, 45.0, 15.0)
            left = sector_stats(points, args.front_center, 270.0, 30.0)
            right = sector_stats(points, args.front_center, 90.0, 30.0)

            action, reason, pwm = decide(
                choose(front, args.decision_stat),
                choose(left, args.decision_stat),
                choose(right, args.decision_stat),
                age,
                args.front_stop,
                args.slow_distance,
            )
            print(
                f"{k:02d} pts={len(points):4d}/{len(raw_points):4d} "
                f"{fmt_stats('front', front)} "
                f"{fmt_stats('fl', front_left)} "
                f"{fmt_stats('fr', front_right)} "
                f"{fmt_stats('left', left)} "
                f"{fmt_stats('right', right)} "
                f"age={age:.3f}s -> would {action} pwm={pwm} ({reason})"
            )
            time.sleep(args.pause_seconds)
    finally:
        ser.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=LIDAR_PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--iterations", type=int, default=12)
    parser.add_argument("--scan-seconds", type=float, default=0.25)
    parser.add_argument("--pause-seconds", type=float, default=0.25)
    parser.add_argument(
        "--front-center",
        type=float,
        default=270.0,
        help="raw LiDAR angle that points toward the robot front; use 270 if the left mounting side faces front",
    )
    parser.add_argument("--front-width", type=float, default=60.0)
    parser.add_argument("--front-stop", type=float, default=0.40)
    parser.add_argument("--slow-distance", type=float, default=0.60)
    parser.add_argument(
        "--ignore-raw-arc",
        action="append",
        type=parse_angle_arc,
        default=[],
        help="ignore raw LiDAR angle arc START:END degrees; repeat for fixed self-obstructions",
    )
    parser.add_argument(
        "--ignore-robot-arc",
        action="append",
        type=parse_angle_arc,
        default=[],
        help="ignore robot-relative angle arc START:END degrees after front-center mapping",
    )
    parser.add_argument(
        "--decision-stat",
        choices=["min", "p10", "median"],
        default="min",
        help="min is safest; p10/median are useful for diagnosing fixed self-obstructions in dry-run only",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
