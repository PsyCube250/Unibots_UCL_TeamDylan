#!/usr/bin/env python3
"""
Ground obstacle-avoidance controller for Jetson + STL-27L LiDAR + STM32.

The default mode is sensor/STM32 preflight only. Motor output requires both
--live-motors and --ground-test. On every exit path it sends STOP and ENABLE 0.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import serial

from lidar_motor_dry_run import (
    BAUD as LIDAR_BAUD,
    LIDAR_PORT,
    AngleArc,
    choose,
    filter_points,
    fmt_stats,
    parse_angle_arc,
    parse_points,
    sector_stats,
)

STM32_PORT = os.environ.get("UNIBOTS_STM32_PORT", "/dev/ttyTHS1")
STM32_BAUD = 115200


@dataclass
class ControllerState:
    action: str = "STOP"
    action_since: float = 0.0
    last_lidar_s: float = 0.0
    last_command_s: float = 0.0
    forced_action: str = ""
    forced_until_s: float = 0.0
    queued_action: str = ""


def send_line(ser: serial.Serial, line: str) -> None:
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()


def drain_lines(ser: serial.Serial, seconds: float, show_raw: bool) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        lines.append(line)
        if show_raw:
            print("STM32", line)
    return lines


def read_scan(ser: serial.Serial, seconds: float) -> list[tuple[float, float, int]]:
    raw = bytearray()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw.extend(ser.read(4096))
    return parse_points(raw)


def has_ok(lines: Iterable[str], prefix: str) -> bool:
    return any(line.startswith(prefix) for line in lines)


def preflight_stm32(ser: serial.Serial, args: argparse.Namespace) -> None:
    ser.reset_input_buffer()
    for command in ("STOP", "ENABLE 0", "PING", "STATUS", f"LIMIT {args.limit}", f"TELEM {args.telemetry_hz}"):
        send_line(ser, command)
        lines = drain_lines(ser, 0.35, args.show_raw)
        if command == "PING" and not has_ok(lines, "OK PONG"):
            raise RuntimeError("STM32 did not answer PING")
        if command.startswith("LIMIT") and not has_ok(lines, f"OK LIMIT {args.limit}"):
            raise RuntimeError(f"STM32 did not accept LIMIT {args.limit}")


def choose_action(
    front: Optional[float],
    left: Optional[float],
    right: Optional[float],
    rear: Optional[float],
    lidar_age_s: float,
    args: argparse.Namespace,
) -> tuple[str, str]:
    if lidar_age_s > args.stale_seconds:
        return "STOP", f"lidar stale {lidar_age_s:.2f}s"
    if front is None:
        return "STOP", "no front sector data"
    left_v = left if left is not None else 0.0
    right_v = right if right is not None else 0.0
    turn_action = "TURN_LEFT" if left_v >= right_v else "TURN_RIGHT"
    turn_reason = "left clearer" if turn_action == "TURN_LEFT" else "right clearer"
    rear_clear = rear is None or rear >= args.rear_stop_m
    if front < args.emergency_stop_m:
        if args.nonstop_recovery:
            if rear_clear:
                return "BACKUP", f"emergency front {front:.2f}m, backing up"
            return turn_action, f"emergency front {front:.2f}m, rear blocked, {turn_reason}"
        return "STOP", f"front {front:.2f}m < emergency {args.emergency_stop_m:.2f}m"
    if front < args.hard_stop_m:
        if args.nonstop_recovery:
            if rear_clear:
                return "BACKUP", f"front {front:.2f}m < hard stop, backing up"
            return turn_action, f"front {front:.2f}m < hard stop, rear blocked, {turn_reason}"
        return "STOP", f"front {front:.2f}m < hard stop {args.hard_stop_m:.2f}m"
    if front < args.avoid_m:
        if left_v >= right_v:
            return "TURN_LEFT", f"front {front:.2f}m, left clearer {left_v:.2f}>{right_v:.2f}"
        return "TURN_RIGHT", f"front {front:.2f}m, right clearer {right_v:.2f}>{left_v:.2f}"
    return "FORWARD", f"front clear {front:.2f}m"


def pwm_for_action(action: str, state: ControllerState, args: argparse.Namespace) -> list[int]:
    if action == "FORWARD":
        elapsed = time.monotonic() - state.action_since
        pwm = args.kick_pwm if elapsed < args.kick_seconds else args.forward_pwm
        return [pwm, pwm, pwm, pwm]
    if action == "TURN_LEFT":
        pwm = args.turn_pwm
        return [-pwm, pwm, -pwm, pwm]
    if action == "TURN_RIGHT":
        pwm = args.turn_pwm
        return [pwm, -pwm, pwm, -pwm]
    if action == "BACKUP":
        pwm = args.backup_pwm
        return [-pwm, -pwm, -pwm, -pwm]
    return [0, 0, 0, 0]


def clearer_turn(left: Optional[float], right: Optional[float]) -> str:
    left_v = left if left is not None else 0.0
    right_v = right if right is not None else 0.0
    return "TURN_LEFT" if left_v >= right_v else "TURN_RIGHT"


def send_motion_command(
    stm: serial.Serial,
    action: str,
    pwm: list[int],
    live: bool,
    show_raw: bool,
) -> None:
    if not live:
        return
    if action == "STOP":
        send_line(stm, "STOP")
    else:
        send_line(stm, "PWM " + " ".join(str(v) for v in pwm))
    drain_lines(stm, 0.02, show_raw)


def stop_and_disable(stm: Optional[serial.Serial]) -> None:
    if stm is None:
        return
    try:
        send_line(stm, "STOP")
        send_line(stm, "ENABLE 0")
    except Exception:
        pass


def run(args: argparse.Namespace) -> int:
    live = args.live_motors and args.ground_test
    if args.live_motors != args.ground_test:
        raise SystemExit("Ground motion requires both --live-motors and --ground-test")
    if max(args.forward_pwm, args.turn_pwm, args.kick_pwm, args.backup_pwm) > args.limit:
        raise SystemExit("PWM values must not exceed --limit")

    lidar = serial.Serial(args.lidar_port, args.lidar_baud, timeout=0.03)
    stm: Optional[serial.Serial] = None
    state = ControllerState(action="STOP", action_since=time.monotonic())

    print(
        f"LiDAR obstacle controller: lidar={args.lidar_port}@{args.lidar_baud}, "
        f"stm32={args.stm32_port}@{args.stm32_baud}, front_center={args.front_center:g}"
    )
    print(
        f"mode={'LIVE GROUND MOTION' if live else 'PREFLIGHT ONLY'}, "
        f"limit={args.limit}, forward={args.forward_pwm}, turn={args.turn_pwm}, "
        f"backup={args.backup_pwm}, kick={args.kick_pwm}/{args.kick_seconds:.2f}s, "
        f"front_stat={args.decision_stat}, side_stat={args.side_stat}"
    )

    try:
        stm = serial.Serial(args.stm32_port, args.stm32_baud, timeout=0.08)
        preflight_stm32(stm, args)
        if live:
            send_line(stm, "ENABLE 1")
            drain_lines(stm, 0.25, args.show_raw)
            print("LIVE motors enabled. Keep a hand near power. Ctrl-C or timeout stops.")
        else:
            print("Preflight OK. Not enabling motors.")

        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            raw_points = read_scan(lidar, args.scan_seconds)
            now = time.monotonic()
            if raw_points:
                state.last_lidar_s = now
            lidar_age = now - state.last_lidar_s if state.last_lidar_s else 999.0
            points = filter_points(
                raw_points,
                args.front_center,
                args.ignore_raw_arc,
                args.ignore_robot_arc,
            )

            front = sector_stats(points, args.front_center, 0.0, args.front_width / 2.0)
            left = sector_stats(points, args.front_center, 270.0, 35.0)
            right = sector_stats(points, args.front_center, 90.0, 35.0)
            rear = sector_stats(points, args.front_center, 180.0, 35.0)
            front_left = sector_stats(points, args.front_center, 315.0, 20.0)
            front_right = sector_stats(points, args.front_center, 45.0, 20.0)

            if state.forced_action and now >= state.forced_until_s:
                if state.queued_action:
                    state.forced_action = state.queued_action
                    state.queued_action = ""
                    state.forced_until_s = now + args.recovery_turn_seconds
                    state.action_since = now
                else:
                    state.forced_action = ""

            if state.forced_action:
                action = state.forced_action
                reason = f"recovery action for {max(0.0, state.forced_until_s - now):.2f}s"
            else:
                action, reason = choose_action(
                    choose(front, args.decision_stat),
                    choose(left, args.side_stat),
                    choose(right, args.side_stat),
                    choose(rear, args.side_stat),
                    lidar_age,
                    args,
                )
                if action == "BACKUP":
                    state.forced_action = "BACKUP"
                    state.forced_until_s = now + args.backup_seconds
                    state.queued_action = clearer_turn(
                        choose(left, args.side_stat),
                        choose(right, args.side_stat),
                    )
                    reason = (
                        f"{reason}; then {state.queued_action} "
                        f"for {args.recovery_turn_seconds:.2f}s"
                    )
            if action != state.action:
                state.action = action
                state.action_since = now

            pwm = pwm_for_action(action, state, args)
            if now - state.last_command_s >= args.command_period:
                send_motion_command(stm, action, pwm, live, args.show_raw)
                state.last_command_s = now

            print(
                f"t={now - start:05.2f}s pts={len(points):4d}/{len(raw_points):4d} "
                f"{fmt_stats('front', front)} {fmt_stats('fl', front_left)} "
                f"{fmt_stats('fr', front_right)} {fmt_stats('left', left)} "
                f"{fmt_stats('right', right)} {fmt_stats('rear', rear)} "
                f"age={lidar_age:.2f}s -> "
                f"{action:10s} pwm={pwm} {reason}"
            )
            time.sleep(args.loop_sleep)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        stop_and_disable(stm)
        if stm is not None:
            stm.close()
        lidar.close()
        print("Sent STOP and ENABLE 0 on exit")

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lidar-port", default=LIDAR_PORT)
    parser.add_argument("--lidar-baud", type=int, default=LIDAR_BAUD)
    parser.add_argument("--stm32-port", default=STM32_PORT)
    parser.add_argument("--stm32-baud", type=int, default=STM32_BAUD)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--front-center", type=float, default=270.0)
    parser.add_argument("--front-width", type=float, default=60.0)
    parser.add_argument("--decision-stat", choices=["min", "p10", "median"], default="p10")
    parser.add_argument("--side-stat", choices=["min", "p10", "median"], default="median")
    parser.add_argument("--nonstop-recovery", action="store_true")
    parser.add_argument("--emergency-stop-m", type=float, default=0.16)
    parser.add_argument("--hard-stop-m", type=float, default=0.32)
    parser.add_argument("--rear-stop-m", type=float, default=0.25)
    parser.add_argument("--avoid-m", type=float, default=0.75)
    parser.add_argument("--stale-seconds", type=float, default=0.50)
    parser.add_argument("--scan-seconds", type=float, default=0.12)
    parser.add_argument("--loop-sleep", type=float, default=0.03)
    parser.add_argument("--command-period", type=float, default=0.10)
    parser.add_argument("--telemetry-hz", type=int, default=10)
    parser.add_argument("--limit", type=int, default=45)
    parser.add_argument("--forward-pwm", type=int, default=35)
    parser.add_argument("--turn-pwm", type=int, default=34)
    parser.add_argument("--backup-pwm", type=int, default=32)
    parser.add_argument("--backup-seconds", type=float, default=0.55)
    parser.add_argument("--recovery-turn-seconds", type=float, default=0.75)
    parser.add_argument("--kick-pwm", type=int, default=45)
    parser.add_argument("--kick-seconds", type=float, default=0.25)
    parser.add_argument("--ignore-raw-arc", action="append", type=parse_angle_arc, default=[])
    parser.add_argument("--ignore-robot-arc", action="append", type=parse_angle_arc, default=[])
    parser.add_argument("--show-raw", action="store_true")
    parser.add_argument("--live-motors", action="store_true")
    parser.add_argument("--ground-test", action="store_true")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
