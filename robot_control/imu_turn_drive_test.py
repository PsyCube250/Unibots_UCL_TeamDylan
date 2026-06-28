#!/usr/bin/env python3
"""
Short supervised floor test for STM32 motor direction and IMU yaw response.

Live sequence:
  1. drive straight for --forward-seconds
  2. stop for --after-forward-pause-s
  3. turn left in place to --turn-degrees using IMU yaw
  4. stop for --pause-s
  5. turn right in place to --turn-degrees using IMU yaw
  6. stop and disable motors

Default mode is dry-run. Motor output requires both --live-motors and
--ground-test. Every exit path sends STOP and ENABLE 0.
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass
from typing import Iterable, Optional

from jetson_safety_imu import ImuReader

STM32_PORT = os.environ.get("UNIBOTS_STM32_PORT", "/dev/ttyTHS1")
STM32_BAUD = 115200


@dataclass
class SerialReply:
    ok: bool
    lines: list[str]


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


def command_expect(ser, command: str, ok_prefix: str, show_raw: bool) -> SerialReply:
    send_line(ser, command)
    lines = drain_lines(ser, 0.35, show_raw)
    return SerialReply(any(line.startswith(ok_prefix) for line in lines), lines)


def stop_and_disable(ser: Optional[object]) -> None:
    if ser is None:
        return
    try:
        send_line(ser, "STOP")
        send_line(ser, "ENABLE 0")
    except Exception:
        pass


def signed_delta_deg(start: float, current: float) -> float:
    return (current - start + 540.0) % 360.0 - 180.0


def open_serial(port: str, baud: int):
    import serial

    return serial.Serial(port, baud, timeout=0.05)


def read_yaw(imu: ImuReader, timeout_s: float = 1.0) -> float:
    deadline = time.monotonic() + timeout_s
    last_reason = "no IMU sample"
    while time.monotonic() < deadline:
        sample = imu.read()
        if sample.ok and sample.yaw_deg is not None:
            return sample.yaw_deg
        last_reason = sample.reason
        time.sleep(0.02)
    raise RuntimeError(f"Could not read IMU yaw: {last_reason}")


def pwm_for_turn(direction: str, pwm: int) -> list[int]:
    if direction == "left":
        return [-pwm, pwm, -pwm, pwm]
    if direction == "right":
        return [pwm, -pwm, pwm, -pwm]
    raise ValueError(direction)


def send_pwm(ser, pwm: Iterable[int]) -> None:
    send_line(ser, "PWM " + " ".join(str(int(v)) for v in pwm))


def stop_pause(ser, seconds: float) -> None:
    send_line(ser, "STOP")
    drain_lines(ser, 0.05)
    time.sleep(seconds)


def run_turn_step(ser, imu: ImuReader, direction: str, args: argparse.Namespace) -> None:
    start_yaw = read_yaw(imu)
    start_t = time.monotonic()
    next_print = 0.0
    max_delta = 0.0
    print(f"{direction.upper()} start yaw={start_yaw:.1f} target={args.turn_degrees:.1f}deg")

    while True:
        now = time.monotonic()
        yaw = read_yaw(imu, timeout_s=0.25)
        delta = signed_delta_deg(start_yaw, yaw)
        progress = abs(delta)
        max_delta = max(max_delta, progress)

        if progress >= max(0.0, args.turn_degrees - args.turn_tolerance_deg):
            break
        if now - start_t > args.turn_timeout_s:
            print(f"{direction.upper()} timeout at delta={delta:.1f}deg; stopping")
            break

        pwm = args.turn_pwm
        remaining = args.turn_degrees - progress
        if remaining < args.turn_slowdown_deg:
            pwm = max(args.min_turn_pwm, int(args.turn_pwm * 0.58))
        send_pwm(ser, pwm_for_turn(direction, pwm))

        if now >= next_print:
            print(
                f"{direction.upper()} yaw={yaw:.1f} delta={delta:+.1f}deg "
                f"remaining={max(0.0, remaining):.1f} pwm={pwm}"
            )
            next_print = now + 0.25
        drain_lines(ser, 0.02, args.show_raw)

    stop_pause(ser, args.pause_s)
    final_yaw = read_yaw(imu)
    final_delta = signed_delta_deg(start_yaw, final_yaw)
    print(
        f"{direction.upper()} done final yaw={final_yaw:.1f} "
        f"delta={final_delta:+.1f}deg max_abs_delta={max_delta:.1f}deg"
    )


def run_forward_step(ser, args: argparse.Namespace) -> None:
    print(f"FORWARD start pwm={args.forward_pwm} duration={args.forward_seconds:.2f}s")
    start = time.monotonic()
    next_print = 0.0
    while time.monotonic() - start < args.forward_seconds:
        send_pwm(ser, [args.forward_pwm] * 4)
        if time.monotonic() >= next_print:
            print(f"FORWARD t={time.monotonic() - start:.2f}s pwm={args.forward_pwm}")
            next_print = time.monotonic() + 0.25
        drain_lines(ser, 0.04, args.show_raw)
    stop_pause(ser, args.after_forward_pause_s)
    print("FORWARD done")


def preflight(ser, args: argparse.Namespace) -> None:
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    for command, ok_prefix in (
        ("STOP", "OK STOP"),
        ("ENABLE 0", "OK ENABLE 0"),
        ("PING", "OK PONG"),
        ("STATUS", "OK STATUS"),
        (f"LIMIT {args.limit}", f"OK LIMIT {args.limit}"),
        (f"TELEM {args.telemetry_hz}", f"OK TELEM {args.telemetry_hz}"),
    ):
        reply = command_expect(ser, command, ok_prefix, args.show_raw)
        if command in ("PING", f"LIMIT {args.limit}") and not reply.ok:
            raise RuntimeError(f"STM32 did not accept {command}; replies={reply.lines[-5:]}")


def run(args: argparse.Namespace) -> int:
    live = args.live_motors and args.ground_test
    if args.live_motors != args.ground_test:
        raise SystemExit("Motor movement requires both --live-motors and --ground-test")
    if max(abs(args.turn_pwm), abs(args.forward_pwm), abs(args.min_turn_pwm)) > args.limit:
        raise SystemExit("PWM values must not exceed --limit")

    print(
        f"IMU turn/straight test: stm32={args.port}@{args.baud}, "
        f"mode={'LIVE GROUND MOTION' if live else 'DRY RUN'}, "
        f"limit={args.limit}, turn_pwm={args.turn_pwm}, forward_pwm={args.forward_pwm}"
    )

    ser = None
    imu = None
    sent_motion = False
    try:
        ser = open_serial(args.port, args.baud)
        preflight(ser, args)

        imu = ImuReader(
            bus=str(args.imu_bus),
            address=hex(args.imu_address),
            calibrate_seconds=args.imu_calibrate_s,
        )
        print(
            f"Opening IMU bus={args.imu_bus} address=0x{args.imu_address:02x}; "
            f"keep robot still for {args.imu_calibrate_s:.1f}s calibration"
        )
        imu.open(require=True)
        first_yaw = read_yaw(imu)
        print(
            f"IMU OK sensor={imu.sensor} bus={imu.bus_no} "
            f"address=0x{imu.address:02x} yaw={first_yaw:.1f}"
        )

        if not live:
            print("Dry-run complete. Add --live-motors --ground-test to move.")
            return 0

        command_expect(ser, "ENABLE 1", "OK ENABLE 1", args.show_raw)
        sent_motion = True
        print("LIVE motors enabled. Ctrl-C or timeout stops and disables.")

        run_forward_step(ser, args)
        run_turn_step(ser, imu, "left", args)
        run_turn_step(ser, imu, "right", args)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 130
    finally:
        if sent_motion or args.send_stop_on_exit:
            stop_and_disable(ser)
            print("Sent STOP and ENABLE 0 on exit")
        if imu is not None:
            imu.close()
        if ser is not None:
            ser.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=STM32_PORT)
    parser.add_argument("--baud", type=int, default=STM32_BAUD)
    parser.add_argument("--limit", type=int, default=45)
    parser.add_argument("--telemetry-hz", type=int, default=10)
    parser.add_argument("--show-raw", action="store_true")
    parser.add_argument("--send-stop-on-exit", action="store_true")

    parser.add_argument("--imu-bus", type=int, default=1)
    parser.add_argument("--imu-address", type=lambda text: int(text, 0), default=0x6A)
    parser.add_argument("--imu-calibrate-s", type=float, default=1.0)

    parser.add_argument("--turn-degrees", type=float, default=90.0)
    parser.add_argument("--turn-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--turn-timeout-s", type=float, default=10.0)
    parser.add_argument("--turn-pwm", type=int, default=45)
    parser.add_argument("--min-turn-pwm", type=int, default=34)
    parser.add_argument("--turn-slowdown-deg", type=float, default=22.0)
    parser.add_argument("--pause-s", type=float, default=0.40)

    parser.add_argument("--forward-seconds", type=float, default=1.0)
    parser.add_argument("--forward-pwm", type=int, default=38)
    parser.add_argument("--after-forward-pause-s", type=float, default=3.0)

    parser.add_argument("--live-motors", action="store_true")
    parser.add_argument("--ground-test", action="store_true")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
