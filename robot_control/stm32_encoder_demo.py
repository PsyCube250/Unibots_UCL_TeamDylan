#!/usr/bin/env python3
"""
Jetson-side helper for the proposed STM32 four-wheel encoder firmware.

Default mode is telemetry-only: it opens the serial port and prints encoder
counts/ticks-per-second without sending motor commands.

Examples:
  python3 robot_control/stm32_encoder_demo.py --port /dev/ttyACM0
  python3 robot_control/stm32_encoder_demo.py --port /dev/ttyACM0 --send-stop

Lifted-wheel bench test only, after explicit approval:
  python3 robot_control/stm32_encoder_demo.py --port /dev/ttyACM0 --bench-test --enable-motors --mode pwm --values 10 0 0 0
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 115200


@dataclass
class Telemetry:
    seq: int
    ms: int
    mode: int
    enabled: bool
    limit: int
    enc: list[int]
    tps: list[float]
    pwm: list[int]
    target: list[float]
    fault: int


def parse_values(text: str) -> list[int]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected exactly four values")
    values = [int(p) for p in parts]
    for value in values:
        if value < -100 or value > 100:
            raise argparse.ArgumentTypeError("values must be in -100..100")
    return values


def parse_signs(text: str) -> list[int]:
    values = parse_values(text)
    for value in values:
        if value not in (-1, 1):
            raise argparse.ArgumentTypeError("sign values must be -1 or 1")
    return values


def parse_pulse(text: str) -> tuple[str, int, int]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected WHEEL PWM MS, for example FL 8 200")
    wheel = parts[0].upper()
    if wheel not in {"FL", "FR", "BL", "BR", "0", "1", "2", "3"}:
        raise argparse.ArgumentTypeError("wheel must be FL, FR, BL, BR, 0, 1, 2, or 3")
    pwm = int(parts[1])
    ms = int(parts[2])
    if abs(pwm) > 45:
        raise argparse.ArgumentTypeError("pulse PWM must be in -45..45")
    if ms < 50 or ms > 400:
        raise argparse.ArgumentTypeError("pulse duration must be 50..400 ms")
    return wheel, pwm, ms


def parse_telemetry(line: str) -> Optional[Telemetry]:
    parts = line.strip().split()
    if len(parts) != 23 or parts[0] != "T":
        return None

    try:
        return Telemetry(
            seq=int(parts[1]),
            ms=int(parts[2]),
            mode=int(parts[3]),
            enabled=bool(int(parts[4])),
            limit=int(parts[5]),
            enc=[int(x) for x in parts[6:10]],
            tps=[float(x) for x in parts[10:14]],
            pwm=[int(x) for x in parts[14:18]],
            target=[float(x) for x in parts[18:22]],
            fault=int(parts[22]),
        )
    except ValueError:
        return None


def send_line(ser, line: str) -> None:
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()


def format_telemetry(t: Telemetry) -> str:
    mode_name = {0: "STOP", 1: "PWM", 2: "VEL"}.get(t.mode, str(t.mode))
    return (
        f"seq={t.seq:06d} mode={mode_name:<4} enabled={int(t.enabled)} "
        f"limit={t.limit:02d} enc={t.enc} tps={[round(x, 1) for x in t.tps]} "
        f"pwm={t.pwm} target={[round(x, 1) for x in t.target]} fault=0x{t.fault:04x}"
    )


def open_serial(port: str, baud: int):
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - depends on Jetson package state.
        raise SystemExit("Missing pyserial. Install python3-serial or pyserial.") from exc

    return serial.Serial(port=port, baudrate=baud, timeout=0.2)


def run(args: argparse.Namespace) -> int:
    ser = open_serial(args.port, args.baud)
    print(f"Connected to {args.port} at {args.baud}")

    sent_motion = False
    repeat_motion = False
    try:
        if args.ping:
            send_line(ser, "PING")

        if args.status:
            send_line(ser, "STATUS")

        if args.send_stop:
            send_line(ser, "STOP")
            send_line(ser, "ENABLE 0")
            print("Sent STOP and ENABLE 0")

        if args.telemetry_hz is not None:
            send_line(ser, f"TELEM {args.telemetry_hz}")

        if args.zero_enc:
            send_line(ser, "ZERO_ENC")

        if args.motor_sign is not None:
            send_line(ser, "MOTOR_SIGN " + " ".join(str(v) for v in args.motor_sign))

        if args.enc_sign is not None:
            send_line(ser, "ENC_SIGN " + " ".join(str(v) for v in args.enc_sign))

        if args.bench_test or args.enable_motors or args.pulse is not None:
            if not (args.bench_test and args.enable_motors):
                raise SystemExit("Bench motion requires both --bench-test and --enable-motors")
            if max(abs(v) for v in args.values) > args.limit:
                raise SystemExit("--values exceed --limit; keep first tests around +/-10 to +/-15")
            if args.pulse is not None and abs(args.pulse[1]) > args.limit:
                raise SystemExit("--pulse PWM exceeds --limit")

            send_line(ser, f"LIMIT {args.limit}")
            send_line(ser, "ENABLE 1")
            sent_motion = True
            repeat_motion = args.pulse is None
            print("Bench command enabled. Wheels must be lifted and power must be supervised.")

            if args.pulse is not None:
                wheel, pwm, ms = args.pulse
                send_line(ser, f"PULSE {wheel} {pwm} {ms}")
                print(f"Sent one pulse: wheel={wheel} pwm={pwm} ms={ms}")

        start = time.monotonic()
        next_motion = 0.0
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            now = time.monotonic()
            if repeat_motion and now >= next_motion:
                values = " ".join(str(v) for v in args.values)
                send_line(ser, f"{args.mode.upper()} {values}")
                next_motion = now + 0.1

            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("ascii", errors="replace").strip()
            telemetry = parse_telemetry(line)
            if telemetry:
                print(format_telemetry(telemetry))
            elif args.show_raw:
                print(line)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        if sent_motion or args.send_stop_on_exit:
            try:
                send_line(ser, "STOP")
                send_line(ser, "ENABLE 0")
                print("Sent STOP and ENABLE 0 on exit")
            except Exception as exc:  # pragma: no cover - best effort shutdown.
                print(f"Could not send stop on exit: {exc}", file=sys.stderr)
        ser.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, help="STM32 serial port, prefer /dev/serial/by-id when available")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; 0 means until Ctrl-C")
    parser.add_argument("--show-raw", action="store_true", help="print non-telemetry serial lines")
    parser.add_argument("--send-stop", action="store_true", help="send STOP and disable motors at startup")
    parser.add_argument("--send-stop-on-exit", action="store_true", help="send STOP and disable motors when exiting")
    parser.add_argument("--telemetry-hz", type=int, default=None, help="request STM32 telemetry rate")
    parser.add_argument("--ping", action="store_true", help="send PING; non-motion")
    parser.add_argument("--status", action="store_true", help="request firmware status/pin map; non-motion")
    parser.add_argument("--zero-enc", action="store_true", help="zero encoder counts; non-motion")
    parser.add_argument("--motor-sign", type=parse_signs, default=None, help="runtime motor signs, e.g. '1 -1 1 -1'; non-motion")
    parser.add_argument("--enc-sign", type=parse_signs, default=None, help="runtime encoder signs, e.g. '1 1 -1 1'; non-motion")

    parser.add_argument("--bench-test", action="store_true", help="acknowledge lifted-wheel supervised bench test")
    parser.add_argument("--enable-motors", action="store_true", help="allow motor output; requires --bench-test")
    parser.add_argument("--mode", choices=["pwm", "vel"], default="pwm")
    parser.add_argument("--values", type=parse_values, default=[0, 0, 0, 0], help="four PWM values or four target ticks/sec")
    parser.add_argument("--pulse", type=parse_pulse, default=None, help="one short pulse: WHEEL PWM MS, e.g. 'FL 8 200'")
    parser.add_argument("--limit", type=int, default=15, help="PWM limit for bench command; keep first tests at 10..15")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
