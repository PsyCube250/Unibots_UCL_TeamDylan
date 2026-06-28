#!/usr/bin/env python3
"""
Safe servo gate open/close test for the Jetson 40-pin header.

Default mode is dry-run only. Add --live-servo only after the servo is powered
from a suitable 5V supply, the signal ground is shared with the Jetson, and the
mechanism is clear.
"""

from __future__ import annotations

import argparse
import threading
import time

from jetson_safety_imu import _prepare_jetson_gpio_model_env


def pulse_to_duty(pulse_us: float, frequency_hz: float) -> float:
    period_us = 1_000_000.0 / frequency_hz
    return max(0.0, min(100.0, pulse_us * 100.0 / period_us))


def bounded_pulse_us(value: int, args: argparse.Namespace) -> int:
    if value < args.min_us or value > args.max_us:
        raise SystemExit(f"Pulse {value}us is outside safe range {args.min_us}..{args.max_us}us")
    return value


def pulse_steps(start_us: int, end_us: int, step_us: int) -> list[int]:
    if start_us == end_us:
        return [end_us]
    direction = 1 if end_us > start_us else -1
    step = max(1, abs(step_us)) * direction
    values = list(range(start_us, end_us, step))
    if not values or values[-1] != end_us:
        values.append(end_us)
    return values


class HardwarePwmServoGate:
    def __init__(self, pin: int, frequency_hz: float) -> None:
        self.pin = pin
        self.frequency_hz = frequency_hz
        self.gpio = None
        self.pwm = None

    def open(self) -> None:
        _prepare_jetson_gpio_model_env()
        import Jetson.GPIO as GPIO

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self.gpio = GPIO
        self.pwm = GPIO.PWM(self.pin, self.frequency_hz)
        self.pwm.start(0.0)

    def write_us(self, pulse_us: int) -> None:
        if self.pwm is None:
            raise RuntimeError("Servo PWM not opened")
        self.pwm.ChangeDutyCycle(pulse_to_duty(pulse_us, self.frequency_hz))

    def close(self, final_us: int, hold_s: float, detach: bool) -> None:
        if self.pwm is not None:
            try:
                self.write_us(final_us)
                time.sleep(max(0.0, hold_s))
                if detach:
                    self.pwm.ChangeDutyCycle(0.0)
                    self.pwm.stop()
            finally:
                self.pwm = None
        if self.gpio is not None:
            self.gpio.cleanup(self.pin)
            self.gpio = None


class BitbangServoGate:
    def __init__(self, pin: int, frequency_hz: float) -> None:
        self.pin = pin
        self.frequency_hz = frequency_hz
        self.gpio = None
        self.pulse_us = 1500
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None

    def open(self) -> None:
        _prepare_jetson_gpio_model_env()
        import Jetson.GPIO as GPIO

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self.gpio = GPIO
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        period_s = 1.0 / self.frequency_hz
        while not self.stop_event.is_set():
            pulse_s = max(0.0005, min(0.0025, self.pulse_us / 1_000_000.0))
            start_s = time.perf_counter()
            self.gpio.output(self.pin, self.gpio.HIGH)
            time.sleep(pulse_s)
            self.gpio.output(self.pin, self.gpio.LOW)
            elapsed_s = time.perf_counter() - start_s
            time.sleep(max(0.0, period_s - elapsed_s))

    def write_us(self, pulse_us: int) -> None:
        self.pulse_us = pulse_us

    def close(self, final_us: int, hold_s: float, detach: bool) -> None:
        if self.gpio is None:
            return
        self.write_us(final_us)
        time.sleep(max(0.0, hold_s))
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        self.gpio.output(self.pin, self.gpio.LOW)
        self.gpio.cleanup(self.pin)
        self.gpio = None


def make_gate(args: argparse.Namespace):
    if args.backend == "gpio-bitbang":
        return BitbangServoGate(args.pin, args.frequency_hz)
    return HardwarePwmServoGate(args.pin, args.frequency_hz)


def move_servo(gate: ServoGate, start_us: int, end_us: int, args: argparse.Namespace) -> None:
    values = pulse_steps(start_us, end_us, args.step_us)
    if len(values) <= 1:
        gate.write_us(end_us)
        time.sleep(args.move_s)
        return
    delay_s = max(0.0, args.move_s / max(1, len(values) - 1))
    for pulse in values:
        gate.write_us(pulse)
        time.sleep(delay_s)


def print_plan(args: argparse.Namespace) -> None:
    direction = "larger pulse" if args.open_us > args.home_us else "smaller pulse"
    print("Servo gate test plan:")
    print(f"  signal pin: Jetson BOARD {args.pin}")
    print(f"  backend: {args.backend}")
    print(f"  frequency: {args.frequency_hz:.1f} Hz")
    print(f"  closed/home: {args.home_us} us")
    print(f"  open target: {args.open_us} us ({direction})")
    print(f"  cycles: {args.cycles}")
    print(f"  movement time: {args.move_s:.2f} s each way")
    print(f"  open hold: {args.open_hold_s:.2f} s")
    print(f"  final detach: {args.detach_final}")
    if not args.live_servo:
        print("Dry-run only. Add --live-servo to actually move the servo.")


def run(args: argparse.Namespace) -> int:
    args.home_us = bounded_pulse_us(args.home_us, args)
    args.open_us = bounded_pulse_us(args.open_us, args)
    print_plan(args)
    if not args.live_servo:
        return 0

    gate = make_gate(args)
    gate.open()
    current_us = args.home_us
    try:
        print("Sending home pulse")
        gate.write_us(args.home_us)
        time.sleep(args.home_hold_s)
        for cycle in range(args.cycles):
            print(f"Cycle {cycle + 1}/{args.cycles}: open")
            move_servo(gate, current_us, args.open_us, args)
            current_us = args.open_us
            time.sleep(args.open_hold_s)
            print(f"Cycle {cycle + 1}/{args.cycles}: home")
            move_servo(gate, current_us, args.home_us, args)
            current_us = args.home_us
            time.sleep(args.home_hold_s)
    except KeyboardInterrupt:
        print("\nInterrupted; returning to home")
    finally:
        gate.close(args.home_us, args.home_hold_s, args.detach_final)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pin", type=int, default=32, help="Jetson BOARD pin for servo signal")
    parser.add_argument("--backend", choices=["hardware-pwm", "gpio-bitbang"], default="hardware-pwm")
    parser.add_argument("--frequency-hz", type=float, default=50.0)
    parser.add_argument("--home-us", type=int, default=1500, help="closed/home pulse width")
    parser.add_argument("--open-us", type=int, default=2000, help="open pulse width; use 1000 if direction is reversed")
    parser.add_argument("--min-us", type=int, default=900)
    parser.add_argument("--max-us", type=int, default=2100)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--move-s", type=float, default=0.60)
    parser.add_argument("--step-us", type=int, default=10)
    parser.add_argument("--open-hold-s", type=float, default=1.00)
    parser.add_argument("--home-hold-s", type=float, default=0.80)
    parser.add_argument("--detach-final", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--live-servo", action="store_true", help="actually drive the servo PWM")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
