#!/usr/bin/env python3
"""
Safe dual-servo gate test for the Jetson 40-pin header.

Default mode is dry-run only. Add --live-servo only after both servos are
powered from a suitable external 5V-6V supply, grounds are shared with the
Jetson, and the mechanism is clear.
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


class ServoGate:
    def __init__(self, name: str, pin: int, backend: str, frequency_hz: float) -> None:
        self.name = name
        self.pin = pin
        self.backend = backend
        self.frequency_hz = frequency_hz
        self.gpio = None
        self.pwm = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.pulse_us = 1500

    def open(self) -> None:
        _prepare_jetson_gpio_model_env()
        import Jetson.GPIO as GPIO

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self.gpio = GPIO
        if self.backend == "gpio-bitbang":
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._bitbang_loop, daemon=True)
            self.thread.start()
            return
        self.pwm = GPIO.PWM(self.pin, self.frequency_hz)
        self.pwm.start(0.0)

    def _bitbang_loop(self) -> None:
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
        self.pulse_us = int(pulse_us)
        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(pulse_to_duty(self.pulse_us, self.frequency_hz))

    def close(self, final_us: int, hold_s: float, detach: bool) -> None:
        if self.gpio is None:
            return
        try:
            self.write_us(final_us)
            time.sleep(max(0.0, hold_s))
            if self.pwm is not None:
                if detach:
                    self.pwm.ChangeDutyCycle(0.0)
                self.pwm.stop()
                self.pwm = None
            if self.thread is not None:
                self.stop_event.set()
                self.thread.join(timeout=1.0)
                self.thread = None
            try:
                self.gpio.output(self.pin, self.gpio.LOW)
            except RuntimeError:
                pass
            try:
                self.gpio.cleanup(self.pin)
            except RuntimeError:
                pass
        finally:
            self.gpio = None


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


def move_pair(
    gate_a: ServoGate,
    gate_b: ServoGate,
    current_a: int,
    target_a: int,
    current_b: int,
    target_b: int,
    args: argparse.Namespace,
) -> tuple[int, int]:
    thread_a = threading.Thread(target=move_servo, args=(gate_a, current_a, target_a, args))
    thread_b = threading.Thread(target=move_servo, args=(gate_b, current_b, target_b, args))
    thread_a.start()
    thread_b.start()
    thread_a.join()
    thread_b.join()
    return target_a, target_b


def print_plan(args: argparse.Namespace) -> None:
    print("Dual servo gate test plan:")
    print(f"  gate A signal pin: Jetson BOARD {args.pin_a}")
    print(f"  gate B signal pin: Jetson BOARD {args.pin_b}")
    print(f"  backend: {args.backend}")
    print(f"  frequency: {args.frequency_hz:.1f} Hz")
    print(f"  gate A home/open: {args.home_a_us} us -> {args.open_a_us} us")
    print(f"  gate B home/open: {args.home_b_us} us -> {args.open_b_us} us")
    print(f"  sequence: {args.sequence}")
    print(f"  cycles: {args.cycles}")
    print(f"  movement time: {args.move_s:.2f} s each move")
    print(f"  open hold before closing: {args.open_hold_s:.2f} s")
    print(f"  final detach: {args.detach_final}")
    print()
    print("Wiring reminder:")
    print("  Servo A signal -> Jetson BOARD 32")
    print("  Servo B signal -> Jetson BOARD 33")
    print("  Servo power red -> external 5V-6V +")
    print("  Servo brown/black -> external GND")
    print("  External GND -> Jetson GND")
    if not args.live_servo:
        print()
        print("Dry-run only. Add --live-servo to actually move both servos.")


def run_cycle(gate_a: ServoGate, gate_b: ServoGate, args: argparse.Namespace) -> None:
    current_a = args.home_a_us
    current_b = args.home_b_us
    print("Sending both gates home")
    gate_a.write_us(current_a)
    gate_b.write_us(current_b)
    time.sleep(args.home_hold_s)

    for cycle in range(args.cycles):
        print(f"Cycle {cycle + 1}/{args.cycles}: open")
        if args.sequence == "together":
            current_a, current_b = move_pair(
                gate_a, gate_b, current_a, args.open_a_us, current_b, args.open_b_us, args
            )
        elif args.sequence == "a-then-b":
            move_servo(gate_a, current_a, args.open_a_us, args)
            current_a = args.open_a_us
            time.sleep(args.between_servo_s)
            move_servo(gate_b, current_b, args.open_b_us, args)
            current_b = args.open_b_us
        elif args.sequence == "b-then-a":
            move_servo(gate_b, current_b, args.open_b_us, args)
            current_b = args.open_b_us
            time.sleep(args.between_servo_s)
            move_servo(gate_a, current_a, args.open_a_us, args)
            current_a = args.open_a_us
        else:
            raise RuntimeError(f"unknown sequence {args.sequence}")

        print(f"Hold open for {args.open_hold_s:.2f}s")
        time.sleep(args.open_hold_s)

        print(f"Cycle {cycle + 1}/{args.cycles}: close/home")
        if args.sequence == "together":
            current_a, current_b = move_pair(
                gate_a, gate_b, current_a, args.home_a_us, current_b, args.home_b_us, args
            )
        elif args.sequence == "a-then-b":
            move_servo(gate_a, current_a, args.home_a_us, args)
            current_a = args.home_a_us
            time.sleep(args.between_servo_s)
            move_servo(gate_b, current_b, args.home_b_us, args)
            current_b = args.home_b_us
        else:
            move_servo(gate_b, current_b, args.home_b_us, args)
            current_b = args.home_b_us
            time.sleep(args.between_servo_s)
            move_servo(gate_a, current_a, args.home_a_us, args)
            current_a = args.home_a_us

        time.sleep(args.home_hold_s)


def run(args: argparse.Namespace) -> int:
    args.home_a_us = bounded_pulse_us(args.home_a_us, args)
    args.open_a_us = bounded_pulse_us(args.open_a_us, args)
    args.home_b_us = bounded_pulse_us(args.home_b_us, args)
    args.open_b_us = bounded_pulse_us(args.open_b_us, args)
    print_plan(args)
    if not args.live_servo:
        return 0

    gate_a = ServoGate("A", args.pin_a, args.backend, args.frequency_hz)
    gate_b = ServoGate("B", args.pin_b, args.backend, args.frequency_hz)
    gate_a.open()
    gate_b.open()
    try:
        run_cycle(gate_a, gate_b, args)
    except KeyboardInterrupt:
        print("\nInterrupted; returning both gates home")
    finally:
        gate_a.close(args.home_a_us, args.home_hold_s, args.detach_final)
        gate_b.close(args.home_b_us, args.home_hold_s, args.detach_final)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pin-a", type=int, default=32, help="Jetson BOARD pin for gate A signal")
    parser.add_argument("--pin-b", type=int, default=33, help="Jetson BOARD pin for gate B signal")
    parser.add_argument("--backend", choices=["hardware-pwm", "gpio-bitbang"], default="hardware-pwm")
    parser.add_argument("--frequency-hz", type=float, default=50.0)
    parser.add_argument("--home-a-us", type=int, default=1500)
    parser.add_argument("--open-a-us", type=int, default=2000)
    parser.add_argument("--home-b-us", type=int, default=1500)
    parser.add_argument("--open-b-us", type=int, default=2000)
    parser.add_argument("--min-us", type=int, default=900)
    parser.add_argument("--max-us", type=int, default=2100)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--sequence", choices=["together", "a-then-b", "b-then-a"], default="together")
    parser.add_argument("--between-servo-s", type=float, default=0.25)
    parser.add_argument("--move-s", type=float, default=0.60)
    parser.add_argument("--step-us", type=int, default=10)
    parser.add_argument("--open-hold-s", type=float, default=7.00)
    parser.add_argument("--home-hold-s", type=float, default=0.80)
    parser.add_argument("--detach-final", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--live-servo", action="store_true", help="actually drive both servo PWM signals")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
