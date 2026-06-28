#!/usr/bin/env python3
"""
Jetson safety I/O and IMU helpers.

This module is intentionally safe to import on non-Jetson machines. Hardware
libraries are imported only when the matching feature is opened.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class SafetySnapshot:
    enabled: bool
    kill_active: bool = False
    raw_level: Optional[int] = None
    red_on: bool = False
    green_on: bool = False
    reason: str = "safety io disabled"


class SafetyIO:
    def __init__(
        self,
        kill_pin: int = 7,
        red_pin: int = 29,
        green_pin: int = 31,
        kill_active_high: bool = True,
        led_active_low: bool = False,
        pull_up: bool = True,
        no_leds: bool = False,
    ) -> None:
        self.kill_pin = kill_pin
        self.red_pin = red_pin
        self.green_pin = green_pin
        self.kill_active_high = kill_active_high
        self.led_active_low = led_active_low
        self.pull_up = pull_up
        self.no_leds = no_leds
        self.gpio = None
        self.enabled = False
        self.led_red = False
        self.led_green = False
        self._last_reason = "safety io disabled"

    def open(self, require: bool = False) -> None:
        _prepare_jetson_gpio_model_env()
        try:
            import Jetson.GPIO as GPIO
        except Exception as exc:
            if require:
                raise RuntimeError(f"Jetson.GPIO unavailable: {exc}") from exc
            self._last_reason = f"Jetson.GPIO unavailable: {exc}"
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            pud = GPIO.PUD_UP if self.pull_up else GPIO.PUD_OFF
            GPIO.setup(self.kill_pin, GPIO.IN, pull_up_down=pud)
            if not self.no_leds:
                GPIO.setup(self.red_pin, GPIO.OUT, initial=self._gpio_level(True))
                GPIO.setup(self.green_pin, GPIO.OUT, initial=self._gpio_level(False))
                self.led_red = True
                self.led_green = False
            self.gpio = GPIO
            self.enabled = True
            self._last_reason = "ok"
        except Exception as exc:
            if require:
                raise RuntimeError(f"Could not open Jetson safety GPIO: {exc}") from exc
            self._last_reason = f"gpio open failed: {exc}"

    def _gpio_level(self, on: bool) -> int:
        if self.led_active_low:
            return 0 if on else 1
        return 1 if on else 0

    def _write_leds(self, red: bool, green: bool) -> None:
        self.led_red = red
        self.led_green = green
        if not self.enabled or self.no_leds or self.gpio is None:
            return
        self.gpio.output(self.red_pin, self._gpio_level(red))
        self.gpio.output(self.green_pin, self._gpio_level(green))

    def read(self) -> SafetySnapshot:
        if not self.enabled or self.gpio is None:
            return SafetySnapshot(False, reason=self._last_reason)
        raw = int(self.gpio.input(self.kill_pin))
        kill = bool(raw) if self.kill_active_high else not bool(raw)
        return SafetySnapshot(
            enabled=True,
            kill_active=kill,
            raw_level=raw,
            red_on=self.led_red,
            green_on=self.led_green,
            reason="kill active" if kill else "ok",
        )

    def update_leds(self, *, kill: bool, running: bool, warning: bool = False) -> SafetySnapshot:
        now = time.monotonic()
        if kill:
            self._write_leds(True, False)
        elif warning:
            self._write_leds(True, True)
        elif running:
            self._write_leds(False, True)
        else:
            self._write_leds(int(now / 0.50) % 2 == 0, False)
        return self.read()

    def close(self) -> None:
        if self.gpio is None:
            return
        try:
            self._write_leds(True, False)
        finally:
            # Do not GPIO.cleanup() globally; other Jetson processes may own
            # unrelated pins. Leave pins in a conservative red state.
            self.gpio = None
            self.enabled = False


def _read_text(path: str) -> str:
    try:
        with open(path, "rb") as handle:
            return handle.read().decode("utf-8", errors="replace").replace("\x00", "\n")
    except OSError:
        return ""


def _prepare_jetson_gpio_model_env() -> None:
    if os.environ.get("JETSON_MODEL_NAME"):
        return
    model = _read_text("/proc/device-tree/model")
    compatible = _read_text("/proc/device-tree/compatible")
    text = f"{model}\n{compatible}"
    if "Jetson Orin Nano" in text or "p3768-0000+p3767-0005-super" in text:
        os.environ["JETSON_MODEL_NAME"] = "JETSON_ORIN_NANO"


@dataclass
class ImuSample:
    enabled: bool
    ok: bool = False
    sensor: str = ""
    bus: Optional[int] = None
    address: Optional[int] = None
    yaw_deg: Optional[float] = None
    gyro_z_rad_s: Optional[float] = None
    accel_m_s2: Optional[tuple[float, float, float]] = None
    age_s: float = 999.0
    reason: str = "imu disabled"


def _signed16(lo: int, hi: int) -> int:
    value = lo | (hi << 8)
    return value - 65536 if value & 0x8000 else value


class ImuReader:
    LSM6DS_ADDRS = (0x6A, 0x6B)
    MPU6050_ADDRS = (0x68, 0x69)

    def __init__(
        self,
        bus: str = "auto",
        address: str = "auto",
        calibrate_seconds: float = 1.5,
    ) -> None:
        self.bus_arg = bus
        self.address_arg = address
        self.calibrate_seconds = max(0.0, calibrate_seconds)
        self.bus = None
        self.bus_no: Optional[int] = None
        self.address: Optional[int] = None
        self.sensor = ""
        self.enabled = False
        self.yaw_deg = 0.0
        self.bias_z_rad_s = 0.0
        self.last_s = 0.0
        self.last_ok_s = 0.0
        self.last_reason = "imu disabled"

    def open(self, require: bool = False) -> None:
        try:
            import smbus2 as smbus
        except Exception:
            try:
                import smbus
            except Exception as exc:
                if require:
                    raise RuntimeError(f"smbus unavailable: {exc}") from exc
                self.last_reason = f"smbus unavailable: {exc}"
                return

        bus_candidates = [int(self.bus_arg)] if self.bus_arg != "auto" else [1, 0, 7, 2, 5, 9]
        addr_candidates = (
            [int(self.address_arg, 0)]
            if self.address_arg != "auto"
            else list(self.LSM6DS_ADDRS) + list(self.MPU6050_ADDRS)
        )

        for bus_no in bus_candidates:
            try:
                bus = smbus.SMBus(bus_no)
            except Exception:
                continue
            try:
                found = self._detect_on_bus(bus, addr_candidates)
                if found is not None:
                    sensor, addr = found
                    self.bus = bus
                    self.bus_no = bus_no
                    self.address = addr
                    self.sensor = sensor
                    self._configure()
                    self.enabled = True
                    self.last_s = time.monotonic()
                    self.last_ok_s = self.last_s
                    self._calibrate_bias()
                    self.last_reason = "ok"
                    return
            except Exception:
                pass
            try:
                bus.close()
            except Exception:
                pass

        if require:
            raise RuntimeError("No supported IMU found on candidate I2C buses")
        self.last_reason = "no supported IMU found"

    def _detect_on_bus(self, bus, addrs: list[int]) -> Optional[tuple[str, int]]:
        for addr in addrs:
            if addr in self.LSM6DS_ADDRS:
                try:
                    who = bus.read_byte_data(addr, 0x0F)
                    if who in (0x69, 0x6A, 0x6C):
                        return ("LSM6DS", addr)
                except Exception:
                    pass
            if addr in self.MPU6050_ADDRS:
                try:
                    who = bus.read_byte_data(addr, 0x75)
                    if who in (0x68, 0x69, 0x70, 0x71):
                        return ("MPU6050", addr)
                except Exception:
                    pass
        return None

    def _configure(self) -> None:
        if self.bus is None or self.address is None:
            return
        if self.sensor == "LSM6DS":
            self.bus.write_byte_data(self.address, 0x10, 0x40)  # accel 104 Hz, +/-2g
            self.bus.write_byte_data(self.address, 0x11, 0x40)  # gyro 104 Hz, 245 dps
        elif self.sensor == "MPU6050":
            self.bus.write_byte_data(self.address, 0x6B, 0x00)  # wake
            self.bus.write_byte_data(self.address, 0x1B, 0x00)  # +/-250 dps
            self.bus.write_byte_data(self.address, 0x1C, 0x00)  # +/-2g

    def _calibrate_bias(self) -> None:
        if self.calibrate_seconds <= 0:
            return
        values = []
        deadline = time.monotonic() + self.calibrate_seconds
        while time.monotonic() < deadline:
            gyro_z, _ = self._read_raw()
            values.append(gyro_z)
            time.sleep(0.01)
        if values:
            self.bias_z_rad_s = sum(values) / len(values)

    def _read_raw(self) -> tuple[float, tuple[float, float, float]]:
        if self.bus is None or self.address is None:
            raise RuntimeError("IMU not opened")
        if self.sensor == "LSM6DS":
            data = self.bus.read_i2c_block_data(self.address, 0x22, 12)
            gx = _signed16(data[0], data[1])
            gy = _signed16(data[2], data[3])
            gz = _signed16(data[4], data[5])
            ax = _signed16(data[6], data[7])
            ay = _signed16(data[8], data[9])
            az = _signed16(data[10], data[11])
            gyro_z = gz * 0.00875 * math.pi / 180.0
            accel = (ax * 0.000061 * 9.80665, ay * 0.000061 * 9.80665, az * 0.000061 * 9.80665)
            return gyro_z, accel
        if self.sensor == "MPU6050":
            accel_data = self.bus.read_i2c_block_data(self.address, 0x3B, 6)
            gyro_data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
            ax = _signed16(accel_data[1], accel_data[0])
            ay = _signed16(accel_data[3], accel_data[2])
            az = _signed16(accel_data[5], accel_data[4])
            gz = _signed16(gyro_data[5], gyro_data[4])
            gyro_z = (gz / 131.0) * math.pi / 180.0
            accel = (ax / 16384.0 * 9.80665, ay / 16384.0 * 9.80665, az / 16384.0 * 9.80665)
            return gyro_z, accel
        raise RuntimeError(f"Unsupported IMU {self.sensor}")

    def read(self) -> ImuSample:
        if not self.enabled:
            return ImuSample(False, reason=self.last_reason)
        now = time.monotonic()
        try:
            gyro_z, accel = self._read_raw()
            if self.last_s:
                dt = max(0.0, min(0.20, now - self.last_s))
                self.yaw_deg = (self.yaw_deg + (gyro_z - self.bias_z_rad_s) * dt * 180.0 / math.pi) % 360.0
            self.last_s = now
            self.last_ok_s = now
            self.last_reason = "ok"
            return ImuSample(
                enabled=True,
                ok=True,
                sensor=self.sensor,
                bus=self.bus_no,
                address=self.address,
                yaw_deg=self.yaw_deg,
                gyro_z_rad_s=gyro_z - self.bias_z_rad_s,
                accel_m_s2=accel,
                age_s=0.0,
                reason="ok",
            )
        except Exception as exc:
            age = now - self.last_ok_s if self.last_ok_s else 999.0
            self.last_reason = f"imu read failed: {exc}"
            return ImuSample(
                enabled=True,
                ok=False,
                sensor=self.sensor,
                bus=self.bus_no,
                address=self.address,
                yaw_deg=self.yaw_deg,
                age_s=age,
                reason=self.last_reason,
            )

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
        self.enabled = False


def add_safety_imu_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--enable-safety-io", action="store_true", help="enable Jetson kill switch and red/green LED in monitor mode")
    parser.add_argument("--disable-safety-io", action="store_true", help="allow live mode without Jetson kill switch GPIO")
    parser.add_argument("--kill-pin", type=int, default=7, help="Jetson BOARD pin for kill switch input")
    parser.add_argument("--kill-active-low", action="store_true", help="treat LOW as kill instead of HIGH")
    parser.add_argument("--red-led-pin", type=int, default=29, help="Jetson BOARD pin for red LED")
    parser.add_argument("--green-led-pin", type=int, default=31, help="Jetson BOARD pin for green LED")
    parser.add_argument("--led-active-low", action="store_true", help="use for common-anode LED wiring")
    parser.add_argument("--no-leds", action="store_true", help="read kill switch but do not drive LEDs")
    parser.add_argument("--enable-imu", action="store_true", help="read IMU and publish yaw in monitor state")
    parser.add_argument("--require-imu", action="store_true", help="STOP live motion when IMU is missing or stale")
    parser.add_argument("--imu-bus", default="auto")
    parser.add_argument("--imu-address", default="auto")
    parser.add_argument("--imu-calibrate-seconds", type=float, default=1.5)
    parser.add_argument("--imu-stale-seconds", type=float, default=0.50)


def open_safety_io(args, live: bool) -> SafetyIO:
    safety = SafetyIO(
        kill_pin=args.kill_pin,
        red_pin=args.red_led_pin,
        green_pin=args.green_led_pin,
        kill_active_high=not args.kill_active_low,
        led_active_low=args.led_active_low,
        no_leds=args.no_leds,
    )
    require = (live and not args.disable_safety_io) or args.enable_safety_io
    if require:
        safety.open(require=live and not args.disable_safety_io)
    return safety


def open_imu(args, live: bool) -> ImuReader:
    imu = ImuReader(
        bus=args.imu_bus,
        address=args.imu_address,
        calibrate_seconds=args.imu_calibrate_seconds,
    )
    if args.enable_imu or args.require_imu:
        imu.open(require=args.require_imu and live)
    return imu


def main() -> int:
    parser = argparse.ArgumentParser(description="Read Jetson kill switch/LEDs and IMU without moving motors.")
    add_safety_imu_args(parser)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--period", type=float, default=0.10)
    args = parser.parse_args()
    safety = open_safety_io(args, live=False)
    imu = open_imu(args, live=False)
    start = time.monotonic()
    try:
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            safety_snapshot = safety.read()
            imu_sample = imu.read()
            safety.update_leds(kill=safety_snapshot.kill_active, running=not safety_snapshot.kill_active, warning=imu_sample.enabled and not imu_sample.ok)
            addr = f"0x{imu_sample.address:02x}" if imu_sample.address is not None else "--"
            print(
                f"kill={int(safety_snapshot.kill_active)} raw={safety_snapshot.raw_level} "
                f"imu={imu_sample.sensor or '--'} bus={imu_sample.bus} addr={addr} "
                f"yaw={imu_sample.yaw_deg if imu_sample.yaw_deg is not None else -1:.1f} "
                f"gz={imu_sample.gyro_z_rad_s if imu_sample.gyro_z_rad_s is not None else 0.0:.3f} "
                f"{safety_snapshot.reason} | {imu_sample.reason}",
                flush=True,
            )
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        safety.close()
        imu.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
