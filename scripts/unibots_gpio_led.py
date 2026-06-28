#!/usr/bin/env python3
import argparse
import os
import time


def read_text(path):
    try:
        with open(path, 'rb') as handle:
            return handle.read().decode('utf-8', errors='replace').replace('\x00', '\n')
    except OSError:
        return ''


def prepare_jetson_gpio_model_env():
    if os.environ.get('JETSON_MODEL_NAME'):
        return
    text = read_text('/proc/device-tree/model') + '\n' + read_text('/proc/device-tree/compatible')
    if 'Jetson Orin Nano' in text or 'p3768-0000+p3767-0005-super' in text:
        os.environ['JETSON_MODEL_NAME'] = 'JETSON_ORIN_NANO'


def led_level(on, active_low):
    if active_low:
        return 0 if on else 1
    return 1 if on else 0


def main():
    parser = argparse.ArgumentParser(description='Set Unibots red/green GPIO LED state.')
    parser.add_argument('state', choices=['red', 'green', 'off'])
    parser.add_argument('--red-pin', type=int, default=29)
    parser.add_argument('--green-pin', type=int, default=31)
    parser.add_argument('--led-active-low', action='store_true')
    parser.add_argument('--hold-s', type=float, default=0.0)
    args = parser.parse_args()

    prepare_jetson_gpio_model_env()
    try:
        import Jetson.GPIO as GPIO
    except Exception as exc:
        print(f'Jetson.GPIO unavailable: {exc}', flush=True)
        return 0

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    red = args.state == 'red'
    green = args.state == 'green'
    GPIO.setup(args.red_pin, GPIO.OUT, initial=led_level(red, args.led_active_low))
    GPIO.setup(args.green_pin, GPIO.OUT, initial=led_level(green, args.led_active_low))
    GPIO.output(args.red_pin, led_level(red, args.led_active_low))
    GPIO.output(args.green_pin, led_level(green, args.led_active_low))
    if args.hold_s > 0:
        time.sleep(args.hold_s)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
