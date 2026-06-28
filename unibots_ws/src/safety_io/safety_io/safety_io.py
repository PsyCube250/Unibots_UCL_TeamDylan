import json
import fcntl
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


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


class GpioSafety:
    def __init__(self, node):
        self.node = node
        self.gpio = None
        self.enabled = False
        self.reason = 'gpio disabled'
        self.raw_level = None
        self.red_on = False
        self.green_on = False

        self.button_pin = int(node.get_parameter('button_pin').value)
        self.red_pin = int(node.get_parameter('red_led_pin').value)
        self.green_pin = int(node.get_parameter('green_led_pin').value)
        self.button_active_low = bool(node.get_parameter('button_active_low').value)
        self.button_pull_up = bool(node.get_parameter('button_pull_up').value)
        self.led_active_low = bool(node.get_parameter('led_active_low').value)
        self.no_leds = bool(node.get_parameter('no_leds').value)

    def open(self):
        if not bool(self.node.get_parameter('enable_gpio').value):
            self.reason = 'gpio disabled by parameter'
            return

        prepare_jetson_gpio_model_env()
        try:
            import Jetson.GPIO as GPIO
        except Exception as exc:
            self.reason = f'Jetson.GPIO unavailable: {exc}'
            if bool(self.node.get_parameter('require_gpio').value):
                raise RuntimeError(self.reason) from exc
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            pud = GPIO.PUD_UP if self.button_pull_up else GPIO.PUD_OFF
            GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=pud)
            if not self.no_leds:
                GPIO.setup(self.red_pin, GPIO.OUT, initial=self.led_level(True))
                GPIO.setup(self.green_pin, GPIO.OUT, initial=self.led_level(False))
                self.red_on = True
                self.green_on = False
            self.gpio = GPIO
            self.enabled = True
            self.reason = 'ok'
        except Exception as exc:
            self.reason = f'gpio open failed: {exc}'
            if bool(self.node.get_parameter('require_gpio').value):
                raise RuntimeError(self.reason) from exc

    def led_level(self, on):
        if self.led_active_low:
            return 0 if on else 1
        return 1 if on else 0

    def read_button_active(self):
        if not self.enabled or self.gpio is None:
            return False
        raw = int(self.gpio.input(self.button_pin))
        self.raw_level = raw
        return (raw == 0) if self.button_active_low else (raw == 1)

    def write_leds(self, red, green):
        self.red_on = bool(red)
        self.green_on = bool(green)
        if not self.enabled or self.no_leds or self.gpio is None:
            return
        self.gpio.output(self.red_pin, self.led_level(self.red_on))
        self.gpio.output(self.green_pin, self.led_level(self.green_on))

    def close(self):
        if self.enabled:
            self.write_leds(True, False)
        self.gpio = None
        self.enabled = False


class ModulinoButtonsSafety:
    I2C_SLAVE = 0x0703

    def __init__(self, node):
        self.node = node
        self.fd = None
        self.enabled = False
        self.reason = 'i2c disabled'
        self.raw_level = None
        self.red_on = False
        self.green_on = False
        self.gpio = None
        self.signal_led_pin = None
        self.no_leds = bool(node.get_parameter('no_leds').value)
        self.led_active_low = bool(node.get_parameter('led_active_low').value)

        self.bus_number = int(node.get_parameter('modulino_i2c_bus').value)
        self.address = int(node.get_parameter('modulino_i2c_address').value)
        self.button_index = int(node.get_parameter('modulino_button_index').value)
        self.signal_led_pin = int(node.get_parameter('red_led_pin').value)
        if self.button_index < 0 or self.button_index > 2:
            self.button_index = 0

    def open(self):
        if not bool(self.node.get_parameter('enable_gpio').value):
            self.reason = 'i2c safety disabled by parameter'
            return

        path = f'/dev/i2c-{self.bus_number}'
        try:
            self.fd = os.open(path, os.O_RDWR | os.O_CLOEXEC)
            fcntl.ioctl(self.fd, self.I2C_SLAVE, self.address)
            self.read_button_active()
            self.open_signal_led()
            self.enabled = True
            self.reason = 'ok'
        except Exception as exc:
            self.reason = f'modulino i2c open failed: {exc}'
            self.close()
            if bool(self.node.get_parameter('require_gpio').value):
                raise RuntimeError(self.reason) from exc

    def open_signal_led(self):
        if self.no_leds or self.signal_led_pin <= 0:
            return
        prepare_jetson_gpio_model_env()
        try:
            import Jetson.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.signal_led_pin, GPIO.OUT, initial=self.led_level(True))
            self.gpio = GPIO
        except Exception as exc:
            self.node.get_logger().warn(f'signal LED unavailable on pin {self.signal_led_pin}: {exc}')
            self.gpio = None

    def led_level(self, on):
        if self.led_active_low:
            return 0 if on else 1
        return 1 if on else 0

    def read_states(self):
        if self.fd is None:
            raise RuntimeError('i2c bus is not open')
        data = os.read(self.fd, 4)
        if len(data) < 4:
            raise RuntimeError(f'short i2c read: {len(data)} bytes')
        return list(data[1:4])

    def read_button_active(self):
        if not self.enabled and self.fd is None:
            return False
        try:
            states = self.read_states()
        except Exception as exc:
            self.reason = f'modulino i2c read failed: {exc}'
            self.enabled = False
            self.raw_level = None
            return False
        self.raw_level = states
        self.enabled = True
        self.reason = 'ok'
        return bool(states[self.button_index])

    def write_leds(self, red, green):
        self.red_on = bool(red)
        self.green_on = bool(green)
        if self.gpio is not None and self.signal_led_pin is not None:
            self.gpio.output(self.signal_led_pin, self.led_level(self.red_on or self.green_on))
        if self.fd is None:
            return
        try:
            os.write(self.fd, bytes([1 if self.red_on else 0, 1 if self.green_on else 0, 0]))
        except Exception as exc:
            self.reason = f'modulino i2c led write failed: {exc}'

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        if self.gpio is not None and self.signal_led_pin is not None:
            try:
                self.gpio.output(self.signal_led_pin, self.led_level(True))
                self.gpio.cleanup(self.signal_led_pin)
            except Exception:
                pass
        self.fd = None
        self.gpio = None
        self.enabled = False


class SafetyIoNode(Node):
    def __init__(self):
        super().__init__('safety_io')

        self.declare_parameter('enable_gpio', True)
        self.declare_parameter('require_gpio', False)
        self.declare_parameter('button_backend', 'gpio')
        self.declare_parameter('button_pin', 7)
        self.declare_parameter('button_active_low', True)
        self.declare_parameter('button_pull_up', True)
        self.declare_parameter('modulino_i2c_bus', 1)
        self.declare_parameter('modulino_i2c_address', 62)
        self.declare_parameter('modulino_button_index', 0)
        self.declare_parameter('red_led_pin', 29)
        self.declare_parameter('green_led_pin', 31)
        self.declare_parameter('led_active_low', False)
        self.declare_parameter('no_leds', False)
        self.declare_parameter('default_paused', False)
        self.declare_parameter('debounce_s', 0.25)
        self.declare_parameter('blink_s', 0.35)
        self.declare_parameter('decision_stale_s', 1.5)
        self.declare_parameter('publish_hz', 20.0)

        self.button_backend = str(self.get_parameter('button_backend').value).strip().lower()
        if self.button_backend in ('i2c', 'modulino', 'modulino_i2c', 'modulino-buttons'):
            self.gpio = ModulinoButtonsSafety(self)
        else:
            self.button_backend = 'gpio'
            self.gpio = GpioSafety(self)
        self.gpio.open()

        self.paused = bool(self.get_parameter('default_paused').value)
        self.press_count = 0
        self.last_button_active = self.gpio.read_button_active()
        self.last_press_s = time.monotonic()
        self.last_event = 'boot'
        self.decision_state = {}
        self.decision_last_s = 0.0

        self.pub = self.create_publisher(String, '/safety_state', 10)
        self.create_subscription(String, '/decision_state', self.decision_cb, 10)

        period = 1.0 / max(1.0, float(self.get_parameter('publish_hz').value))
        self.timer = self.create_timer(period, self.loop)
        self.get_logger().info(
            f'safety_io ready: backend={self.button_backend}, '
            f'gpio={self.gpio.enabled}, '
            f'initial_button_active={self.last_button_active}, '
            f'default_paused={self.paused}'
        )

    def decision_cb(self, msg):
        try:
            self.decision_state = json.loads(msg.data)
            self.decision_last_s = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Bad /decision_state payload: {exc}')

    def decision_alive(self, now):
        if self.decision_last_s <= 0.0:
            return False
        return now - self.decision_last_s <= float(self.get_parameter('decision_stale_s').value)

    def update_button(self, now):
        active = self.gpio.read_button_active()
        debounce_s = float(self.get_parameter('debounce_s').value)
        if active and not self.last_button_active and now - self.last_press_s >= debounce_s:
            self.paused = not self.paused
            self.press_count += 1
            self.last_press_s = now
            self.last_event = 'pause' if self.paused else 'resume'
        self.last_button_active = active

    def update_leds(self, now):
        if self.paused:
            self.gpio.write_leds(True, False)
            return 'red'

        if self.decision_alive(now):
            blink_s = max(0.05, float(self.get_parameter('blink_s').value))
            green_on = int(now / blink_s) % 2 == 0
            self.gpio.write_leds(False, green_on)
            return 'green_blink' if green_on else 'off_blink'

        self.gpio.write_leds(False, True)
        return 'green'

    def loop(self):
        now = time.monotonic()
        self.update_button(now)
        led = self.update_leds(now)

        msg = String()
        msg.data = json.dumps({
            'enabled': self.gpio.enabled,
            'backend': self.button_backend,
            'paused': self.paused,
            'run_allowed': self.gpio.enabled and not self.paused,
            'raw_level': self.gpio.raw_level,
            'press_count': self.press_count,
            'event': self.last_event,
            'led': led,
            'red_on': self.gpio.red_on,
            'green_on': self.gpio.green_on,
            'decision_alive': self.decision_alive(now),
            'decision_state': self.decision_state.get('state'),
            'collected': self.decision_state.get('collected'),
            'target_count': self.decision_state.get('target_count'),
            'reason': self.gpio.reason,
            'timestamp': time.time(),
        })
        self.pub.publish(msg)

    def destroy_node(self):
        self.gpio.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyIoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
