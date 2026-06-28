import json
import math
import threading
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except Exception:
    serial = None


SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 115200
MOTOR_SIGN = (1, -1, 1, -1)


def clamp(value, limit):
    return max(-limit, min(limit, int(round(value))))


def parse_int_set(value):
    return {
        int(part.strip())
        for part in str(value).split(',')
        if part.strip()
    }


def parse_int_list(value):
    return [
        int(part.strip())
        for part in str(value).split(',')
        if part.strip()
    ]


def signed_delta_deg(current, start):
    return ((float(current) - float(start) + 180.0) % 360.0) - 180.0


def pulse_to_duty(pulse_us, frequency_hz):
    period_us = 1_000_000.0 / frequency_hz
    return max(0.0, min(100.0, pulse_us * 100.0 / period_us))


def pulse_steps(start_us, end_us, step_us):
    if start_us == end_us:
        return [end_us]
    direction = 1 if end_us > start_us else -1
    step = max(1, abs(step_us)) * direction
    values = list(range(start_us, end_us, step))
    if not values or values[-1] != end_us:
        values.append(end_us)
    return values


class Stm32MotorLink:
    def __init__(self, node, port, baud, live, limit, motor_sign):
        self.node = node
        self.port = port
        self.baud = baud
        self.live = live
        self.limit = limit
        self.motor_sign = motor_sign
        self.ser = None
        self.last_line = ''
        if live:
            if serial is None:
                raise RuntimeError('python3-serial is not available')
            self.ser = serial.Serial(port, baud, timeout=0.08)
            self.preflight()
            self.command('ENABLE 1')
            self.node.get_logger().warn('LIVE motors enabled by decision_maker')
        else:
            self.node.get_logger().warn('decision_maker dry-run: set live_motors and ground_test true to move')

    def command(self, line, log=True):
        line = line.strip()
        if log and line != self.last_line and rclpy.ok():
            self.node.get_logger().info(f'STM32: {line}')
        self.last_line = line
        if self.ser and self.ser.is_open:
            self.ser.write((line + '\n').encode('ascii'))
            self.ser.flush()

    def preflight(self):
        for line in (
            'STOP',
            'ENABLE 0',
            'PING',
            f'LIMIT {self.limit}',
            'MOTOR_SIGN ' + ' '.join(str(v) for v in self.motor_sign),
            'TELEM 0',
        ):
            self.command(line)
            time.sleep(0.08)

    def pwm(self, values):
        values = [clamp(v, self.limit) for v in values]
        self.command('PWM ' + ' '.join(str(v) for v in values))

    def stop(self):
        self.command('STOP')

    def close(self):
        try:
            self.command('STOP', log=False)
            self.command('ENABLE 0', log=False)
        finally:
            if self.ser is not None:
                self.ser.close()


class ServoGate:
    def __init__(self, pin, backend, frequency_hz):
        self.pin = pin
        self.backend = backend
        self.frequency_hz = frequency_hz
        self.gpio = None
        self.pwm = None
        self.thread = None
        self.stop_event = threading.Event()
        self.pulse_us = 1500

    def open(self):
        import Jetson.GPIO as GPIO

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self.gpio = GPIO
        if self.backend == 'gpio-bitbang':
            self.stop_event.clear()
            self.thread = threading.Thread(target=self.bitbang_loop, daemon=True)
            self.thread.start()
        else:
            self.pwm = GPIO.PWM(self.pin, self.frequency_hz)
            self.pwm.start(0.0)

    def bitbang_loop(self):
        period_s = 1.0 / self.frequency_hz
        while not self.stop_event.is_set():
            pulse_s = max(0.0005, min(0.0025, self.pulse_us / 1_000_000.0))
            start = time.perf_counter()
            self.gpio.output(self.pin, self.gpio.HIGH)
            time.sleep(pulse_s)
            self.gpio.output(self.pin, self.gpio.LOW)
            time.sleep(max(0.0, period_s - (time.perf_counter() - start)))

    def write_us(self, pulse_us):
        self.pulse_us = int(pulse_us)
        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(pulse_to_duty(self.pulse_us, self.frequency_hz))

    def close(self, final_us, hold_s, detach=True):
        if self.gpio is None:
            return
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
        self.gpio.output(self.pin, self.gpio.LOW)
        self.gpio.cleanup(self.pin)
        self.gpio = None


@dataclass
class Step:
    action: str = ''
    pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    until_s: float = 0.0
    reason: str = ''


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_maker')

        self.declare_parameter('live_motors', False)
        self.declare_parameter('ground_test', False)
        self.declare_parameter('serial_port', SERIAL_PORT)
        self.declare_parameter('baud_rate', BAUD_RATE)
        self.declare_parameter('limit', 80)
        self.declare_parameter('target_count', 3)
        self.declare_parameter('home_tag_ids', '20,21')
        self.declare_parameter('allow_no_lidar_in_dry_run', True)
        self.declare_parameter('allow_no_lidar_live', False)
        self.declare_parameter('require_ball_detector_live', True)
        self.declare_parameter('search_when_no_ball', True)
        self.declare_parameter('require_safety_io', False)
        self.declare_parameter('safety_stale_s', 0.75)
        self.declare_parameter('mission_timeout_s', 180.0)

        self.declare_parameter('approach_pwm', 60)
        self.declare_parameter('creep_pwm', 50)
        self.declare_parameter('collect_drive_pwm', 45)
        self.declare_parameter('turn_pwm', 60)
        self.declare_parameter('min_turn_pwm', 18)
        self.declare_parameter('search_pwm', 60)
        self.declare_parameter('escape_pwm', 18)
        self.declare_parameter('step_min_s', 0.20)
        self.declare_parameter('step_max_s', 0.75)
        self.declare_parameter('step_forward_s', 0.38)
        self.declare_parameter('step_arc_s', 0.35)
        self.declare_parameter('step_search_s', 0.50)
        self.declare_parameter('step_spin_angle_deg', 55.0)
        self.declare_parameter('center_tolerance_deg', 6.0)
        self.declare_parameter('slow_distance_cm', 55.0)
        self.declare_parameter('collect_distance_cm', 22.0)
        self.declare_parameter('collect_diameter_px', 145.0)
        self.declare_parameter('collect_drive_s', 0.30)
        self.declare_parameter('collect_hold_s', 0.20)
        self.declare_parameter('ball_stale_s', 0.65)

        self.declare_parameter('lidar_stop_m', 0.24)
        self.declare_parameter('lidar_slow_m', 0.42)
        self.declare_parameter('rear_stop_m', 0.18)
        self.declare_parameter('lidar_stale_s', 0.60)

        self.declare_parameter('search_use_imu', True)
        self.declare_parameter('search_imu_turn_deg', 135.0)
        self.declare_parameter('search_imu_step_s', 0.75)
        self.declare_parameter('search_imu_stall_s', 2.0)
        self.declare_parameter('search_imu_progress_epsilon_deg', 4.0)
        self.declare_parameter('search_imu_recover_s', 0.45)
        self.declare_parameter('search_imu_recover_turn_pwm', 30)
        self.declare_parameter('search_imu_recover_reverse_pwm', 14)

        self.declare_parameter('home_approach_cm', 20.0)
        self.declare_parameter('home_distance_tolerance_cm', 4.0)
        self.declare_parameter('home_align_tolerance_deg', 4.0)
        self.declare_parameter('home_align_gain', 1.2)
        self.declare_parameter('home_search_pwm', 34)
        self.declare_parameter('home_search_boost_pwm', 45)
        self.declare_parameter('home_approach_pwm', 32)
        self.declare_parameter('home_creep_pwm', 24)
        self.declare_parameter('home_turn_direction', 'right')
        self.declare_parameter('home_turn_180_deg', 180.0)
        self.declare_parameter('home_turn_180_pwm', 30)
        self.declare_parameter('home_turn_boost_pwm', 42)
        self.declare_parameter('home_turn_tolerance_deg', 5.0)
        self.declare_parameter('home_turn_stall_s', 1.60)
        self.declare_parameter('home_backup_pwm', 18)
        self.declare_parameter('home_backup_force_pwm', 26)
        self.declare_parameter('home_backup_force_s', 1.30)
        self.declare_parameter('home_backup_timeout_s', 4.5)
        self.declare_parameter('home_rear_dock_m', 0.13)
        self.declare_parameter('home_dock_confirm_s', 0.35)
        self.declare_parameter('home_post_turn_pause_s', 0.35)
        self.declare_parameter('home_tag_stale_s', 1.80)

        self.declare_parameter('release_gate_after_dock', True)
        self.declare_parameter('release_live_servo', False)
        self.declare_parameter('release_servo_pin', 32)
        self.declare_parameter('release_servo_pins', '32,33')
        self.declare_parameter('release_servo_backend', 'hardware-pwm')
        self.declare_parameter('release_servo_frequency_hz', 50.0)
        self.declare_parameter('release_servo_home_us', 1500)
        self.declare_parameter('release_servo_open_us', 2000)
        self.declare_parameter('release_servo_home_us_list', '')
        self.declare_parameter('release_servo_open_us_list', '')
        self.declare_parameter('release_servo_min_us', 900)
        self.declare_parameter('release_servo_max_us', 2100)
        self.declare_parameter('release_servo_move_s', 0.60)
        self.declare_parameter('release_servo_step_us', 10)
        self.declare_parameter('release_servo_open_hold_s', 7.0)
        self.declare_parameter('release_servo_home_hold_s', 0.80)
        self.declare_parameter('release_servo_return_home', True)

        self.limit = int(self.get_parameter('limit').value)
        self.target_count = int(self.get_parameter('target_count').value)
        self.home_tag_ids = parse_int_set(self.get_parameter('home_tag_ids').value)
        live = bool(self.get_parameter('live_motors').value) and bool(self.get_parameter('ground_test').value)
        self.motor = Stm32MotorLink(
            self,
            str(self.get_parameter('serial_port').value),
            int(self.get_parameter('baud_rate').value),
            live,
            self.limit,
            MOTOR_SIGN,
        )

        self.ball = None
        self.ball_last_s = 0.0
        self.home_target = None
        self.home_last_s = 0.0
        self.obstacles = []
        self.lidar = {'ok': False, 'reason': 'no lidar', 'age_s': 999.0}
        self.imu = {'ok': False, 'yaw_deg': None, 'reason': 'no imu', 'age_s': 999.0}
        self.safety = {'enabled': False, 'paused': False, 'reason': 'no safety state'}
        self.safety_last_s = 0.0
        self.pause_active = False
        self.pause_started_s = 0.0

        self.state = 'SEARCH'
        self.mission_start_s = time.monotonic()
        self.prev_state = 'SEARCH'
        self.collected = 0
        self.collect_start_s = 0.0
        self.step = Step()
        self.search_dir = 1
        self.search_yaw_start = None
        self.search_last_progress = 0.0
        self.search_last_progress_s = 0.0
        self.home_turn_start_yaw = None
        self.home_turn_last_progress = 0.0
        self.home_turn_last_progress_s = 0.0
        self.home_back_start_s = 0.0
        self.home_back_wait_until_s = 0.0
        self.dock_close_since_s = 0.0
        self.release_done = False
        self.release_status = ''
        self.current_action = 'STOP'
        self.current_pwm = [0, 0, 0, 0]
        self.current_reason = 'boot'

        self.create_subscription(String, '/ball_navigate', self.ball_cb, 10)
        self.create_subscription(String, '/apriltag_detections', self.tag_cb, 10)
        self.create_subscription(String, '/obstacle', self.obstacle_cb, 10)
        self.create_subscription(String, '/lidar_sectors', self.lidar_cb, 10)
        self.create_subscription(String, '/imu_state', self.imu_cb, 10)
        self.create_subscription(String, '/safety_state', self.safety_cb, 10)
        self.state_pub = self.create_publisher(String, '/decision_state', 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info(f'Decision node ready, home IDs={sorted(self.home_tag_ids)}')

    def mix_pwm(self, forward, turn_right):
        return [
            clamp(forward + turn_right, self.limit),
            clamp(forward - turn_right, self.limit),
            clamp(forward + turn_right, self.limit),
            clamp(forward - turn_right, self.limit),
        ]

    def turn_pwm(self, direction, pwm):
        sign = 1 if direction == 'right' else -1
        return self.mix_pwm(0, sign * abs(pwm))

    def signed_pwm(self, value, minimum, maximum):
        pwm = clamp(value, maximum)
        if pwm == 0:
            return 0
        if abs(pwm) < minimum:
            return minimum if pwm > 0 else -minimum
        return pwm

    def ball_cb(self, msg):
        try:
            self.ball = json.loads(msg.data)
            self.ball_last_s = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Bad /ball_navigate payload: {exc}')

    def tag_cb(self, msg):
        try:
            data = json.loads(msg.data)
            target = data.get('target')
            if target is None:
                tags = data.get('tags', [])
                matches = [
                    tag for tag in tags
                    if str(tag.get('id', tag.get('tag_id', ''))) in {str(i) for i in self.home_tag_ids}
                ]
                if matches:
                    target = min(matches, key=lambda tag: tag.get('distance_m', tag.get('distance_cm', 9999.0)))
            if target is not None:
                self.home_target = target
                self.home_last_s = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Bad /apriltag_detections payload: {exc}')

    def obstacle_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacles = data.get('obstacles', [])
        except Exception as exc:
            self.get_logger().warn(f'Bad /obstacle payload: {exc}')

    def lidar_cb(self, msg):
        try:
            self.lidar = json.loads(msg.data)
            self.lidar['last_s'] = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Bad /lidar_sectors payload: {exc}')

    def imu_cb(self, msg):
        try:
            self.imu = json.loads(msg.data)
            self.imu['last_s'] = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Bad /imu_state payload: {exc}')

    def safety_cb(self, msg):
        try:
            self.safety = json.loads(msg.data)
            self.safety_last_s = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Bad /safety_state payload: {exc}')

    def ball_visible(self):
        if self.ball is None:
            return False
        if time.monotonic() - self.ball_last_s > float(self.get_parameter('ball_stale_s').value):
            return False
        if not bool(self.ball.get('visible', True)):
            return False
        if self.ball.get('angle') is None:
            return False
        return True

    def ball_detector_alive(self):
        return self.ball_last_s > 0.0 and time.monotonic() - self.ball_last_s <= float(self.get_parameter('ball_stale_s').value)

    def home_visible(self):
        if self.home_target is None:
            return False
        return time.monotonic() - self.home_last_s <= float(self.get_parameter('home_tag_stale_s').value)

    def imu_ok(self):
        age = time.monotonic() - self.imu.get('last_s', 0.0)
        return bool(self.imu.get('ok')) and self.imu.get('yaw_deg') is not None and age <= 1.5

    def yaw(self):
        return float(self.imu['yaw_deg']) if self.imu_ok() else None

    def safety_fresh(self):
        return self.safety_last_s > 0.0 and time.monotonic() - self.safety_last_s <= float(self.get_parameter('safety_stale_s').value)

    def safety_blocks_motion(self):
        if self.safety_fresh() and bool(self.safety.get('paused', False)):
            return True, 'paused by kill switch'
        if self.motor.live and bool(self.get_parameter('require_safety_io').value):
            if not self.safety_fresh():
                return True, 'waiting for safety switch'
            if not bool(self.safety.get('enabled', False)):
                return True, self.safety.get('reason', 'safety switch unavailable')
        return False, 'safety ok'

    def update_pause_clock(self, paused, now):
        if paused and not self.pause_active:
            self.pause_active = True
            self.pause_started_s = now
            return
        if not paused and self.pause_active:
            pause_s = max(0.0, now - self.pause_started_s)
            self.mission_start_s += pause_s
            if self.collect_start_s:
                self.collect_start_s += pause_s
            if self.home_back_start_s:
                self.home_back_start_s += pause_s
            if self.home_back_wait_until_s:
                self.home_back_wait_until_s += pause_s
            if self.dock_close_since_s:
                self.dock_close_since_s += pause_s
            self.pause_active = False
            self.pause_started_s = 0.0

    def lidar_ok(self):
        age = time.monotonic() - self.lidar.get('last_s', 0.0)
        if not self.lidar.get('ok', False):
            return False
        return age <= float(self.get_parameter('lidar_stale_s').value)

    def front_m(self):
        return self.lidar.get('front_m')

    def rear_m(self):
        return self.lidar.get('rear_m')

    def lidar_allows_forward(self):
        if not self.lidar_ok():
            if self.motor.live:
                if bool(self.get_parameter('allow_no_lidar_live').value):
                    return True, self.lidar.get('reason', 'lidar unavailable'), 1.0
                return False, self.lidar.get('reason', 'lidar unavailable'), 0.0
            if not bool(self.get_parameter('allow_no_lidar_in_dry_run').value):
                return False, self.lidar.get('reason', 'lidar unavailable'), 0.0
            return True, self.lidar.get('reason', 'lidar unavailable'), 1.0
        front = self.front_m()
        if front is None:
            return True, 'front clear/no close return', 1.0
        stop = float(self.get_parameter('lidar_stop_m').value)
        slow = float(self.get_parameter('lidar_slow_m').value)
        if front < stop:
            return False, f'front {front:.2f}m < stop {stop:.2f}m', 0.0
        if front < slow:
            scale = max(0.25, (front - stop) / max(0.01, slow - stop))
            return True, f'front caution {front:.2f}m', scale
        return True, 'clear', 1.0

    def rear_allows_reverse(self):
        rear = self.rear_m()
        if rear is None:
            return True, 'rear unknown'
        rear_stop = float(self.get_parameter('rear_stop_m').value)
        return rear >= rear_stop, f'rear {rear:.2f}m'

    def obstacle_blocking(self):
        for obs in self.obstacles:
            if obs.get('distance_cm', 9999.0) < float(self.get_parameter('lidar_stop_m').value) * 100.0:
                if abs(obs.get('angle_deg', 999.0)) < 30.0:
                    return True
        return False

    def closest_obstacle(self):
        if not self.obstacles:
            return None
        return min(self.obstacles, key=lambda item: item.get('distance_cm', 9999.0))

    def start_step(self, action, pwm, duration_s, reason):
        duration_s = max(float(self.get_parameter('step_min_s').value), min(float(self.get_parameter('step_max_s').value), duration_s))
        self.step = Step(action, [clamp(v, self.limit) for v in pwm], time.monotonic() + duration_s, reason)
        self.set_motion(action, self.step.pwm, f'{reason}; step {duration_s:.2f}s')

    def clear_step(self):
        self.step = Step()

    def run_step_if_active(self):
        if self.step.action and time.monotonic() < self.step.until_s:
            remaining = self.step.until_s - time.monotonic()
            self.set_motion(self.step.action, self.step.pwm, f'{self.step.reason}; step remaining {remaining:.2f}s')
            return True
        return False

    def set_motion(self, action, pwm, reason):
        self.current_action = action
        self.current_pwm = [clamp(v, self.limit) for v in pwm]
        self.current_reason = reason
        if action == 'STOP':
            self.motor.stop()
        else:
            self.motor.pwm(self.current_pwm)

    def stop(self, reason):
        self.clear_step()
        self.set_motion('STOP', [0, 0, 0, 0], reason)

    def ball_angle(self):
        return float(self.ball.get('angle', 0.0))

    def ball_distance_cm(self):
        value = self.ball.get('distance_cm')
        return float(value) if value is not None else None

    def ball_reached(self):
        if not self.ball_visible():
            return False, 'no ball'
        angle = abs(self.ball_angle())
        distance = self.ball_distance_cm()
        centered = angle <= float(self.get_parameter('center_tolerance_deg').value)
        if centered and distance is not None and distance <= float(self.get_parameter('collect_distance_cm').value):
            return True, f'distance {distance:.1f}cm'
        if centered and float(self.ball.get('diameter_px', 0.0)) >= float(self.get_parameter('collect_diameter_px').value):
            return True, f'size {self.ball.get("diameter_px")}px'
        if self.ball.get('collect_candidate'):
            return True, 'bottom centered'
        return False, 'not reached'

    def target_angle_distance(self, target):
        angle = target.get('angle_deg', target.get('angle'))
        distance_cm = target.get('distance_cm')
        if distance_cm is None and target.get('distance_m') is not None:
            distance_cm = float(target['distance_m']) * 100.0
        return (
            float(angle) if angle is not None else None,
            float(distance_cm) if distance_cm is not None else None,
        )

    def loop(self):
        now = time.monotonic()
        safety_blocked, safety_reason = self.safety_blocks_motion()
        self.update_pause_clock(safety_blocked, now)
        if safety_blocked:
            self.clear_step()
            self.stop(safety_reason)
            self.publish_state()
            return

        if self.run_step_if_active():
            self.publish_state()
            return

        timeout_s = float(self.get_parameter('mission_timeout_s').value)
        if timeout_s > 0.0 and now - self.mission_start_s >= timeout_s and self.state not in ('RELEASE_GATE', 'FINISHED'):
            self.state = 'FINISHED'
            self.stop(f'mission timeout {timeout_s:.1f}s')
            self.publish_state()
            return

        if (
            self.motor.live
            and bool(self.get_parameter('require_ball_detector_live').value)
            and not self.ball_detector_alive()
            and self.state not in ('RELEASE_GATE', 'FINISHED')
        ):
            self.stop('waiting for live ball detector')
            self.publish_state()
            return

        if self.state not in ('HOME_SEARCH', 'HOME_ALIGN', 'HOME_TURN_180', 'HOME_BACK_TO_WALL', 'RELEASE_GATE', 'FINISHED'):
            if self.collected >= self.target_count:
                self.state = 'HOME_SEARCH'
                self.clear_step()

        if self.obstacle_blocking() and self.state not in ('HOME_BACK_TO_WALL', 'RELEASE_GATE', 'FINISHED'):
            self.prev_state = self.state
            self.state = 'AVOID'
            self.clear_step()

        if self.state == 'SEARCH':
            self.do_search(now)
        elif self.state == 'APPROACH_BALL':
            self.do_approach_ball(now)
        elif self.state == 'COLLECT':
            self.do_collect(now)
        elif self.state == 'AVOID':
            self.do_avoid()
        elif self.state in ('HOME_SEARCH', 'HOME_ALIGN'):
            self.do_home_align_or_search(now)
        elif self.state == 'HOME_TURN_180':
            self.do_home_turn_180(now)
        elif self.state == 'HOME_BACK_TO_WALL':
            self.do_home_back_to_wall(now)
        elif self.state == 'RELEASE_GATE':
            self.do_release_gate()
        else:
            self.stop('finished')
        self.publish_state()

    def do_search(self, now):
        if self.ball_visible():
            self.state = 'APPROACH_BALL'
            return
        if not bool(self.get_parameter('search_when_no_ball').value):
            self.stop('detector alive; no ball visible')
            return

        yaw = self.yaw() if bool(self.get_parameter('search_use_imu').value) else None
        if yaw is None:
            turn = self.search_dir * int(self.get_parameter('search_pwm').value)
            self.search_dir *= -1
            self.start_step('SEARCH_TURN', self.mix_pwm(0, turn), float(self.get_parameter('step_search_s').value), 'no ball; timed sweep')
            return

        if self.search_yaw_start is None:
            self.search_yaw_start = yaw
            self.search_last_progress = 0.0
            self.search_last_progress_s = now
        progress = abs(signed_delta_deg(yaw, self.search_yaw_start))
        if progress - self.search_last_progress >= float(self.get_parameter('search_imu_progress_epsilon_deg').value):
            self.search_last_progress = progress
            self.search_last_progress_s = now
        if progress >= float(self.get_parameter('search_imu_turn_deg').value):
            self.search_dir *= -1
            self.search_yaw_start = yaw
            self.search_last_progress = 0.0
            self.search_last_progress_s = now
            progress = 0.0
        if now - self.search_last_progress_s >= float(self.get_parameter('search_imu_stall_s').value):
            self.search_dir *= -1
            reverse = -int(self.get_parameter('search_imu_recover_reverse_pwm').value)
            turn = self.search_dir * max(int(self.get_parameter('search_pwm').value), int(self.get_parameter('search_imu_recover_turn_pwm').value))
            self.start_step('SEARCH_RECOVERY', self.mix_pwm(reverse, turn), float(self.get_parameter('search_imu_recover_s').value), 'search yaw stalled')
            self.search_yaw_start = yaw
            self.search_last_progress_s = now
            return
        turn = self.search_dir * int(self.get_parameter('search_pwm').value)
        self.start_step(
            'SEARCH_IMU_TURN',
            self.mix_pwm(0, turn),
            float(self.get_parameter('search_imu_step_s').value),
            f'no ball; imu sweep {progress:.0f}/{float(self.get_parameter("search_imu_turn_deg").value):.0f}deg',
        )

    def do_approach_ball(self, now):
        if not self.ball_visible():
            self.state = 'SEARCH'
            return

        reached, reach_reason = self.ball_reached()
        if reached:
            self.state = 'COLLECT'
            self.collect_start_s = now
            self.clear_step()
            self.do_collect(now)
            return

        ok, safety_reason, scale = self.lidar_allows_forward()
        if not ok:
            self.state = 'AVOID'
            self.do_avoid()
            return

        angle = self.ball_angle()
        distance = self.ball_distance_cm()
        abs_angle = abs(angle)
        turn_dir = 1 if angle > 0.0 else -1
        front_cautious = self.front_m() is not None and self.front_m() < float(self.get_parameter('lidar_slow_m').value)
        if abs_angle >= float(self.get_parameter('step_spin_angle_deg').value) or front_cautious:
            turn = self.signed_pwm(angle * 1.0, int(self.get_parameter('min_turn_pwm').value), int(self.get_parameter('turn_pwm').value))
            duration = 0.22 + min(90.0, abs_angle) * 0.008
            self.start_step('STEP_TURN_TO_BALL', self.mix_pwm(0, turn), duration, f'turn to ball {angle:.1f}deg; {safety_reason}')
            return

        if abs_angle > float(self.get_parameter('center_tolerance_deg').value):
            forward = int(self.get_parameter('approach_pwm').value)
            if distance is not None and distance < float(self.get_parameter('slow_distance_cm').value):
                forward = min(forward, int(self.get_parameter('creep_pwm').value))
            forward = clamp(forward * scale, self.limit)
            turn = turn_dir * 8
            side = 'right' if turn_dir > 0 else 'left'
            self.start_step('STEP_ARC_TO_BALL', self.mix_pwm(forward, turn), float(self.get_parameter('step_arc_s').value), f'{side} arc to ball {angle:.1f}deg')
            return

        forward = int(self.get_parameter('approach_pwm').value)
        if distance is not None and distance < float(self.get_parameter('slow_distance_cm').value):
            forward = min(forward, int(self.get_parameter('creep_pwm').value))
        forward = clamp(forward * scale, self.limit)
        self.start_step('STEP_FORWARD', [forward, forward, forward, forward], float(self.get_parameter('step_forward_s').value), 'centered ball')

    def do_collect(self, now):
        elapsed = now - self.collect_start_s
        drive_s = float(self.get_parameter('collect_drive_s').value)
        hold_s = float(self.get_parameter('collect_hold_s').value)
        ok, safety_reason, scale = self.lidar_allows_forward()
        if elapsed < drive_s and ok:
            forward = clamp(int(self.get_parameter('collect_drive_pwm').value) * scale, self.limit)
            self.set_motion('COLLECT_FORWARD', [forward, forward, forward, forward], f'collect drive {elapsed:.2f}/{drive_s:.2f}s')
            return
        if elapsed < drive_s + hold_s:
            self.stop(f'collect settle; {safety_reason}')
            return
        self.collected += 1
        self.collect_start_s = 0.0
        self.clear_step()
        if self.collected >= self.target_count:
            self.state = 'HOME_SEARCH'
            self.stop(f'ball counted {self.collected}/{self.target_count}; returning home')
        else:
            self.state = 'SEARCH'
            self.stop(f'ball counted {self.collected}/{self.target_count}')

    def do_avoid(self):
        if not self.obstacle_blocking() and self.lidar_allows_forward()[0]:
            self.state = self.prev_state if self.prev_state != 'AVOID' else 'SEARCH'
            return
        rear_ok, rear_reason = self.rear_allows_reverse()
        if rear_ok:
            pwm = [-int(self.get_parameter('escape_pwm').value)] * 4
            self.start_step('AVOID_BACKUP', pwm, 0.45, f'front blocked; {rear_reason}')
            return
        obs = self.closest_obstacle()
        turn = int(self.get_parameter('turn_pwm').value)
        if obs and obs.get('side') == 'right':
            turn = -turn
        self.start_step('AVOID_TURN', self.mix_pwm(0, turn), 0.45, f'front blocked; rear {rear_reason}')

    def do_home_align_or_search(self, now):
        if not self.home_visible():
            self.state = 'HOME_SEARCH'
            yaw = self.yaw()
            pwm_value = int(self.get_parameter('home_search_pwm').value)
            if yaw is not None:
                if self.search_yaw_start is None:
                    self.search_yaw_start = yaw
                    self.search_last_progress = 0.0
                    self.search_last_progress_s = now
                progress = abs(signed_delta_deg(yaw, self.search_yaw_start))
                if progress - self.search_last_progress >= 3.0:
                    self.search_last_progress = progress
                    self.search_last_progress_s = now
                if now - self.search_last_progress_s >= 0.9:
                    pwm_value = max(pwm_value, int(self.get_parameter('home_search_boost_pwm').value))
                if progress >= 180.0:
                    self.search_dir *= -1
                    self.search_yaw_start = yaw
            self.start_step('HOME_SEARCH_TAG', self.mix_pwm(0, self.search_dir * pwm_value), 0.45, 'searching for home tag 20/21')
            return

        self.state = 'HOME_ALIGN'
        self.search_yaw_start = None
        angle, distance_cm = self.target_angle_distance(self.home_target)
        if angle is None:
            self.stop('home tag visible but angle unavailable')
            return
        tolerance = float(self.get_parameter('home_align_tolerance_deg').value)
        if abs(angle) > tolerance:
            turn = self.signed_pwm(
                angle * float(self.get_parameter('home_align_gain').value),
                int(self.get_parameter('min_turn_pwm').value),
                int(self.get_parameter('home_turn_180_pwm').value),
            )
            self.start_step('HOME_ALIGN_TAG', self.mix_pwm(0, turn), 0.25, f'home tag angle {angle:+.1f}deg')
            return
        if distance_cm is None:
            self.stop('home tag aligned but distance unavailable')
            return

        target = float(self.get_parameter('home_approach_cm').value)
        tol = float(self.get_parameter('home_distance_tolerance_cm').value)
        if distance_cm > target + tol:
            ok, safety_reason, scale = self.lidar_allows_forward()
            if not ok:
                self.stop(f'home approach blocked: {safety_reason}')
                return
            forward = int(self.get_parameter('home_approach_pwm').value)
            if distance_cm < 28.0:
                forward = min(forward, int(self.get_parameter('home_creep_pwm').value))
            forward = clamp(forward * scale, self.limit)
            turn = clamp(angle * 0.25, 5)
            self.start_step('HOME_APPROACH_TAG', self.mix_pwm(forward, turn), 0.35, f'home tag {distance_cm:.1f}cm')
            return
        if distance_cm < target - tol:
            rear_ok, rear_reason = self.rear_allows_reverse()
            if not rear_ok:
                self.stop(f'home too close but rear blocked: {rear_reason}')
                return
            back = -int(self.get_parameter('home_creep_pwm').value)
            self.start_step('HOME_BACK_FROM_TAG', [back, back, back, back], 0.25, f'home too close {distance_cm:.1f}cm')
            return

        yaw = self.yaw()
        if yaw is None:
            self.stop('ready for 180 but IMU yaw unavailable')
            return
        self.state = 'HOME_TURN_180'
        self.home_turn_start_yaw = yaw
        self.home_turn_last_progress = 0.0
        self.home_turn_last_progress_s = now
        self.stop(f'aligned at {distance_cm:.1f}cm; starting 180')

    def do_home_turn_180(self, now):
        yaw = self.yaw()
        if yaw is None:
            self.stop('turn 180 waiting for IMU')
            return
        if self.home_turn_start_yaw is None:
            self.home_turn_start_yaw = yaw
            self.home_turn_last_progress_s = now
        progress = abs(signed_delta_deg(yaw, self.home_turn_start_yaw))
        if progress - self.home_turn_last_progress >= 2.0:
            self.home_turn_last_progress = progress
            self.home_turn_last_progress_s = now
        target = float(self.get_parameter('home_turn_180_deg').value)
        tol = float(self.get_parameter('home_turn_tolerance_deg').value)
        if progress >= target - tol:
            self.state = 'HOME_BACK_TO_WALL'
            self.home_back_start_s = now
            self.home_back_wait_until_s = now + float(self.get_parameter('home_post_turn_pause_s').value)
            self.dock_close_since_s = 0.0
            self.stop(f'180 done {progress:.1f}deg; preparing reverse dock')
            return
        pwm = int(self.get_parameter('home_turn_180_pwm').value)
        if now - self.home_turn_last_progress_s >= float(self.get_parameter('home_turn_stall_s').value):
            pwm = max(pwm, int(self.get_parameter('home_turn_boost_pwm').value))
        direction = str(self.get_parameter('home_turn_direction').value)
        self.start_step('HOME_TURN_180', self.turn_pwm(direction, pwm), 0.25, f'turning {progress:.1f}/{target:.0f}deg')

    def do_home_back_to_wall(self, now):
        if now < self.home_back_wait_until_s:
            self.stop('post-turn settle before reverse dock')
            return
        elapsed = now - self.home_back_wait_until_s
        force_s = float(self.get_parameter('home_backup_force_s').value)
        if elapsed < force_s:
            pwm = -max(int(self.get_parameter('home_backup_pwm').value), int(self.get_parameter('home_backup_force_pwm').value))
            self.set_motion('HOME_BACK_TO_WALL_FORCE', [pwm, pwm, pwm, pwm], f'forcing rear dock {elapsed:.1f}/{force_s:.1f}s')
            return

        rear = self.rear_m()
        rear_dock = float(self.get_parameter('home_rear_dock_m').value)
        if rear is None or rear <= rear_dock:
            if self.dock_close_since_s <= 0.0:
                self.dock_close_since_s = now
                self.stop(f'rear dock settling rear={rear}')
                return
            if now - self.dock_close_since_s >= float(self.get_parameter('home_dock_confirm_s').value):
                self.state = 'RELEASE_GATE'
                self.stop(f'docked rear={rear}')
                return
            self.stop(f'confirming dock rear={rear}')
            return

        if now - self.home_back_wait_until_s > float(self.get_parameter('home_backup_timeout_s').value):
            self.state = 'RELEASE_GATE'
            self.stop(f'reverse dock timeout; rear={rear}')
            return

        rear_ok, rear_reason = self.rear_allows_reverse()
        if not rear_ok:
            self.state = 'RELEASE_GATE'
            self.stop(f'rear close/blocked; treating docked: {rear_reason}')
            return
        pwm = -int(self.get_parameter('home_backup_pwm').value)
        self.set_motion('HOME_BACK_TO_WALL', [pwm, pwm, pwm, pwm], f'reversing to wall; {rear_reason}')

    def bounded_servo_pulse(self, value):
        min_us = int(self.get_parameter('release_servo_min_us').value)
        max_us = int(self.get_parameter('release_servo_max_us').value)
        if value < min_us or value > max_us:
            raise RuntimeError(f'servo pulse {value} outside {min_us}..{max_us}us')
        return int(value)

    def move_servo(self, gate, start_us, end_us):
        move_s = float(self.get_parameter('release_servo_move_s').value)
        step_us = int(self.get_parameter('release_servo_step_us').value)
        values = pulse_steps(start_us, end_us, step_us)
        delay = max(0.0, move_s / max(1, len(values) - 1))
        for value in values:
            gate.write_us(value)
            time.sleep(delay)

    def move_servos(self, gates, start_values, end_values):
        move_s = float(self.get_parameter('release_servo_move_s').value)
        step_us = max(1, abs(int(self.get_parameter('release_servo_step_us').value)))
        largest_delta = max(abs(end - start) for start, end in zip(start_values, end_values))
        count = max(1, int(math.ceil(largest_delta / step_us)))
        delay = max(0.0, move_s / count)
        for index in range(count + 1):
            ratio = index / count
            for gate, start_us, end_us in zip(gates, start_values, end_values):
                pulse = int(round(start_us + (end_us - start_us) * ratio))
                gate.write_us(pulse)
            time.sleep(delay)

    def release_servo_config(self):
        pins = parse_int_list(self.get_parameter('release_servo_pins').value)
        if not pins:
            pins = [int(self.get_parameter('release_servo_pin').value)]
        home_default = self.bounded_servo_pulse(int(self.get_parameter('release_servo_home_us').value))
        open_default = self.bounded_servo_pulse(int(self.get_parameter('release_servo_open_us').value))
        home_values = parse_int_list(self.get_parameter('release_servo_home_us_list').value)
        open_values = parse_int_list(self.get_parameter('release_servo_open_us_list').value)
        if home_values and len(home_values) != len(pins):
            raise RuntimeError('release_servo_home_us_list must match release_servo_pins length')
        if open_values and len(open_values) != len(pins):
            raise RuntimeError('release_servo_open_us_list must match release_servo_pins length')
        if not home_values:
            home_values = [home_default] * len(pins)
        if not open_values:
            open_values = [open_default] * len(pins)
        home_values = [self.bounded_servo_pulse(value) for value in home_values]
        open_values = [self.bounded_servo_pulse(value) for value in open_values]
        return pins, home_values, open_values

    def do_release_gate(self):
        self.stop('release gate phase')
        if self.release_done:
            self.state = 'FINISHED'
            return
        if not bool(self.get_parameter('release_gate_after_dock').value):
            self.release_status = 'disabled'
            self.release_done = True
            self.state = 'FINISHED'
            return
        if not bool(self.get_parameter('release_live_servo').value):
            self.release_status = 'dry-run; set release_live_servo true to move servo'
            self.release_done = True
            self.state = 'FINISHED'
            return

        pins, home_values, open_values = self.release_servo_config()
        gates = [
            ServoGate(
                pin,
                str(self.get_parameter('release_servo_backend').value),
                float(self.get_parameter('release_servo_frequency_hz').value),
            )
            for pin in pins
        ]
        try:
            self.release_status = f'opening pins {pins}'
            for gate, home_us in zip(gates, home_values):
                gate.open()
                gate.write_us(home_us)
            time.sleep(float(self.get_parameter('release_servo_home_hold_s').value))
            self.move_servos(gates, home_values, open_values)
            time.sleep(float(self.get_parameter('release_servo_open_hold_s').value))
            if bool(self.get_parameter('release_servo_return_home').value):
                self.release_status = 'closing'
                self.move_servos(gates, open_values, home_values)
                time.sleep(float(self.get_parameter('release_servo_home_hold_s').value))
            self.release_status = 'done'
        except Exception as exc:
            self.release_status = f'servo error: {exc}'
        finally:
            for gate, home_us in zip(gates, home_values):
                try:
                    gate.close(home_us, float(self.get_parameter('release_servo_home_hold_s').value), True)
                except Exception as exc:
                    self.get_logger().warn(f'Servo cleanup failed on pin {gate.pin}: {exc}')
            self.release_done = True
            self.state = 'FINISHED'

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'action': self.current_action,
            'reason': self.current_reason,
            'pwm': self.current_pwm,
            'live_motors': self.motor.live,
            'collected': self.collected,
            'target_count': self.target_count,
            'ball_visible': self.ball_visible(),
            'ball': self.ball if self.ball_visible() else None,
            'home_visible': self.home_visible(),
            'home_target': self.home_target if self.home_visible() else None,
            'lidar': self.lidar,
            'imu': self.imu,
            'safety': self.safety,
            'paused': bool(self.safety.get('paused', False)) if self.safety_fresh() else False,
            'release_done': self.release_done,
            'release_status': self.release_status,
            'runtime_s': round(time.monotonic() - self.mission_start_s, 2),
            'timestamp': time.time(),
        })
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
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
