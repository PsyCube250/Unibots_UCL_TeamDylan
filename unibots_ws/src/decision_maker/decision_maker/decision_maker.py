import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import serial
import time
import random

SERIAL_PORT = '/dev/ttyTHS0'
BAUD_RATE = 115200

FORWARD_SPEED = 180
SLOW_SPEED = 100
OBSTACLE_STOP_CM = 20.0
OBSTACLE_AVOID_CM = 30.0
BALL_COLLECT_CM = 15.0
BALL_CENTRE_TOLERANCE = 8.0
HOME_CENTRE_TOLERANCE = 10.0
SEARCH_TURN_DEG = 25
COLLECT_HOLD_SEC = 2.0
HOME_DISTANCE_STOP_CM = 25.0

MATCH_DURATION_SEC = 150.0 
RANDOM_ANGLE_OPTIONS = [-135, -90, 90, 135]
RANDOM_TURN_HOLD_SEC = 3.0

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_maker')

        self.create_subscription(String, '/ball_navigate', self.ball_cb, 10)
        self.create_subscription(String, '/apriltag_detections', self.tag_cb, 10)
        self.create_subscription(String, '/obstacle', self.obstacle_cb, 10)

        self.ball_angle = None
        self.ball_dist_cm = None
        self.ball_count = 0
        self.last_seen = time.time()

        self.home_angle = None
        self.home_dist_cm = None
        self.facing_wall = None
        self.home_wait = None

        self.obstacles = []
        self.obstacle_count = 0

        self.STATE_SEARCH = 'SEARCH'
        self.STATE_APPROACH_BALL = 'APPROACH_BALL'
        self.STATE_COLLECT = 'COLLECT'
        self.STATE_RANDOM_TURN = 'RANDOM_TURN'
        self.STATE_RETURN_HOME = 'RETURN_HOME'
        self.STATE_AVOID = 'AVOID'
        self.STATE_FINISHED = 'FINISHED'

        self.state = self.STATE_SEARCH
        self.prev_state = self.STATE_SEARCH
        self.collect_start = None
        self.search_dir = 1
        self.search_steps = 0

        self.match_start_time = time.time()
        self.match_finished = False
        self.random_turn_start = None

        self.declare_parameter('own_zone', 'North')
        self.own_zone = self.get_parameter('own_zone').get_parameter_value().string_value
        self.get_logger().info(f'Own zone: {self.own_zone}')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Serial open: {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'Serial failed: {e} - no serial')
            self.ser = None

        self.create_timer(0.1, self.loop)

    def ball_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.ball_angle = data.get('angle', None)
            self.ball_dist_cm = data.get('distance_cm', None)
            self.ball_count = int(data.get('count', 0))
            self.last_seen = time.time()
        except Exception:
            pass

    def tag_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.facing_wall = data.get('facing')
            for tag in data.get('tags', []):
                if tag.get('wall') == self.own_zone:
                    self.home_angle = tag['angle']
                    self.home_dist_cm = tag['distance_cm']
                    return
            self.home_angle = None
            self.home_dist_cm = None
        except Exception:
            pass

    def obstacle_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacles = data.get('obstacles', [])
            self.obstacle_count = data.get('obstacle_count', 0)
        except Exception:
            pass

    def send(self, cmd: str):
        self.get_logger().info(f'CMD: {cmd}')
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode())
            except Exception as e:
                pass

    def closest_obstacle(self):
        if not self.obstacles: return None
        return min(self.obstacles, key=lambda x: x['distance_cm'])

    def obstacle_blocking(self):
        for obs in self.obstacles:
            if obs['distance_cm'] < OBSTACLE_STOP_CM and abs(obs['angle_deg']) < 30:
                return True
        return False

    def obstacle_warning(self):
        for obs in self.obstacles:
            if obs['distance_cm'] < OBSTACLE_AVOID_CM:
                return True
        return False

    def avoid_direction(self):
        obs = self.closest_obstacle()
        if obs is None: return SEARCH_TURN_DEG
        if obs['side'] == 'left': return SEARCH_TURN_DEG
        elif obs['side'] == 'right': return -SEARCH_TURN_DEG
        else: return SEARCH_TURN_DEG

    def loop(self):
        # watchdog timer to kill ghost data
        if self.ball_angle is not None and (time.time() - self.last_seen > 0.5):
            self.ball_angle = None
            self.ball_dist_cm = None
            self.ball_count = 0

        elapsed_match = time.time() - self.match_start_time
        if not self.match_finished and elapsed_match >= MATCH_DURATION_SEC:
            self.get_logger().warn('MISSION TIMER DONE!')
            self.match_finished = True

        if self.obstacle_blocking() and self.state not in [self.STATE_AVOID, self.STATE_FINISHED]:
            self.prev_state = self.state
            self.state = self.STATE_AVOID

        if self.state == self.STATE_FINISHED:
            self.send('STOP')
            return

        if self.match_finished and self.state in [self.STATE_SEARCH, self.STATE_APPROACH_BALL]:
            if self.state != self.STATE_RETURN_HOME:
                 self.state = self.STATE_RETURN_HOME

        if self.state == self.STATE_SEARCH: self.do_search()
        elif self.state == self.STATE_APPROACH_BALL: self.do_approach_ball()
        elif self.state == self.STATE_COLLECT: self.do_collect()
        elif self.state == self.STATE_RANDOM_TURN: self.do_random_turn()
        elif self.state == self.STATE_RETURN_HOME: self.do_return_home()
        elif self.state == self.STATE_AVOID: self.do_avoid()

    def do_search(self):
        if self.ball_angle is not None and self.ball_dist_cm is not None:
            self.state = self.STATE_APPROACH_BALL
            return

        self.search_steps += 1
        turn = SEARCH_TURN_DEG * self.search_dir
        self.send(f'TURN,{turn}')

        if self.search_steps >= 6:
            self.search_dir *= -1
            self.search_steps = 0

    def do_approach_ball(self):
        if self.ball_angle is None:
            self.state = self.STATE_SEARCH
            return

        if self.ball_dist_cm is not None and self.ball_dist_cm < BALL_COLLECT_CM:
            self.state = self.STATE_COLLECT
            self.collect_start = time.time()
            self.send('COLLECT')
            return

        speed = SLOW_SPEED if self.obstacle_warning() else FORWARD_SPEED

        if abs(self.ball_angle) > BALL_CENTRE_TOLERANCE:
            self.send(f'TURN,{self.ball_angle:.0f}')
        else:
            self.send(f'FORWARD,{speed}')

    def do_collect(self):
        if self.collect_start is None:
            self.collect_start = time.time()
            self.send('COLLECT')

        elapsed = time.time() - self.collect_start
        if elapsed >= COLLECT_HOLD_SEC:
            self.state = self.STATE_RANDOM_TURN
            self.collect_start = None
            self.random_turn_start = None

    def do_random_turn(self):
        if self.random_turn_start is None:
            self.random_turn_start = time.time()
            angle = random.choice(RANDOM_ANGLE_OPTIONS)
            self.send(f'TURN,{angle}')

        elapsed = time.time() - self.random_turn_start
        if elapsed >= RANDOM_TURN_HOLD_SEC:
            self.random_turn_start = None
            self.send('STOP')

            if self.match_finished:
                self.state = self.STATE_RETURN_HOME
            else:
                self.state = self.STATE_SEARCH

    def do_return_home(self):
        if self.home_angle is not None and self.home_dist_cm is not None:

            if self.home_dist_cm < HOME_DISTANCE_STOP_CM:
                if self.home_wait is None:
                    self.send('STOP')
                    self.home_wait = time.time()
                    return
                
                # non-blocking wait to replace time.sleep(0.5)
                if time.time() - self.home_wait < 0.5:
                    return
                
                if self.match_finished:
                    self.send('TURN,180')
                    self.state = self.STATE_FINISHED
                else:
                    self.state = self.STATE_SEARCH
                
                self.home_wait = None
                return

            if abs(self.home_angle) > HOME_CENTRE_TOLERANCE:
                self.send(f'TURN,{self.home_angle:.0f}')
            else:
                speed = SLOW_SPEED if self.obstacle_warning() else FORWARD_SPEED
                self.send(f'FORWARD,{speed}')
        else:
            self.send(f'TURN,{SEARCH_TURN_DEG}')

    def do_avoid(self):
        if not self.obstacle_blocking():
            self.state = self.prev_state
            return

        turn = self.avoid_direction()
        self.send(f'TURN,{turn}')


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send('STOP')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
