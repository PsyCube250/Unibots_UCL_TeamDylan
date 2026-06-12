import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import serial
import time
import random  # Required for random re-orientation

# Serial port to STM32
SERIAL_PORT = '/dev/ttyTHS0'   # Jetson Orin Nano UART TX/RX pins
BAUD_RATE = 115200

# Tuning constants
FORWARD_SPEED = 180           # 0-255
SLOW_SPEED = 100
OBSTACLE_STOP_CM = 20.0       # Stop if obstacle closer than this
OBSTACLE_AVOID_CM = 30.0      # Start avoiding if closer than this
BALL_COLLECT_CM = 15.0        # Close enough to collect
BALL_CENTRE_TOLERANCE = 8.0   # Degrees — within this = centred on ball
HOME_CENTRE_TOLERANCE = 10.0  # Degrees — within this = facing home
SEARCH_TURN_DEG = 25          # Degrees to turn each search step
COLLECT_HOLD_SEC = 2.0        # Duration to drive straight over the ball
HOME_DISTANCE_STOP_CM = 25.0  # Stop this close to home wall

# NEW: Mission Constants
# Match Duration: 2.5 minutes = 150 seconds
MATCH_DURATION_SEC = 150.0 
# Random re-orientation parameters
RANDOM_ANGLE_OPTIONS = [-135, -90, 90, 135] # Large angles to face new areas
RANDOM_TURN_HOLD_SEC = 3.0 # Approximate duration to allow turn to execute

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_maker')

        # Subscriptions (Original)
        self.create_subscription(String, '/ball_navigate',   self.ball_cb,     10)
        self.create_subscription(String, '/apriltag_detections', self.tag_cb,  10)
        self.create_subscription(String, '/obstacle',        self.obstacle_cb, 10)

        # Latest data from each sensor node (Original)
        self.ball_angle    = None   # degrees, None = no ball visible
        self.ball_dist_cm  = None
        self.ball_count    = 0

        self.home_angle    = None   # degrees to home wall tag, None = not visible
        self.home_dist_cm  = None
        self.facing_wall   = None

        self.obstacles     = []     # list of {angle_deg, distance_cm, side}
        self.obstacle_count = 0

        # Revised State machine
        self.STATE_SEARCH        = 'SEARCH'
        self.STATE_APPROACH_BALL = 'APPROACH_BALL'
        self.STATE_COLLECT       = 'COLLECT'       # Drives straight over ball
        self.STATE_RANDOM_TURN   = 'RANDOM_TURN'   # NEW: Re-orientation after collect
        self.STATE_RETURN_HOME   = 'RETURN_HOME'
        self.STATE_AVOID         = 'AVOID'
        self.STATE_FINISHED      = 'FINISHED'      # NEW: End of mission, executing final 180

        self.state          = self.STATE_SEARCH
        self.prev_state     = self.STATE_SEARCH
        self.collect_start  = None
        self.search_dir     = 1    # 1 = right, -1 = left
        self.search_steps   = 0

        # NEW: Timing variables
        self.match_start_time = time.time()
        self.match_finished = False # Boolean flag for timer completion
        self.random_turn_start = None

        # Own zone set via ROS parameter (Original)
        self.declare_parameter('own_zone', 'North')
        self.own_zone = self.get_parameter('own_zone').get_parameter_value().string_value
        self.get_logger().info(f'Own zone: {self.own_zone}')

        # Serial to STM32 (Original)
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Serial open: {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'Serial failed: {e} — running without serial')
            self.ser = None

        # Main loop at 10Hz
        self.create_timer(0.1, self.loop)
        self.get_logger().info('Decision maker (Continuous Collection Mode) ready!')

    # ------------------------------------------------------------------ #
    #  Callbacks (Original)                                              #
    # ------------------------------------------------------------------ #

    def ball_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.ball_angle   = data.get('angle', None)
            self.ball_dist_cm = data.get('distance_cm', None)
            self.ball_count   = int(data.get('count', 0))
        except Exception:
            self.ball_angle   = None
            self.ball_dist_cm = None
            self.ball_count   = 0

    def tag_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.facing_wall = data.get('facing')
            for tag in data.get('tags', []):
                if tag.get('wall') == self.own_zone:
                    self.home_angle   = tag['angle']
                    self.home_dist_cm = tag['distance_cm']
                    return
            # Own zone tag not visible this frame
            self.home_angle   = None
            self.home_dist_cm = None
        except Exception:
            pass

    def obstacle_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacles      = data.get('obstacles', [])
            self.obstacle_count = data.get('obstacle_count', 0)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    #  Serial helpers (Original)                                         #
    # ------------------------------------------------------------------ #

    def send(self, cmd: str):
        self.get_logger().info(f'CMD: {cmd}')
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode())
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    # ------------------------------------------------------------------ #
    #  Obstacle helpers (Original)                                       #
    # ------------------------------------------------------------------ #

    def closest_obstacle(self):
        if not self.obstacles:
            return None
        return min(self.obstacles, key=lambda x: x['distance_cm'])

    def obstacle_blocking(self):
        """Returns True if any obstacle is within stop threshold dead-ahead."""
        for obs in self.obstacles:
            if obs['distance_cm'] < OBSTACLE_STOP_CM and abs(obs['angle_deg']) < 30:
                return True
        return False

    def obstacle_warning(self):
        """Returns True if an obstacle is within avoid threshold."""
        for obs in self.obstacles:
            if obs['distance_cm'] < OBSTACLE_AVOID_CM:
                return True
        return False

    def avoid_direction(self):
        """Returns turn angle to dodge closest obstacle."""
        obs = self.closest_obstacle()
        if obs is None:
            return SEARCH_TURN_DEG
        # Turn away from obstacle side
        if obs['side'] == 'left':
            return SEARCH_TURN_DEG      # turn right
        elif obs['side'] == 'right':
            return -SEARCH_TURN_DEG     # turn left
        else:
            return SEARCH_TURN_DEG      # center — default right

    # ------------------------------------------------------------------ #
    #  State machine loop                                                #
    # ------------------------------------------------------------------ #

    def loop(self):
        # NEW: 1. Check Mission Timer
        elapsed_match = time.time() - self.match_start_time
        if not self.match_finished and elapsed_match >= MATCH_DURATION_SEC:
            self.get_logger().warn('MISSION TIMER DONE! Completing sequence and heading home.')
            self.match_finished = True

        # ---- AVOID overrides everything (Original) ----
        # Safety constraint: Do not override FINISHED state.
        if self.obstacle_blocking() and self.state not in [self.STATE_AVOID, self.STATE_FINISHED]:
            self.get_logger().warn('Obstacle! Switching to AVOID')
            self.prev_state = self.state
            self.state = self.STATE_AVOID

        # NEW: 2. End of Mission Behavior
        # Stop everything if we are completely done.
        if self.state == self.STATE_FINISHED:
            self.send('STOP')
            return

        # If mission is complete, force Return Home in safe states.
        # This prevents starting a new collection or search cycle.
        if self.match_finished and self.state in [self.STATE_SEARCH, self.STATE_APPROACH_BALL]:
            if self.state != self.STATE_RETURN_HOME:
                 self.get_logger().info('Mission over (timer) — Switches to RETURN_HOME.')
                 self.state = self.STATE_RETURN_HOME

        # ---- State handlers ----
        if self.state == self.STATE_SEARCH:
            self.do_search()

        elif self.state == self.STATE_APPROACH_BALL:
            self.do_approach_ball()

        elif self.state == self.STATE_COLLECT:
            self.do_collect()

        elif self.state == self.STATE_RANDOM_TURN:
            self.do_random_turn()

        elif self.state == self.STATE_RETURN_HOME:
            self.do_return_home()

        elif self.state == self.STATE_AVOID:
            self.do_avoid()

    # ---- SEARCH (Revised) ----
    def do_search(self):
        if self.ball_angle is not None and self.ball_dist_cm is not None:
            self.get_logger().info(f'Ball found! Angle:{self.ball_angle:.1f} Dist:{self.ball_dist_cm:.1f}cm')
            self.state = self.STATE_APPROACH_BALL
            return

        # No ball — rotate to scan (Original)
        self.search_steps += 1
        turn = SEARCH_TURN_DEG * self.search_dir
        self.send(f'TURN,{turn}')

        if self.search_steps >= 6:
            self.search_dir *= -1
            self.search_steps = 0

    # ---- APPROACH BALL (Revised) ----
    def do_approach_ball(self):
        if self.ball_angle is None:
            # Lost the ball — go back to search
            self.get_logger().info('Ball lost — returning to SEARCH')
            self.state = self.STATE_SEARCH
            return

        # Close enough to collect — Revised transition
        if self.ball_dist_cm is not None and self.ball_dist_cm < BALL_COLLECT_CM:
            self.get_logger().info('Ball in range — Starts straight drive over ball.')
            self.state = self.STATE_COLLECT
            self.collect_start = time.time()
            self.send('COLLECT') # Signal drive over ball sequence
            return

        # Obstacle warning — slow down (Original)
        speed = SLOW_SPEED if self.obstacle_warning() else FORWARD_SPEED

        # Steer toward ball
        if abs(self.ball_angle) > BALL_CENTRE_TOLERANCE:
            self.send(f'TURN,{self.ball_angle:.0f}')
        else:
            self.send(f'FORWARD,{speed}')

    # ---- COLLECT (REVISED FOR CONTINUOUS MODE) ----
    def do_collect(self):
        # Initialize sequence timer (Original)
        if self.collect_start is None:
            self.collect_start = time.time()
            self.send('COLLECT') # Drive straight over ball

        elapsed = time.time() - self.collect_start
        # Modified Behavior: Clearing the fixed hold, transition to turn state
        if elapsed >= COLLECT_HOLD_SEC:
            # USER: "just drive over them and then turn in any random way until another ball found"
            self.get_logger().info('Driven over ball sequence complete. Re-orienting.')
            self.state = self.STATE_RANDOM_TURN # Transition to new state for turning
            self.collect_start = None
            self.random_turn_start = None # Ready for the next state timer

    # ---- RANDOM TURN (NEW STATE) ----
    def do_random_turn(self):
        """User: 'turn in any random way until another ball found'
        Initiates a single, large, random turn, waits, then returns to SEARCH or Home."""
        if self.random_turn_start is None:
            self.random_turn_start = time.time()
            # Select a random large angle to face a new area
            angle = random.choice(RANDOM_ANGLE_OPTIONS)
            self.get_logger().info(f'Executing random re-orientation turn: {angle} degrees')
            self.send(f'TURN,{angle}')

        elapsed = time.time() - self.random_turn_start
        # Allow turn command time to physically execute before continuing logic
        if elapsed >= RANDOM_TURN_HOLD_SEC:
            self.random_turn_start = None
            self.send('STOP') # Pause briefly before searching

            # If mission is done, transition home; else return to search
            if self.match_finished:
                self.get_logger().info('Re-orientation complete. Heading Home (mission timer done).')
                self.state = self.STATE_RETURN_HOME
            else:
                self.get_logger().info('Re-orientation complete. Resuming SEARCH.')
                self.state = self.STATE_SEARCH

    # ---- RETURN HOME (REVISED FOR FINAL 180°) ----
    def do_return_home(self):
        # If home tag is visible, navigate toward it (Original Logic)
        if self.home_angle is not None and self.home_dist_cm is not None:

            # Reached Target — MODIFIED Behavior
            if self.home_dist_cm < HOME_DISTANCE_STOP_CM:
                self.get_logger().info('At Home Zone. Issuing STOP.')
                self.send('STOP') # Stop forward motion immediately
                
                # pauses slightly before starting new complex turn for robustness
                time.sleep(0.5)

                if self.match_finished:
                    # USER: "after going back it turn 180 so the back of the robot is against the wall"
                    self.get_logger().info('Match complete! Executing final 180° turn.')
                    self.send('TURN,180') # Perform the requested 180° re-orientation
                    self.state = self.STATE_FINISHED # Final non-processing state
                else:
                    # fallback behavior: timer should break this loop, but robust if called otherwise
                    self.get_logger().info('At Home Zone (Match NOT Done) — Deposits, back to SEARCH')
                    self.state = self.STATE_SEARCH
                return

            # Face home tag (Original steer)
            if abs(self.home_angle) > HOME_CENTRE_TOLERANCE:
                self.send(f'TURN,{self.home_angle:.0f}')
            else:
                # Obstacle Warning (Original speed constraint)
                speed = SLOW_SPEED if self.obstacle_warning() else FORWARD_SPEED
                self.send(f'FORWARD,{speed}')
        else:
            # Home tag not visible — rotate to find it (Original scan)
            self.send(f'TURN,{SEARCH_TURN_DEG}')

    # ---- AVOID (Original) ----
    def do_avoid(self):
        if not self.obstacle_blocking():
            self.get_logger().info(f'Obstacle cleared — back to {self.prev_state}')
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
        node.send('STOP') # Ensure robot stops physically
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
