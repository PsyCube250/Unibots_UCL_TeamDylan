#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="${UNIBOTS_LOG_DIR:-$REPO_DIR/logs}"
mkdir -p "$LOG_DIR"

if [ "${UNIBOTS_ROS_BOOT_LOG_REDIRECTED:-0}" != "1" ]; then
  export UNIBOTS_ROS_BOOT_LOG_REDIRECTED=1
  exec > >(tee -a "$LOG_DIR/unibots_ros_main_boot.log") 2>&1
fi

cd "$REPO_DIR"

echo "==== unibots ROS main boot $(date -Iseconds) ===="
echo "repo=$REPO_DIR"

READY_FILE="${UNIBOTS_READY_FILE:-/tmp/unibots_ready_for_button}"
rm -f "$READY_FILE"
BUTTON_BACKEND="${UNIBOTS_BUTTON_BACKEND:-gpio}"

LED_ARGS=(
  --red-pin "${UNIBOTS_RED_LED_PIN:-29}"
  --green-pin "${UNIBOTS_GREEN_LED_PIN:-31}"
)
if [ "${UNIBOTS_LED_ACTIVE_LOW:-0}" = "1" ]; then
  LED_ARGS+=(--led-active-low)
fi

if [ "${UNIBOTS_ENABLE_GPIO_SAFETY:-1}" = "1" ] && [ "$BUTTON_BACKEND" = "gpio" ]; then
  python3 scripts/unibots_gpio_led.py red "${LED_ARGS[@]}" || true
fi

set +u
# shellcheck disable=SC1091
source scripts/jetson_ros_yolo_env.sh
set -u

PYTHON_BIN="${UNIBOTS_PYTHON:-$REPO_DIR/unibots_ws/venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN="python3"
fi

wait_path() {
  local label="$1"
  local path="$2"
  local timeout_s="${3:-45}"
  local start_s
  start_s="$(date +%s)"
  while [ ! -e "$path" ]; do
    if [ $(( "$(date +%s)" - start_s )) -ge "$timeout_s" ]; then
      echo "missing $label after ${timeout_s}s: $path"
      return 1
    fi
    sleep 0.5
  done
  echo "$label=$path"
}

bool_value() {
  case "${1:-0}" in
    1|true|TRUE|yes|YES|on|ON)
      echo true
      ;;
    *)
      echo false
      ;;
  esac
}

CAMERA="${UNIBOTS_CAMERA:-/dev/video0}"
STM32_PORT="${UNIBOTS_STM32_PORT:-/dev/ttyTHS1}"
LIDAR_PORT="${UNIBOTS_LIDAR_PORT:-/dev/ttyUSB0}"
DEVICE_WAIT_S="${UNIBOTS_DEVICE_WAIT_S:-120}"

wait_path "camera" "$CAMERA" "$DEVICE_WAIT_S"
wait_path "stm32" "$STM32_PORT" "$DEVICE_WAIT_S"
if [ "${UNIBOTS_START_RAW_LIDAR:-1}" = "1" ]; then
  wait_path "lidar" "$LIDAR_PORT" "$DEVICE_WAIT_S"
fi

MODEL_PATH="${UNIBOTS_YOLO_MODEL:-}"
if [ -z "$MODEL_PATH" ]; then
  if [ -f "$REPO_DIR/unibots_ws/src/ball_detector/ball_detector/yolo26n.engine" ]; then
    MODEL_PATH="$REPO_DIR/unibots_ws/src/ball_detector/ball_detector/yolo26n.engine"
  elif [ -f "$REPO_DIR/yolo26n.pt" ]; then
    MODEL_PATH="$REPO_DIR/yolo26n.pt"
  else
    MODEL_PATH="auto"
  fi
fi

pids=()
names=()

ENABLE_GPIO="$(bool_value "${UNIBOTS_ENABLE_GPIO_SAFETY:-1}")"
REQUIRE_GPIO="$(bool_value "${UNIBOTS_REQUIRE_GPIO_SAFETY:-1}")"
BUTTON_ACTIVE_LOW="$(bool_value "${UNIBOTS_KILL_BUTTON_ACTIVE_LOW:-1}")"
LED_ACTIVE_LOW="$(bool_value "${UNIBOTS_LED_ACTIVE_LOW:-0}")"
START_PAUSED="$(bool_value "${UNIBOTS_START_PAUSED:-0}")"
CAMERA_ROTATE_180="$(bool_value "${UNIBOTS_CAMERA_ROTATE_180:-1}")"
YOLO_HALF="$(bool_value "${UNIBOTS_YOLO_HALF:-1}")"
REQUIRE_BALL_COLOR="$(bool_value "${UNIBOTS_REQUIRE_BALL_COLOR:-1}")"
ROS_LIVE_MOTORS="$(bool_value "${UNIBOTS_ROS_LIVE_MOTORS:-0}")"
ROS_GROUND_TEST="$(bool_value "${UNIBOTS_ROS_GROUND_TEST:-0}")"
RELEASE_GATE_AFTER_DOCK="$(bool_value "${UNIBOTS_RELEASE_GATE_AFTER_DOCK:-1}")"
RELEASE_LIVE_SERVO="$(bool_value "${UNIBOTS_RELEASE_LIVE_SERVO:-0}")"

start_node() {
  local name="$1"
  shift
  echo "starting $name: $*"
  setsid "$@" > "$LOG_DIR/${name}.log" 2>&1 &
  pids+=("$!")
  names+=("$name")
}

stop_all() {
  echo "stopping ROS main"
  rm -f "$READY_FILE"
  UNIBOTS_STM32_PORT="$STM32_PORT" bash scripts/unibots_main_stop.sh || true
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
  if [ "${UNIBOTS_ENABLE_GPIO_SAFETY:-1}" = "1" ] && [ "$BUTTON_BACKEND" = "gpio" ]; then
    python3 scripts/unibots_gpio_led.py red "${LED_ARGS[@]}" || true
  fi
}

trap stop_all EXIT INT TERM

start_node safety_io \
  ros2 run safety_io safety_io --ros-args \
    -p enable_gpio:="$ENABLE_GPIO" \
    -p require_gpio:="$REQUIRE_GPIO" \
    -p button_backend:="$BUTTON_BACKEND" \
    -p button_pin:="${UNIBOTS_KILL_BUTTON_PIN:-7}" \
    -p button_active_low:="$BUTTON_ACTIVE_LOW" \
    -p modulino_i2c_bus:="${UNIBOTS_MODULINO_I2C_BUS:-1}" \
    -p modulino_i2c_address:="${UNIBOTS_MODULINO_I2C_ADDRESS:-62}" \
    -p modulino_button_index:="${UNIBOTS_MODULINO_BUTTON_INDEX:-0}" \
    -p red_led_pin:="${UNIBOTS_RED_LED_PIN:-29}" \
    -p green_led_pin:="${UNIBOTS_GREEN_LED_PIN:-31}" \
    -p led_active_low:="$LED_ACTIVE_LOW" \
    -p default_paused:="$START_PAUSED"

sleep 0.5

start_node image_sender \
  ros2 run image_sender image_sender --ros-args \
    -p camera_index:="${UNIBOTS_CAMERA_INDEX:-0}" \
    -p width:="${UNIBOTS_CAMERA_WIDTH:-1280}" \
    -p height:="${UNIBOTS_CAMERA_HEIGHT:-720}" \
    -p fps:="${UNIBOTS_CAMERA_FPS:-20}" \
    -p rotate_180:="$CAMERA_ROTATE_180"

start_node ball_detector \
  ros2 run ball_detector ball_detector --ros-args \
    -p model_path:="$MODEL_PATH" \
    -p confidence:="${UNIBOTS_YOLO_CONF:-0.005}" \
    -p imgsz:="${UNIBOTS_YOLO_IMGSZ:-320}" \
    -p device:="${UNIBOTS_YOLO_DEVICE:-cuda}" \
    -p half:="$YOLO_HALF" \
    -p target_colours:="${UNIBOTS_BALL_COLOURS:-orange}" \
    -p require_colour:="$REQUIRE_BALL_COLOR" \
    -p min_colour_ratio:="${UNIBOTS_MIN_COLOR_RATIO:-0.012}" \
    -p min_diameter_px:="${UNIBOTS_YOLO_MIN_DIAMETER_PX:-2.5}" \
    -p max_diameter_px:="${UNIBOTS_YOLO_MAX_DIAMETER_PX:-160.0}" \
    -p min_aspect:="${UNIBOTS_YOLO_MIN_ASPECT:-0.30}" \
    -p max_aspect:="${UNIBOTS_YOLO_MAX_ASPECT:-3.20}" \
    -p enable_colour_fallback:="${UNIBOTS_ENABLE_COLOR_FALLBACK:-true}" \
    -p fallback_colours:="${UNIBOTS_FALLBACK_BALL_COLOURS:-orange}" \
    -p web_port:="${UNIBOTS_HTTP_PORT:-8770}"

start_node april_tag_detector \
  ros2 run april_tag_detector april_tag_detector --ros-args \
    -p target_tag_ids:="${UNIBOTS_HOME_TAG_IDS:-20,21}" \
    -p tag_size_m:="${UNIBOTS_HOME_TAG_SIZE_M:-0.10}" \
    -p hfov_deg:="${UNIBOTS_CAMERA_HFOV_DEG:-120.0}"

start_node imu \
  ros2 run imu gyro_pub --ros-args \
    -p bus:="${UNIBOTS_IMU_BUS:-auto}" \
    -p address:="${UNIBOTS_IMU_ADDRESS:-auto}" \
    -p calibrate_seconds:="${UNIBOTS_IMU_CALIBRATE_SECONDS:-0.4}"

if [ "${UNIBOTS_START_RAW_LIDAR:-1}" = "1" ]; then
  start_node ldlidar \
    ros2 launch ldlidar_stl_ros2 stl27l.launch.py
  sleep 1
fi

if [ "${UNIBOTS_START_LIDAR_SECTORS:-1}" = "1" ]; then
  start_node lidar \
    ros2 run lidar lidar_scan --ros-args \
      -p front_center_deg:="${UNIBOTS_FRONT_CENTER:-270.0}" \
      -p lidar_stale_s:="${UNIBOTS_LIDAR_STALE_S:-0.60}" \
      -p max_range_m:="${UNIBOTS_LIDAR_MAX_RANGE_M:-2.50}"
fi

decision_args=(
  ros2 run decision_maker decision_maker --ros-args
  -p live_motors:="$ROS_LIVE_MOTORS"
  -p ground_test:="$ROS_GROUND_TEST"
  -p require_safety_io:="$REQUIRE_GPIO"
  -p serial_port:="$STM32_PORT"
  -p target_count:="${UNIBOTS_TARGET_COUNT:-3}"
  -p home_tag_ids:="${UNIBOTS_HOME_TAG_IDS:-20,21}"
  -p mission_timeout_s:="${UNIBOTS_MISSION_TIMEOUT_S:-180.0}"
  -p limit:="${UNIBOTS_PWM_LIMIT:-80}"
  -p approach_pwm:="${UNIBOTS_APPROACH_PWM:-60}"
  -p creep_pwm:="${UNIBOTS_CREEP_PWM:-50}"
  -p collect_drive_pwm:="${UNIBOTS_COLLECT_DRIVE_PWM:-45}"
  -p turn_pwm:="${UNIBOTS_TURN_PWM:-60}"
  -p search_pwm:="${UNIBOTS_SEARCH_PWM:-60}"
  -p step_spin_angle_deg:="${UNIBOTS_STEP_SPIN_ANGLE_DEG:-70.0}"
  -p release_gate_after_dock:="$RELEASE_GATE_AFTER_DOCK"
  -p release_live_servo:="$RELEASE_LIVE_SERVO"
  -p release_servo_pins:="${UNIBOTS_RELEASE_SERVO_PINS:-32,33}"
  -p release_servo_home_us:="${UNIBOTS_RELEASE_SERVO_HOME_US:-1500}"
  -p release_servo_open_us:="${UNIBOTS_RELEASE_SERVO_OPEN_US:-2000}"
  -p release_servo_open_hold_s:="${UNIBOTS_RELEASE_SERVO_OPEN_HOLD_S:-7.0}"
)

if [ "${UNIBOTS_ALLOW_NO_LIDAR_LIVE:-0}" = "1" ]; then
  decision_args+=(-p allow_no_lidar_live:=true)
fi

start_node decision_maker "${decision_args[@]}"

touch "$READY_FILE"
echo "READY_FOR_BUTTON marker=$READY_FILE"
echo "ROS main running. Web view: http://0.0.0.0:${UNIBOTS_HTTP_PORT:-8770}"
echo "Live motors param: UNIBOTS_ROS_LIVE_MOTORS=${UNIBOTS_ROS_LIVE_MOTORS:-0}"

while true; do
  for i in "${!pids[@]}"; do
    if ! kill -0 "${pids[$i]}" 2>/dev/null; then
      echo "node exited: ${names[$i]} pid=${pids[$i]}"
      exit 1
    fi
  done
  sleep 1
done
