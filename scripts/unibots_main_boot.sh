#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="${UNIBOTS_LOG_DIR:-$REPO_DIR/logs}"
mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG_DIR/unibots_main_boot.log") 2>&1

cd "$REPO_DIR"

echo "==== unibots main boot $(date -Iseconds) ===="
echo "repo=$REPO_DIR"

if [ "${UNIBOTS_MAIN_MODE:-ros2}" = "ros2" ]; then
  exec "$REPO_DIR/scripts/unibots_ros_main_boot.sh"
fi

if [ "${UNIBOTS_SOURCE_ROS_ENV:-0}" = "1" ] && [ -f "$REPO_DIR/scripts/jetson_ros_yolo_env.sh" ]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/scripts/jetson_ros_yolo_env.sh" >/dev/null 2>&1 || true
fi

CUSPARSELT_DIR="$REPO_DIR/unibots_ws/venv/lib/python3.10/site-packages/nvidia/cusparselt/lib"
if [ -d "$CUSPARSELT_DIR" ]; then
  export LD_LIBRARY_PATH="$CUSPARSELT_DIR:${LD_LIBRARY_PATH:-}"
fi

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

choose_lidar_port() {
  if [ -n "${UNIBOTS_LIDAR_PORT:-}" ]; then
    echo "$UNIBOTS_LIDAR_PORT"
    return
  fi
  local by_id
  by_id="$(ls -1 /dev/serial/by-id/*Silicon_Labs* 2>/dev/null | head -n 1 || true)"
  if [ -n "$by_id" ]; then
    echo "$by_id"
    return
  fi
  local tty
  tty="$(ls -1 /dev/ttyUSB* 2>/dev/null | head -n 1 || true)"
  if [ -n "$tty" ]; then
    echo "$tty"
    return
  fi
  echo "/dev/ttyUSB0"
}

wait_lidar_port() {
  local timeout_s="${1:-45}"
  local start_s port
  start_s="$(date +%s)"
  while true; do
    port="$(choose_lidar_port)"
    if [ -e "$port" ]; then
      echo "$port"
      return
    fi
    if [ $(( "$(date +%s)" - start_s )) -ge "$timeout_s" ]; then
      echo "missing lidar after ${timeout_s}s, last candidate: $port" >&2
      return 1
    fi
    sleep 0.5
  done
}

CAMERA="${UNIBOTS_CAMERA:-/dev/video0}"
STM32_PORT="${UNIBOTS_STM32_PORT:-/dev/ttyTHS1}"
DEVICE_WAIT_S="${UNIBOTS_DEVICE_WAIT_S:-60}"

wait_path "camera" "$CAMERA" "$DEVICE_WAIT_S"
LIDAR_PORT="$(wait_lidar_port "$DEVICE_WAIT_S")"
wait_path "stm32" "$STM32_PORT" "$DEVICE_WAIT_S"

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

safety_args=()
if [ "${UNIBOTS_ENABLE_GPIO_SAFETY:-0}" != "1" ]; then
  safety_args+=(--disable-safety-io)
fi

home_args=()
if [ "${UNIBOTS_HOME_DOCK_ENABLE:-1}" = "0" ]; then
  home_args+=(--no-home-dock-enable)
fi

release_args=()
if [ "${UNIBOTS_RELEASE_LIVE_SERVO:-0}" = "1" ]; then
  release_args+=(--release-live-servo)
fi

cmd=(
  "$PYTHON_BIN" robot_control/pingpong_yolo_tracker.py
  --camera "$CAMERA"
  --camera-rotate "${UNIBOTS_CAMERA_ROTATE:-180}"
  --lidar-port "$LIDAR_PORT"
  --front-center "${UNIBOTS_FRONT_CENTER:-270}"
  --detector yolo
  --model "$MODEL_PATH"
  --device "${UNIBOTS_YOLO_DEVICE:-cuda}"
  --half
  --imgsz "${UNIBOTS_YOLO_IMGSZ:-320}"
  --yolo-class-ids "${UNIBOTS_YOLO_CLASS_IDS:-32}"
  --yolo-conf "${UNIBOTS_YOLO_CONF:-0.005}"
  --require-ball-color
  --target-colors "${UNIBOTS_TARGET_COLORS:-orange,white}"
  --prefer-color "${UNIBOTS_PREFER_COLOR:-any}"
  --min-color-ratio "${UNIBOTS_MIN_COLOR_RATIO:-0.012}"
  --yolo-min-diameter-px "${UNIBOTS_YOLO_MIN_DIAMETER_PX:-2.5}"
  --yolo-max-diameter-px "${UNIBOTS_YOLO_MAX_DIAMETER_PX:-140}"
  --yolo-min-aspect "${UNIBOTS_YOLO_MIN_ASPECT:-0.30}"
  --yolo-max-aspect "${UNIBOTS_YOLO_MAX_ASPECT:-3.20}"
  --roi-y-min-fraction "${UNIBOTS_ROI_Y_MIN:-0.15}"
  --roi-y-max-fraction "${UNIBOTS_ROI_Y_MAX:-0.98}"
  --target-hold-s "${UNIBOTS_TARGET_HOLD_S:-0.65}"
  --target-smooth-alpha "${UNIBOTS_TARGET_SMOOTH_ALPHA:-0.60}"
  --target-smooth-max-jump-px "${UNIBOTS_TARGET_SMOOTH_MAX_JUMP_PX:-190}"
  --target-switch-lock-s "${UNIBOTS_TARGET_SWITCH_LOCK_S:-1.1}"
  --target-cluster-gap-px "${UNIBOTS_TARGET_CLUSTER_GAP_PX:-120}"
  --target-cluster-bonus "${UNIBOTS_TARGET_CLUSTER_BONUS:-0.35}"
  --target-random-jitter "${UNIBOTS_TARGET_RANDOM_JITTER:-0}"
  --drive-mode tank_step
  --duration "${UNIBOTS_DURATION:-0}"
  --limit "${UNIBOTS_PWM_LIMIT:-45}"
  --kick-pwm "${UNIBOTS_KICK_PWM:-42}"
  --kick-turn-pwm "${UNIBOTS_KICK_TURN_PWM:-5}"
  --kick-s "${UNIBOTS_KICK_S:-0.18}"
  --kick-cooldown-s "${UNIBOTS_KICK_COOLDOWN_S:-1.2}"
  --approach-pwm "${UNIBOTS_APPROACH_PWM:-36}"
  --creep-pwm "${UNIBOTS_CREEP_PWM:-34}"
  --collect-drive-pwm "${UNIBOTS_COLLECT_DRIVE_PWM:-28}"
  --collect-drive-s "${UNIBOTS_COLLECT_DRIVE_S:-0.30}"
  --collect-hold-s "${UNIBOTS_COLLECT_HOLD_S:-0.20}"
  --turn-pwm "${UNIBOTS_TURN_PWM:-28}"
  --search-pwm "${UNIBOTS_SEARCH_PWM:-32}"
  --search-imu-turn-deg "${UNIBOTS_SEARCH_IMU_TURN_DEG:-135}"
  --search-imu-step-s "${UNIBOTS_SEARCH_IMU_STEP_S:-0.75}"
  --search-imu-alternate
  --allow-imu-search-without-front-lidar
  --explore-forward-pwm "${UNIBOTS_EXPLORE_FORWARD_PWM:-24}"
  --escape-pwm "${UNIBOTS_ESCAPE_PWM:-18}"
  --step-arc-forward-pwm "${UNIBOTS_STEP_ARC_FORWARD_PWM:-34}"
  --step-arc-turn-pwm "${UNIBOTS_STEP_ARC_TURN_PWM:-8}"
  --step-forward-s "${UNIBOTS_STEP_FORWARD_S:-0.38}"
  --step-arc-s "${UNIBOTS_STEP_ARC_S:-0.35}"
  --step-search-s "${UNIBOTS_STEP_SEARCH_S:-0.50}"
  --search-sweep-s "${UNIBOTS_SEARCH_SWEEP_S:-4.0}"
  --collect-distance-m "${UNIBOTS_COLLECT_DISTANCE_M:-0.22}"
  --collect-diameter-px "${UNIBOTS_COLLECT_DIAMETER_PX:-145}"
  --collect-bottom-y-fraction "${UNIBOTS_COLLECT_BOTTOM_Y_FRACTION:-0.72}"
  --collect-bottom-edge-fraction "${UNIBOTS_COLLECT_BOTTOM_EDGE_FRACTION:-0.88}"
  --collect-bottom-center-fraction "${UNIBOTS_COLLECT_BOTTOM_CENTER_FRACTION:-0.18}"
  --home-tag-ids "${UNIBOTS_HOME_TAG_IDS:-20,21}"
  --home-tag-size-m "${UNIBOTS_HOME_TAG_SIZE_M:-0.10}"
  --home-tag-hold-s "${UNIBOTS_HOME_TAG_HOLD_S:-1.80}"
  --home-dock-approach-m "${UNIBOTS_HOME_DOCK_APPROACH_M:-0.20}"
  --home-dock-distance-tolerance-m "${UNIBOTS_HOME_DOCK_TOLERANCE_M:-0.04}"
  --home-align-gain "${UNIBOTS_HOME_ALIGN_GAIN:-1.2}"
  --home-search-pwm "${UNIBOTS_HOME_SEARCH_PWM:-34}"
  --home-search-boost-pwm "${UNIBOTS_HOME_SEARCH_BOOST_PWM:-45}"
  --home-search-sweep-deg "${UNIBOTS_HOME_SEARCH_SWEEP_DEG:-180}"
  --home-search-segment-deg "${UNIBOTS_HOME_SEARCH_SEGMENT_DEG:-45}"
  --home-search-stall-s "${UNIBOTS_HOME_SEARCH_STALL_S:-0.90}"
  --home-search-progress-epsilon-deg "${UNIBOTS_HOME_SEARCH_PROGRESS_EPSILON_DEG:-3.0}"
  --home-align-turn-pwm "${UNIBOTS_HOME_ALIGN_TURN_PWM:-28}"
  --home-align-boost-pwm "${UNIBOTS_HOME_ALIGN_BOOST_PWM:-45}"
  --home-align-stall-s "${UNIBOTS_HOME_ALIGN_STALL_S:-0.80}"
  --home-align-progress-epsilon-deg "${UNIBOTS_HOME_ALIGN_PROGRESS_EPSILON_DEG:-2.0}"
  --home-align-step-s "${UNIBOTS_HOME_ALIGN_STEP_S:-0.25}"
  --home-min-turn-pwm "${UNIBOTS_HOME_MIN_TURN_PWM:-22}"
  --home-approach-pwm "${UNIBOTS_HOME_APPROACH_PWM:-32}"
  --home-creep-pwm "${UNIBOTS_HOME_CREEP_PWM:-24}"
  --home-step-forward-s "${UNIBOTS_HOME_STEP_FORWARD_S:-0.35}"
  --home-front-stop-m "${UNIBOTS_HOME_FRONT_STOP_M:-0.16}"
  --home-front-slow-m "${UNIBOTS_HOME_FRONT_SLOW_M:-0.34}"
  --home-kick-pwm "${UNIBOTS_HOME_KICK_PWM:-45}"
  --home-kick-s "${UNIBOTS_HOME_KICK_S:-0.25}"
  --home-turn-180-pwm "${UNIBOTS_HOME_TURN_180_PWM:-30}"
  --home-turn-boost-pwm "${UNIBOTS_HOME_TURN_BOOST_PWM:-42}"
  --home-backup-pwm "${UNIBOTS_HOME_BACKUP_PWM:-18}"
  --home-backup-force-pwm "${UNIBOTS_HOME_BACKUP_FORCE_PWM:-26}"
  --home-backup-force-s "${UNIBOTS_HOME_BACKUP_FORCE_S:-1.30}"
  --home-rear-dock-m "${UNIBOTS_HOME_REAR_DOCK_M:-0.13}"
  --home-rear-min-settle-s "${UNIBOTS_HOME_REAR_MIN_SETTLE_S:-0.80}"
  --release-servo-pin "${UNIBOTS_RELEASE_SERVO_PIN:-32}"
  --release-servo-backend "${UNIBOTS_RELEASE_SERVO_BACKEND:-hardware-pwm}"
  --release-servo-home-us "${UNIBOTS_RELEASE_SERVO_HOME_US:-1500}"
  --release-servo-open-us "${UNIBOTS_RELEASE_SERVO_OPEN_US:-2000}"
  --release-servo-open-hold-s "${UNIBOTS_RELEASE_SERVO_OPEN_HOLD_S:-7.0}"
  --lidar-stop-m "${UNIBOTS_LIDAR_STOP_M:-0.24}"
  --lidar-slow-m "${UNIBOTS_LIDAR_SLOW_M:-0.42}"
  --front-width "${UNIBOTS_FRONT_WIDTH:-36}"
  --diagonal-width "${UNIBOTS_DIAGONAL_WIDTH:-30}"
  --front-stat "${UNIBOTS_FRONT_STAT:-median}"
  --side-stop-m "${UNIBOTS_SIDE_STOP_M:-0.18}"
  --rear-stop-m "${UNIBOTS_REAR_STOP_M:-0.18}"
  --stuck-seconds "${UNIBOTS_STUCK_SECONDS:-2.4}"
  --stuck-recover-s "${UNIBOTS_STUCK_RECOVER_S:-0.65}"
  --enable-imu
  --require-imu
  --imu-calibrate-seconds "${UNIBOTS_IMU_CALIBRATE_SECONDS:-0.4}"
  --stm32-port "$STM32_PORT"
  --allow-no-stm32-ack
  "${home_args[@]}"
  "${release_args[@]}"
  "${safety_args[@]}"
  --live-motors
  --ground-test
  --host "${UNIBOTS_HTTP_HOST:-0.0.0.0}"
  --http-port "${UNIBOTS_HTTP_PORT:-8770}"
)

if [ -n "${UNIBOTS_MAIN_EXTRA_ARGS:-}" ]; then
  # Deliberately split operator-provided extra args.
  # shellcheck disable=SC2206
  extra_args=( $UNIBOTS_MAIN_EXTRA_ARGS )
  cmd+=("${extra_args[@]}")
fi

echo "python=$PYTHON_BIN"
echo "model=$MODEL_PATH"
echo "lidar=$LIDAR_PORT"
echo "http=http://0.0.0.0:${UNIBOTS_HTTP_PORT:-8770}"
echo "starting main"
exec "${cmd[@]}"
