#!/usr/bin/env bash
set -euo pipefail

ROOT="${UNIBOTS_ROOT:-$HOME/Documents/Unibots/Unibots_UCL_TeamDylan}"
cd "$ROOT"

source scripts/jetson_env.sh >/dev/null

cd "$UNIBOTS_WS"

# Safe build only. This does not run nodes and does not move motors.
colcon build --symlink-install \
  --packages-select lidar imu image_sender decision_maker april_tag_detector ball_detector \
  --event-handlers console_direct+
