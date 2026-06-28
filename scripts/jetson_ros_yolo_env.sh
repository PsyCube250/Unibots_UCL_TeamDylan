#!/usr/bin/env bash
set -eo pipefail

UNIBOTS_WS="${UNIBOTS_WS:-/home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan/unibots_ws}"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "$UNIBOTS_WS/install/setup.bash" ]; then
  source "$UNIBOTS_WS/install/setup.bash"
fi

CUSPARSELT_DIR="$UNIBOTS_WS/venv/lib/python3.10/site-packages/nvidia/cusparselt/lib"
if [ -d "$CUSPARSELT_DIR" ]; then
  export LD_LIBRARY_PATH="$CUSPARSELT_DIR:${LD_LIBRARY_PATH:-}"
fi

export PYTHONUNBUFFERED=1
