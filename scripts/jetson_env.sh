#!/usr/bin/env bash
# Source this file on the Jetson before building/running ROS nodes:
#   source scripts/jetson_env.sh
#
# It intentionally disables Python user-site packages because this Jetson has a
# user-installed NumPy 2.x that breaks ROS Humble cv_bridge binaries built
# against NumPy 1.x.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced, not executed:"
  echo "  source ${BASH_SOURCE[0]}"
  exit 2
fi

export UNIBOTS_ROOT="${UNIBOTS_ROOT:-$HOME/Documents/Unibots/Unibots_UCL_TeamDylan}"
export UNIBOTS_WS="$UNIBOTS_ROOT/unibots_ws"

export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# The Orin Nano Super devkit reports a model string not recognized by the
# installed Jetson.GPIO/Blinka stack. Force the closest supported identifiers
# without editing system Python packages.
export JETSON_MODEL_NAME="${JETSON_MODEL_NAME:-JETSON_ORIN_NANO}"
export BLINKA_FORCECHIP="${BLINKA_FORCECHIP:-T234}"
export BLINKA_FORCEBOARD="${BLINKA_FORCEBOARD:-JETSON_ORIN_NANO}"

_unibots_had_nounset=0
case "$-" in
  *u*) _unibots_had_nounset=1; set +u ;;
esac

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "WARN: /opt/ros/humble/setup.bash not found"
fi

if [[ -f "$UNIBOTS_WS/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$UNIBOTS_WS/install/setup.bash"
fi

if [[ "$_unibots_had_nounset" == "1" ]]; then
  set -u
fi
unset _unibots_had_nounset

export UNIBOTS_LIDAR_PORT="${UNIBOTS_LIDAR_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
export UNIBOTS_STM32_PORT="${UNIBOTS_STM32_PORT:-/dev/ttyTHS1}"

echo "Unibots environment ready"
echo "  UNIBOTS_ROOT=$UNIBOTS_ROOT"
echo "  UNIBOTS_WS=$UNIBOTS_WS"
echo "  PYTHONNOUSERSITE=$PYTHONNOUSERSITE"
echo "  JETSON_MODEL_NAME=$JETSON_MODEL_NAME"
echo "  UNIBOTS_LIDAR_PORT=$UNIBOTS_LIDAR_PORT"
echo "  UNIBOTS_STM32_PORT=$UNIBOTS_STM32_PORT"
