#!/usr/bin/env bash
set -euo pipefail

cat <<'MSG'
This installs missing Jetson/ROS dependencies only.
It does not run robot nodes, move motors, or flash STM32 firmware.

Requirements before running:
  - Jetson must have internet/DNS.
  - Run this directly on the Jetson terminal so sudo can ask for your password.
MSG

echo
echo "Checking network..."
ping -c 1 -W 2 8.8.8.8 >/dev/null || {
  echo "ERROR: No IP connectivity. Connect Jetson to Wi-Fi/Ethernet or enable Mac Internet Sharing."
  exit 1
}
ping -c 1 -W 2 archive.ubuntu.com >/dev/null || {
  echo "ERROR: DNS is not working. Fix network/DNS before installing."
  exit 1
}

echo
echo "Installing apt packages..."
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-rclpy \
  ros-humble-ros2cli \
  ros-humble-ros2run \
  ros-humble-ros2topic \
  ros-humble-launch \
  ros-humble-launch-ros \
  ros-humble-cv-bridge \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  python3-numpy \
  python3-opencv \
  python3-smbus \
  i2c-tools \
  v4l-utils

echo
echo "Installing small IMU Python packages into system site-packages..."
sudo -H python3 -m pip install --upgrade \
  adafruit-blinka \
  adafruit-circuitpython-lsm6ds

echo
echo "Skipping torch/ultralytics for now: those are large Jetson-specific installs."
echo "Done. Now run:"
echo "  cd ~/Documents/Unibots/Unibots_UCL_TeamDylan"
echo "  source scripts/jetson_env.sh"
echo "  scripts/jetson_check_env.sh"
