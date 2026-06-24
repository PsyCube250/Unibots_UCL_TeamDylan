#!/usr/bin/env bash
set -u

ROOT="${UNIBOTS_ROOT:-$HOME/Documents/Unibots/Unibots_UCL_TeamDylan}"

echo "=== System ==="
cat /etc/os-release | sed -n '1,5p'
uname -a
dpkg-query --show nvidia-l4t-core 2>/dev/null || true
date
timedatectl 2>/dev/null | sed -n '1,8p' || true

echo
echo "=== Network ==="
ping -c 1 -W 2 8.8.8.8 >/dev/null && echo "IP connectivity: OK" || echo "IP connectivity: FAIL"
ping -c 1 -W 2 archive.ubuntu.com >/dev/null && echo "DNS: OK" || echo "DNS: FAIL"

echo
echo "=== Devices ==="
lsusb
echo "-- serial --"
ls -l /dev/serial/by-id/ 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>/dev/null || true
echo "-- video --"
ls -l /dev/video* 2>/dev/null || true
echo "-- i2c --"
i2cdetect -l 2>/dev/null || true
echo "-- groups --"
groups

echo
echo "=== ROS/Python With Project Env ==="
if [[ -f "$ROOT/scripts/jetson_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "$ROOT/scripts/jetson_env.sh" >/dev/null
else
  export PYTHONNOUSERSITE=1
  [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
fi

if command -v ros2 >/dev/null; then
  echo "ros2 CLI: OK ($(command -v ros2))"
  ros2 pkg list >/dev/null && echo "ros2 packages: OK" || echo "ros2 packages: FAIL"
else
  echo "ros2 CLI: MISSING"
fi
if command -v colcon >/dev/null; then
  echo "colcon: OK ($(command -v colcon))"
else
  echo "colcon: MISSING"
fi

python3 - <<'PY'
import importlib
mods = [
    "rclpy",
    "std_msgs",
    "sensor_msgs",
    "geometry_msgs",
    "cv_bridge",
    "serial",
    "cv2",
    "numpy",
    "board",
    "busio",
    "adafruit_lsm6ds",
]
for name in mods:
    try:
        mod = importlib.import_module(name)
        print(f"OK   {name:16s} {getattr(mod, '__version__', '')}")
    except Exception as exc:
        print(f"MISS {name:16s} {exc!r}")
PY

echo
echo "=== Workspace ==="
if [[ -d "$ROOT/unibots_ws" ]]; then
  cd "$ROOT/unibots_ws"
  colcon list 2>/dev/null || true
else
  echo "Workspace not found: $ROOT/unibots_ws"
fi
