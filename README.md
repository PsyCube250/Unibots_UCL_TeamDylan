# Unibots UCL Team Dylan Robot

Software and firmware for the Unibots UCL Team Dylan four-wheel robot. The
current working stack is intentionally staged and safety-first: Jetson handles
sensors and high-level decisions, while the STM32 handles motor outputs,
watchdog shutdown, and future encoder feedback.

## Current Hardware

| Device | Current notes |
| --- | --- |
| Jetson | Remote access over USB device networking, usually `jetson@192.168.55.1`. |
| STM32 | UART on Jetson `/dev/ttyTHS1`, 115200 baud. Text protocol: `STOP`, `ENABLE`, `LIMIT`, `PWM fl fr bl br`, telemetry. |
| STL-27L LiDAR | USB-to-TTL CP2102, 921600 baud. Stable path preferred: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`. Current robot front is raw angle `270`. |
| USB camera | ELP-USBGS1200P01-H120, 2 MP global shutter, UVC, typically `/dev/video0`. |
| Motor drivers | DRV8833-style IN1/IN2 boards driven only by STM32 pins. |
| Wheels | Four omni/mecanum-style wheels. Current control is open-loop PWM; encoder wiring is documented but not yet reliable. |
| AprilTags | Rulebook fiducials are 100 mm x 100 mm, with tag top aligned to the top of the arena wall. |

Camera references:

- CAD/community model: https://grabcad.com/library/elp-usbgs1200p01-h120-1/files
- ELP product family: https://www.elpcctv.com/elp-2mp-ar0234-sensor-1200p-1080p-90fps-global-shutter-usb-camera-p-388.html

## Safety Rules

- Do not run motors unless the robot is physically ready and the operator has
  explicitly approved that test.
- First motor tests must be lifted-wheel tests.
- Floor tests must have LiDAR safety enabled, low PWM limits, and a human ready
  to cut motor power.
- Jetson must not power the motors.
- Jetson, STM32, and motor drivers must share signal ground where required.
- If LiDAR data is stale, camera data is stale, or STM32 communication fails,
  stop the robot.
- Firmware flashing and autonomous floor movement require explicit approval.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `robot_control/` | Current one-file diagnostic and control scripts used during bring-up. These are the safest entry points right now. |
| `stm32/proposed_firmware/four_wheel_encoder_demo/` | PlatformIO STM32 firmware with UART command protocol, motor watchdog, PWM limit, telemetry, and encoder scaffolding. |
| `unibots_ws/` | ROS 2 workspace with camera, ball detection, AprilTag, LiDAR, IMU, and decision-maker packages using the current STM32 `PWM fl fr bl br` protocol. |
| `scripts/` | Jetson environment, package install, build, and Mac internet-sharing helper scripts. |
| `docs/` | Wiring and bring-up notes. |
| `Testing_Code/` | Legacy experiments and reference snippets. Keep for history; prefer `robot_control/` and `stm32/proposed_firmware/` for active work. |

## Jetson Setup

SSH from the Mac:

```bash
ssh jetson@192.168.55.1
cd ~/Documents/Unibots/Unibots_UCL_TeamDylan
```

Check the Jetson environment and connected hardware:

```bash
bash scripts/jetson_check_env.sh
```

Source the ROS/project environment before ROS work:

```bash
source scripts/jetson_env.sh
```

Build the ROS workspace without running any motors:

```bash
bash scripts/jetson_build_workspace.sh
```

## Boot Autostart

With the physical switch installed, the Jetson can start the main ping-pong
tracker automatically on every boot. The system service runs as the normal
`jetson` user, waits for camera/LiDAR/STM32 device paths to appear, and restarts
if the process exits before the robot is powered off.

Install or update the boot service on the Jetson. Installation alone does not
make the robot run at boot; the final boolean switch below controls that:

```bash
bash scripts/install_unibots_autostart.sh
```

Final autostart switch:

```bash
bash scripts/unibots_autostart_switch.sh status
bash scripts/unibots_autostart_switch.sh on
bash scripts/unibots_autostart_switch.sh off
```

The switch writes `/etc/default/unibots-main`:

```text
UNIBOTS_AUTOSTART_ENABLED=0 or 1
```

When `UNIBOTS_AUTOSTART_ENABLED=1`, the Jetson starts the main tracker at boot.
When it is `0`, the Jetson boots normally and the service refuses to run main.

Start immediately only when the physical switch/safety setup is ready:

```bash
sudo systemctl restart unibots-main.service
```

Check service status:

```bash
systemctl status unibots-main.service
```

Stop the service and send STM32 STOP:

```bash
sudo systemctl stop unibots-main.service
bash scripts/unibots_main_stop.sh
```

Disable autostart:

```bash
bash scripts/unibots_autostart_switch.sh off
sudo systemctl disable --now unibots-main.service
```

## Active Bring-Up Scripts

All commands below are safe defaults unless they explicitly include both
`--live-motors` and `--ground-test`.

### Jetson Kill Switch, Red/Green LED, and IMU

Current Jetson header wiring:

| Function | Jetson Nano physical pin |
| --- | --- |
| Kill switch input | 7 / GPIO09 |
| Red LED output | 29 / GPIO01 |
| Green LED output | 31 / GPIO11 |
| LED/switch ground | 34, 39, or another GND |
| IMU SDA | 27 / I2C0_SDA |
| IMU SCL | 28 / I2C0_SCL |
| IMU GND | 25 / GND |
| IMU power | 17 / 3.3V |

No blue LED is used. Red and green can be on at the same time: KILL is red,
running is green, and warning/IMU fault is red+green. The Jetson currently
reports as an Orin Nano Super, so `robot_control/jetson_safety_imu.py` applies
a compatibility setting before loading `Jetson.GPIO`.

The default kill-switch logic is fail-safe: pin 7 LOW means run/OK, pin 7 HIGH
means KILL or wire-open. This Jetson GPIO library warns that it ignores
software pull-up configuration, so use an external pull-up, for example 10 kOhm
from pin 7 to 3.3V, with the run switch shorting pin 7 to GND. Use
`--kill-active-low` only if the switch wiring is intentionally the opposite.
The detected IMU is an LSM6DS-family device at I2C bus 1, address `0x6a`.

Read the kill switch, drive only the red/green LED, and read IMU yaw without
moving motors:

```bash
python3 robot_control/jetson_safety_imu.py \
  --enable-safety-io \
  --enable-imu \
  --imu-bus 1 \
  --imu-address 0x6a \
  --duration 10
```

### Servo Gate Test

The ping-pong release gate uses one standard 3-wire hobby servo. Use a separate
regulated 5V servo supply; do not power the servo from Jetson 3.3V, and avoid
using the Jetson 5V rail for a loaded gate servo. The servo supply ground must
be shared with the Jetson ground or the PWM signal has no reference.

| Servo wire | Connection |
| --- | --- |
| Signal, often orange/yellow/white | Jetson physical pin 32 / GPIO07 |
| V+ / red | External regulated 5V servo supply + |
| GND / brown/black | External 5V supply - and Jetson GND, such as pin 34 or 39 |

Dry-run only, no PWM output:

```bash
python3 robot_control/servo_gate_test.py
```

After the servo is powered, the mechanism is clear, and the operator approves
the actuator test, run one open/close cycle:

```bash
python3 robot_control/servo_gate_test.py \
  --pin 32 \
  --home-us 1500 \
  --open-us 2000 \
  --cycles 1 \
  --live-servo
```

If pin 32 has not been enabled as hardware PWM in Jetson-IO yet, the hardware
PWM command may complete without movement. For wiring/power verification only,
use the software GPIO pulse fallback:

```bash
python3 robot_control/servo_gate_test.py \
  --backend gpio-bitbang \
  --pin 32 \
  --home-us 1500 \
  --open-us 2000 \
  --cycles 1 \
  --live-servo
```

The software fallback is less precise than hardware PWM, but it should at
least make a correctly powered servo respond. Use hardware PWM for the final
release gate calibration.

Pulse width controls position more repeatably than abstract degrees. Start with
`1500us` as the closed/home position. `2000us` is a typical 90-degree move in
one direction; if the gate moves the wrong way, use `--open-us 1000` instead.
Tune the exact `--home-us` and `--open-us` values with the gate attached but
not jammed against hard stops.

### LiDAR Live View

Read-only LiDAR browser monitor:

```bash
python3 robot_control/lidar_live_view.py \
  --port "$UNIBOTS_LIDAR_PORT" \
  --front-center 270 \
  --http-port 8765
```

Open from the Mac:

```text
http://192.168.55.1:8765
```

### LiDAR Exploration Monitor

Monitor/preflight only, no motor output:

```bash
python3 robot_control/lidar_explore_monitor.py \
  --front-center 270 \
  --enable-safety-io \
  --enable-imu \
  --imu-bus 1 \
  --imu-address 0x6a \
  --http-port 8766
```

Live floor movement requires explicit operator approval:

```bash
python3 robot_control/lidar_explore_monitor.py \
  --front-center 270 \
  --duration 60 \
  --enable-imu \
  --imu-bus 1 \
  --imu-address 0x6a \
  --live-motors \
  --ground-test
```

### Ping-Pong Ball Tracker

Camera + YOLO/color fallback monitor only, no motor output:

```bash
python3 robot_control/pingpong_yolo_tracker.py \
  --camera /dev/video0 \
  --front-center 270 \
  --enable-safety-io \
  --enable-imu \
  --imu-bus 1 \
  --imu-address 0x6a \
  --http-port 8770
```

Open from the Mac:

```text
http://192.168.55.1:8770
```

The tracker uses YOLO first and falls back to yellow/white circular blob
detection because a generic COCO model normally has `sports ball`, not a
specific ping-pong-ball class. The active controller scores multiple balls by
camera size/centering and LiDAR clearance, penalising balls that appear close to
a wall. It uses a simple camera PD turn controller and LiDAR side guards for
the 20 cm × 20 cm chassis: by default it stops/skirts when the front path is
under `0.24 m`, slows under `0.42 m`, and steers away from side walls under
about `0.13-0.22 m`. Three collected targets trigger an open-loop return
replay. Without reliable encoders/IMU odometry, this return is only an
approximation.

Live tracking movement requires explicit operator approval:

```bash
python3 robot_control/pingpong_yolo_tracker.py \
  --camera /dev/video0 \
  --front-center 270 \
  --duration 60 \
  --enable-imu \
  --imu-bus 1 \
  --imu-address 0x6a \
  --approach-pwm 28 \
  --creep-pwm 18 \
  --turn-gain 1.0 \
  --turn-kd 0.10 \
  --lidar-stop-m 0.24 \
  --lidar-slow-m 0.42 \
  --side-stop-m 0.13 \
  --side-slow-m 0.22 \
  --target-clear-m 0.30 \
  --live-motors \
  --ground-test
```

Live mode automatically requires the Jetson kill-switch GPIO unless
`--disable-safety-io` is deliberately supplied. Add `--require-imu` only after
the IMU readings are stable; with that flag, a missing/stale IMU also forces
STOP.

### STM32 Telemetry and Emergency Stop

Telemetry-only readout:

```bash
python3 robot_control/stm32_encoder_demo.py \
  --port /dev/ttyTHS1 \
  --show-raw \
  --send-stop
```

Send STOP and disable motors:

```bash
python3 robot_control/stm32_encoder_demo.py \
  --port /dev/ttyTHS1 \
  --send-stop \
  --send-stop-on-exit
```

## STM32 Firmware

The active proposed firmware is:

```text
stm32/proposed_firmware/four_wheel_encoder_demo/
```

It expects:

| Wheel | STM32 motor pins |
| --- | --- |
| Front-left | PA0, PA1 |
| Front-right | PA2, PA3 |
| Back-left | PA6, PA7 |
| Back-right | PB0, PB1 |

Jetson UART:

| Jetson Nano | STM32 |
| --- | --- |
| Pin 8 TX | PA10 RX |
| Pin 10 RX | PA9 TX |
| GND | GND |

Planned encoder pins are documented in
`docs/encoder_mecanum_wiring.md`. Encoder readings are not yet a proven control
source, so current autonomous tests remain open-loop with LiDAR safety.

Flash only after explicit approval:

```bash
cd stm32/proposed_firmware/four_wheel_encoder_demo
platformio run -t upload
```

## ROS 2 Packages

The ROS workspace contains:

- `image_sender`: V4L2 camera publisher on `/img`, using manual
  `sensor_msgs/Image` conversion instead of `cv_bridge`.
- `ball_detector`: YOLO class-32 detector with white/orange/yellow colour and
  shape filtering. It subscribes to `/img`, publishes `/ball_detections` and
  `/ball_navigate`, includes bottom/center collection hints, and serves an
  annotated MJPEG view on port `8770`.
- `april_tag_detector`: AprilTag detector for arena fiducials. It publishes all
  tags plus a chosen `target` for IDs `20,21`, using the midpoint if both are
  visible.
- `lidar`: consumes `/scan` and publishes `/obstacle` plus `/lidar_sectors`
  with front/front-left/front-right/left/right/rear distances and stale status.
- `imu`: reads LSM6DS/MPU6050-style I2C IMUs, publishes `/rotational_vel` and
  `/imu_state` with integrated yaw.
- `decision_maker`: ROS mission controller using the safer STM32 `PWM` text
  protocol. It is dry-run by default and only opens/enables motors when both
  `live_motors` and `ground_test` parameters are true. The ROS mission sequence
  is: collect 3 balls, return to target tags `20,21`, align and approach, turn
  180 degrees using IMU yaw, reverse dock, then release the gate. Servo release
  is also dry-run unless `release_live_servo` is true.

Monitor ball detection through ROS2 without moving motors:

```bash
cd unibots_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run image_sender image_sender
ros2 run ball_detector ball_detector
ros2 run april_tag_detector april_tag_detector
ros2 run lidar lidar_scan
ros2 run imu gyro_pub
```

Run the ROS decision node in dry-run mode:

```bash
ros2 run decision_maker decision_maker
```

Live ROS motor output still requires explicit operator approval:

```bash
ros2 run decision_maker decision_maker --ros-args \
  -p live_motors:=true \
  -p ground_test:=true \
  -p serial_port:=/dev/ttyTHS1 \
  -p home_tag_ids:=20,21
```

## Git Hygiene

Generated files are ignored: ROS build/install/log folders, Python bytecode,
PlatformIO build artifacts, local logs/backups, editor folders, and model
exports (`*.pt`, `*.engine`, `*.onnx`). Some historical model files are still
tracked because older ROS code references them; avoid adding more binary model
artifacts unless the team decides they belong in Git.
