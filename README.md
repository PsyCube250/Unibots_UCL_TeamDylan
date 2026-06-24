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
| `unibots_ws/` | ROS 2 workspace with camera, ball detection, AprilTag, LiDAR, IMU, and decision-maker packages. Some ROS decision logic is older than the current STM32 PWM protocol. |
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

## Active Bring-Up Scripts

All commands below are safe defaults unless they explicitly include both
`--live-motors` and `--ground-test`.

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
  --http-port 8766
```

Live floor movement requires explicit operator approval:

```bash
python3 robot_control/lidar_explore_monitor.py \
  --front-center 270 \
  --duration 60 \
  --live-motors \
  --ground-test
```

### Ping-Pong Ball Tracker

Camera + YOLO/color fallback monitor only, no motor output:

```bash
python3 robot_control/pingpong_yolo_tracker.py \
  --camera /dev/video0 \
  --front-center 270 \
  --http-port 8770
```

Open from the Mac:

```text
http://192.168.55.1:8770
```

The tracker uses YOLO first and falls back to yellow/white circular blob
detection because a generic COCO model normally has `sports ball`, not a
specific ping-pong-ball class. Three collected targets trigger an open-loop
return replay. Without reliable encoders/IMU odometry, this return is only an
approximation.

Live tracking movement requires explicit operator approval:

```bash
python3 robot_control/pingpong_yolo_tracker.py \
  --camera /dev/video0 \
  --front-center 270 \
  --duration 60 \
  --live-motors \
  --ground-test
```

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

- `image_sender`: V4L2 camera publisher on `/img`.
- `ball_detector`: YOLO detector publishing ball navigation messages.
- `april_tag_detector`: AprilTag detector for arena fiducials.
- `lidar`: STL-27L ROS LiDAR parser.
- `imu`: IMU package.
- `decision_maker`: older ROS state machine.

Important: `decision_maker` still uses an older command style such as
`FORWARD,180` and `TURN,...`. The current STM32 firmware used during bring-up
expects the safer `PWM fl fr bl br` protocol with a low `LIMIT`, so do not use
the old ROS decision node for motor output until it is updated.

## Git Hygiene

Generated files are ignored: ROS build/install/log folders, Python bytecode,
PlatformIO build artifacts, local logs/backups, editor folders, and model
exports (`*.pt`, `*.engine`, `*.onnx`). Some historical model files are still
tracked because older ROS code references them; avoid adding more binary model
artifacts unless the team decides they belong in Git.
