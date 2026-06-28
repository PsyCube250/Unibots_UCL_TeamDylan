# Changes — 2026-06-27

## 2026-06-28 Integration Note

The post-pull integrated ROS2 path now uses `image_sender` for `/img` and
`ball_detector` for YOLO class-32 plus white/orange/yellow colour and shape
filtering. The standalone direct-camera notes below are useful history, but
for current ROS2 testing run `ros2 run image_sender image_sender` and
`ros2 run ball_detector ball_detector`.

## Motor Direction Fix

Tested all four motors individually via SSH over WiFi to the Jetson.
Found that the right-side motors (FR, BR) spin backward when given positive PWM
because they are physically mirrored.

**Fix:** Set default `motorSign` in `four_wheel_encoder_demo.ino` to `{1, -1, 1, -1}`
so positive PWM = forward for all wheels without needing to send `MOTOR_SIGN` at runtime.

### Individual motor test results (PWM +30):
| Motor | Direction | Correct? |
|-------|-----------|----------|
| FL    | Forward   | Yes      |
| FR    | Backward  | Fixed with sign -1 |
| BL    | Forward   | Yes      |
| BR    | Backward  | Fixed with sign -1 |

## STM32 Firmware Updates

- `motorSign` default changed to `{1, -1, 1, -1}` (right-side motors reversed)
- `HARD_PWM_LIMIT` raised from 45 to 80
- `STARTUP_PWM_LIMIT` raised from 15 to 60
- These are defaults in firmware; current STM32 still runs at 45 limit (not reflashed)

## STM32_UART.ino Cleanup

Removed debug overrides that were hardcoding the command to "DROP" and value to 100,
restoring normal command parsing.

## Ball Detection — Complete Rewrite

The ball detector is now a self-contained ROS2 node that:
- Opens the camera directly (no `image_sender` dependency, no `cv_bridge`)
- Runs YOLOv8n (COCO .pt model) on GPU with TensorRT half-precision
- Filters detections by:
  1. **Orange colour** — HSV range H:3-30, S:80-255, V:80-255, requires 20%+ orange pixels
  2. **Ball shape** — aspect ratio 0.6-1.7, not larger than 40% of frame
- Serves a live MJPEG web view at `http://<jetson-ip>:8770`
- Publishes `/ball_detections` (raw boxes, FPS) and `/ball_navigate` (angle + distance to closest cluster)
- Camera feed flipped 180° to correct physical mounting

### Why not the .engine file?
The `.engine` is a custom single-class TensorRT model that detects **everything** as "ball"
(people, TVs, furniture). The `.pt` is standard COCO YOLO with 80 classes — combined with
the orange/shape filter, it reliably detects only orange ping pong balls.

### Key parameters:
| Parameter | Value | Notes |
|-----------|-------|-------|
| Model | `yolo26n.pt` | COCO 80-class, runs on CUDA |
| Confidence | 0.10 | Low threshold, filters do the real work |
| Image size | 640 | Larger = better small ball detection |
| Orange H range | 3–30 | Covers orange to reddish-orange |
| Orange ratio | 20% | Of bounding box pixels must be orange |
| Max box size | 40% of frame | Rejects full-frame false positives |
| FPS | ~10 | On Jetson Orin Nano with .pt model |

## LiDAR Obstacle Detection

- LiDAR driver: `ros2 launch ldlidar_stl_ros2 stl27l.launch.py`
- Obstacle node: `ros2 run lidar lidar_scan`
- Publishes to `/obstacle` with clustered obstacle data (angle, distance, side)
- Self-obstruction observed at ~40° right, 3.2cm (robot chassis) — needs ignore arc

## Development Setup

- SSH to Jetson over WiFi at `192.168.0.237` (network: VM4706224)
- USB hub on Jetson USB-C port carries: LiDAR, camera, laptop ethernet
- STM32 communicates via Jetson's built-in UART `/dev/ttyTHS1` (not through hub)
- LiDAR at `/dev/ttyUSB0` (`/dev/serial/by-id/usb-Silicon_Labs_CP2102_...`)
- Camera (Global Shutter) at `/dev/video0`

---

# How to Run (for teammates)

## Prerequisites
- SSH access to Jetson: `ssh jetson@192.168.0.237`
- Jetson must be on WiFi network `VM4706224`
- USB hub connected with camera + LiDAR

## Starting the system

```bash
# SSH into Jetson
ssh jetson@192.168.0.237

# Set up environment (run ONCE per terminal)
cd /home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan/unibots_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source venv/bin/activate
export LD_LIBRARY_PATH=/home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan/unibots_ws/venv/lib/python3.10/site-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH
```

## Running ball detection
```bash
# Kill any existing camera process
fuser -k /dev/video0 2>/dev/null

# Launch
ros2 run ball_detector ball_detector
```
- Live view: open `http://192.168.0.237:8770` in browser
- Detections published on `/ball_detections` and `/ball_navigate`

## Running LiDAR
```bash
# Terminal 1: LiDAR driver
ros2 launch ldlidar_stl_ros2 stl27l.launch.py

# Terminal 2: Obstacle detector
ros2 run lidar lidar_scan
```
- Obstacles published on `/obstacle` topic
- Monitor: `ros2 topic echo /obstacle`

## Running motors (via STM32 UART)
```bash
# Open serial to STM32
# From Python or miniterm:
#   Port: /dev/ttyTHS1, Baud: 115200
# Commands:
#   ENABLE 1       — arm motors
#   LIMIT 45       — set PWM limit
#   PWM 30 30 30 30  — all wheels forward
#   STOP           — stop all
# Note: watchdog times out after 500ms, re-send PWM every 400ms
```

## Monitoring all topics
```bash
ros2 topic list
ros2 topic echo /ball_detections
ros2 topic echo /ball_navigate
ros2 topic echo /obstacle
```

## Known issues
- LiDAR sees robot body at ~40° right, 3.2cm — ignore arc not yet implemented
- Ball detection runs at ~10 FPS with .pt model (vs 30+ with .engine)
- Camera must be freed (`fuser -k /dev/video0`) if previous process crashed
- NumPy 2.x in venv breaks cv_bridge — that's why ball_detector is self-contained
