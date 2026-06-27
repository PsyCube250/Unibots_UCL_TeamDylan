# Changes — 2026-06-27

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

## STM32_UART.ino Cleanup

Removed debug overrides that were hardcoding the command to "DROP" and value to 100,
restoring normal command parsing.

## Development Setup

- Confirmed SSH to Jetson works over WiFi at `192.168.0.237` (network: VM4706224)
- USB hub connected to Jetson USB-C port carries: LiDAR, camera, laptop ethernet
- STM32 communicates via Jetson's built-in UART `/dev/ttyTHS1` (not through hub)
- LiDAR at `/dev/ttyUSB0` (`/dev/serial/by-id/usb-Silicon_Labs_CP2102_...`)
- Camera (Global Shutter) at `/dev/video0`

## LiDAR Observation

Front-facing LiDAR scan shows self-obstruction from robot body at angles -10 to +45 degrees
(readings of 3-7cm). These angles will need an ignore arc in obstacle detection code.
Left side reads clearly at 55-74cm.
