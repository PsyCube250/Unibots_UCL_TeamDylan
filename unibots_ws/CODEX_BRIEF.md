# Unibots Jetson Orin Nano Robot — Codex Project Brief

## 1. Project overview

This project is a four-wheel omnidirectional or mecanum mobile robot.

The intended architecture is:

```text
STL-27L LiDAR ──USB──┐
USB camera ──────────┤
IMU ──I2C────────────┤
                     ▼
             Jetson Orin Nano
       perception, ROS 2, planning,
        offline AI and safety logic
                     │
              USB serial or UART
                     ▼
                   STM32
       motor control, four encoders,
       wheel-speed PID and watchdog
                     │
             motor driver boards
                     ▼
                four motors
```

The Jetson must not directly count all four quadrature encoders or directly drive the motors.

The STM32 should handle:

* Four motor outputs
* Eight encoder signals: four encoders × A/B channels
* Low-level wheel-speed control
* Motor direction
* Communication watchdog
* Emergency stop if Jetson communication is lost

The Jetson should handle:

* STL-27L LiDAR
* IMU
* Camera and future YOLO detection
* Obstacle detection
* High-level random wandering
* Offline AI recommendations
* Sending target wheel speeds to STM32

---

## 2. Immediate objective

The immediate objective is to produce the simplest safe working prototype in stages:

1. Detect all connected devices.
2. Read the STL-27L LiDAR reliably.
3. Read the IMU reliably.
4. Communicate with the STM32.
5. Test each motor with the wheels lifted and propulsive power limited.
6. Read all four encoders on the STM32.
7. Implement low-speed deterministic random wandering.
8. Stop or turn away when the LiDAR detects an obstacle.
9. Add an offline language model only as a low-frequency high-level advisor.
10. Never allow the language model to bypass the deterministic safety layer.

Do not attempt SLAM or persistent mapping now.

The previous accumulated mapping experiments did not work reliably. Use only current LiDAR scan data.

---

## 3. Safety rules

These rules have the highest priority:

* Never move motors before the user explicitly approves a bench test.
* During the first tests, lift all wheels off the ground.
* Start with very low motor commands.
* Always provide an immediate stop command.
* Motors must use a separate battery or suitable motor power supply.
* Do not power motors from the Jetson.
* Jetson, STM32 and motor drivers must share signal ground where required.
* If LiDAR data becomes stale, stop.
* If STM32 communication is lost, stop.
* If the control program crashes, stop.
* If an AI model times out or returns invalid output, stop.
* Back up every existing file before modifying it.
* Do not delete or overwrite known-working code without creating a timestamped backup.
* Do not run destructive disk, bootloader, pinmux or firmware commands without explicit user approval.
* `sudo` may be used for package installation, serial permissions and service inspection, but not for destructive changes without approval.

---

## 4. Remote connection

The user normally works from a Mac without connecting a display to the Jetson.

The preferred connection is USB device-mode networking:

```text
Mac USB-C <-> Jetson Orin Nano USB-C data port
```

The Jetson still needs its normal external power supply.

Typical connection from the Mac:

```bash
ping -c 4 192.168.55.1
ssh jetson@192.168.55.1
```

Never request that the user paste their password into project files or chat. Password entry must remain inside the local terminal prompt.

The expected Jetson username is usually:

```text
jetson
```

Expected project repository:

```text
/home/jetson/Unibots_UCL_TeamDylan
```

GitHub source:

```text
https://github.com/PsyCube250/Unibots_UCL_TeamDylan
```

Before changing anything, inspect the actual location:

```bash
pwd
find /home/jetson -maxdepth 3 -type d -name "Unibots_UCL_TeamDylan" 2>/dev/null
```

---

## 5. Current hardware allocation

### Jetson Orin Nano

| Device        | Preferred interface                           |
| ------------- | --------------------------------------------- |
| STL-27L LiDAR | USB-to-TTL connected to Jetson USB            |
| Camera        | Jetson USB                                    |
| IMU           | I2C0 on the 40-pin header                     |
| STM32         | Prefer USB for power and serial communication |
| Motor drivers | Connected to STM32, not Jetson                |
| Four encoders | Connected to STM32, not Jetson                |

The Jetson should have enough USB ports for:

```text
USB 1: LiDAR USB-to-TTL
USB 2: camera
USB 3: STM32
USB 4: spare
```

The actual device names must be detected rather than assumed.

Useful detection commands:

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>/dev/null
ls -l /dev/video* 2>/dev/null
i2cdetect -l
dmesg | tail -100
```

Create stable `/dev/serial/by-id/` detection where possible:

```bash
ls -l /dev/serial/by-id/ 2>/dev/null
```

Do not permanently hard-code `/dev/ttyUSB0` if a stable `by-id` path is available.

---

## 6. STL-27L LiDAR

### Electrical connection

Known working colour connection from the existing cable:

```text
Red wire    -> USB-to-TTL RXD
Black wire  -> USB-to-TTL GND
White wire  -> USB-to-TTL GND
Yellow wire -> USB-to-TTL +5V
```

USB-to-TTL pins:

```text
3V3 -> not connected
TXD -> not connected
RXD -> red LiDAR wire
GND -> black and white LiDAR wires
+5V -> yellow LiDAR wire
```

Functional interpretation:

```text
LiDAR TX   -> USB-to-TTL RXD
LiDAR PWM  -> GND when external speed control is unused
LiDAR GND  -> GND
LiDAR P5V  -> +5V
```

### Serial parameters

```text
Expected port: /dev/ttyUSB0
Baud rate: 921600
Data format: 8N1
```

### Known frame format

```text
Header: 0x54
VerLen: 0x2C
Frame length: 47 bytes
Points per frame: 12
```

Each point contains:

```text
distance: uint16 little-endian, millimetres
confidence: uint8
```

Angles:

```text
start_angle = uint16 little-endian / 100.0
end_angle   = uint16 little-endian / 100.0
```

Current validity filter:

```text
30 mm <= distance <= 25000 mm
confidence >= 30
```

The first goal is a reliable real-time scan or sector summary, not a stored map.

Required output should include at least:

```text
front minimum distance
front-left minimum distance
front-right minimum distance
left minimum distance
right minimum distance
scan age
valid point count
```

Use a short rolling window only. Do not stack previous sessions.

If no valid scan has arrived for more than approximately 0.5 seconds, mark LiDAR as stale and request STOP.

---

## 7. IMU

The IMU is connected using I2C0, not the other I2C bus.

Preferred physical connections:

```text
IMU SDA -> Jetson physical Pin 27
IMU SCL -> Jetson physical Pin 28
IMU VCC -> Jetson 3.3V, such as Pin 1 or Pin 17
IMU GND -> Jetson GND, such as Pin 25 or Pin 30
```

Do not assume that hardware I2C0 appears as `/dev/i2c-0`.

First inspect:

```bash
i2cdetect -l
```

Then scan candidate buses carefully:

```bash
sudo i2cdetect -y BUS_NUMBER
```

Possible IMU addresses include:

```text
MPU6050: 0x68 or 0x69
LSM6DSOX: 0x6A or 0x6B
```

Inspect the existing repository to determine the actual IMU model and existing driver before writing a replacement.

Do not run aggressive I2C probing against unknown buses containing critical devices.

---

## 8. STM32 connection and power

Preferred arrangement:

```text
Jetson USB-A -> STM32 USB port
```

This provides:

* STM32 logic power
* USB serial communication
* No additional Jetson UART header usage

Typical Linux device:

```text
/dev/ttyACM0
```

The LiDAR will normally remain on:

```text
/dev/ttyUSB0
```

Do not power the STM32 simultaneously from Jetson USB and a separate 5V header unless the STM32 board power circuitry is confirmed safe for dual-source power.

Do not power motor drivers or motors from the Jetson USB supply.

If the existing system must use hardware UART instead, the historical configuration is:

```text
Jetson /dev/ttyTHS1
Baud: 115200
Jetson Pin 8 TX  -> STM32 RX
Jetson Pin 10 RX <- STM32 TX
Jetson GND       -> STM32 GND
```

Do not use USB serial and the hardware UART protocol simultaneously unless the firmware explicitly supports both.

---

## 9. Existing motor communication attempts

The repository may contain two incompatible approaches.

### Text protocol

Examples:

```text
FORWARD,100
TURN,25
STOP
```

Known issue:

A previous STM32 text-protocol test contained temporary lines similar to:

```cpp
action = "DROP";
valueStr = 100;
```

These force all commands to become a DROP command and must not remain in production code.

Also, the text implementation may only use the sign of the TURN value rather than the magnitude. Do not claim angle-accurate turning unless encoder or IMU feedback is actually implemented.

### Binary four-wheel protocol

Preferred low-level format from the existing project:

```text
Header: 0xFF 0xFE
Four wheel speeds
Checksum
```

Expected ROS topic:

```text
/motor_speeds
std_msgs/msg/Int32MultiArray
```

Example:

```text
[data: motor_1, motor_2, motor_3, motor_4]
```

The historical range was approximately:

```text
-100 to +100
```

Start bench tests around:

```text
±10 to ±15
```

Do not use values such as 180 until the complete scaling path is verified.

Prefer the binary protocol if it has:

* Checksum validation
* Four independent wheel speeds
* Communication watchdog
* Automatic stop after around 500 ms without commands

Codex must inspect the actual current firmware before deciding which protocol is active.

---

## 10. Four motors and encoders

Four quadrature encoders require:

```text
4 encoders × 2 channels = 8 digital inputs
```

Do not connect these eight signals directly to the Jetson.

The STM32 should read the encoders, preferably through hardware timer encoder mode where the selected STM32 supports it.

The STM32 should return telemetry to the Jetson:

```text
encoder tick count for each wheel
measured wheel speed for each wheel
motor command for each wheel
fault flags
watchdog state
battery or driver fault if available
timestamp or sequence number
```

Before assigning pins, identify the exact STM32 board and microcontroller model.

Do not invent a pin mapping without checking:

* Existing firmware
* Board schematic
* Timer alternate functions
* Pins already used by motor drivers
* UART or USB functions
* Debug pins
* Voltage levels

---

## 11. Deterministic obstacle avoidance

The first moving version must not depend on an LLM.

Implement a deterministic safety controller using current LiDAR scan sectors.

Recommended initial behaviour:

```text
FORWARD:
    drive slowly when front clearance is safe

STOP:
    immediate zero command if front obstacle is too close

TURN_LEFT:
    rotate slowly left when the right side is more blocked

TURN_RIGHT:
    rotate slowly right when the left side is more blocked

RANDOM_TURN:
    occasionally select a safe turn direction when open space is available
```

Initial conservative thresholds:

```text
Emergency stop: front distance < 0.35 to 0.40 m
Caution zone: front distance < 0.55 to 0.60 m
Resume forward: front distance > 0.60 m
LiDAR stale timeout: approximately 0.5 s
```

Use hysteresis so the robot does not oscillate rapidly around one threshold.

Random wandering may choose a new direction every few seconds, but only when the safety layer approves it.

Motor command priority:

```text
1. Emergency stop
2. Sensor stale or communication failure stop
3. Collision-avoidance turn
4. Normal wandering action
5. Offline AI suggestion
```

---

## 12. Offline AI model

The offline model must not directly send wheel speeds.

The model may only make low-frequency suggestions using a strict action set:

```text
FORWARD
TURN_LEFT
TURN_RIGHT
STOP
```

Suggested model input:

```json
{
  "front_m": 1.2,
  "front_left_m": 0.8,
  "front_right_m": 1.4,
  "left_m": 0.6,
  "right_m": 1.8,
  "current_action": "FORWARD",
  "action_duration_s": 2.1,
  "lidar_stale": false
}
```

Required model output:

```json
{
  "action": "TURN_RIGHT",
  "reason": "More free space is available on the right."
}
```

Requirements:

* Validate output against a schema.
* Reject unknown actions.
* Apply a short timeout.
* On timeout, malformed JSON or model failure, use STOP or the deterministic controller.
* Run the model at approximately 0.5–1 Hz, not at motor-control frequency.
* The deterministic safety controller must override the model.
* First run the model in shadow mode: print suggestions but do not execute them.

Do not install a large model until available RAM, storage, JetPack version and GPU resources have been checked.

---

## 13. Required repository structure

Preserve the original repository, but gradually organise the safe implementation as:

```text
Unibots_UCL_TeamDylan/
├── CODEX_BRIEF.md
├── README.md
├── scripts/
│   ├── hardware_inventory.sh
│   ├── test_lidar.py
│   ├── test_imu.py
│   ├── test_stm32_serial.py
│   └── emergency_stop.py
├── robot_control/
│   ├── lidar_stl27l.py
│   ├── imu_reader.py
│   ├── stm32_link.py
│   ├── safety_controller.py
│   ├── wander_controller.py
│   ├── ai_advisor.py
│   └── main.py
├── stm32/
│   ├── current_firmware_backup/
│   └── proposed_firmware/
├── config/
│   └── robot.yaml
├── logs/
└── backups/
```

Do not refactor everything at once.

Keep a simple one-file diagnostic script where useful.

---

## 14. First actions Codex must perform

Before editing code:

1. Confirm the operating system and JetPack version.

```bash
cat /etc/os-release
uname -a
dpkg-query --show nvidia-l4t-core 2>/dev/null || true
```

2. Find the repository.

```bash
find /home/jetson -maxdepth 4 -type d -name "Unibots_UCL_TeamDylan" 2>/dev/null
```

3. Inspect Git state.

```bash
cd /home/jetson/Unibots_UCL_TeamDylan
git status
git branch --show-current
git log --oneline -10
```

4. Create a backup branch before modifications.

```bash
git switch -c codex-safe-integration-$(date +%Y%m%d-%H%M%S)
```

If uncommitted work exists, do not discard it. Save a patch and copy modified files to `backups/`.

5. Inventory relevant source files.

```bash
find . -maxdepth 5 -type f \
  \( -name "*.py" -o -name "*.ino" -o -name "*.cpp" -o -name "*.h" \
  -o -name "*.yaml" -o -name "*.launch.py" -o -name "package.xml" \) \
  | sort
```

6. Search for serial ports, GPIO pins, motor commands and safety logic.

```bash
grep -RInE \
"/dev/tty|ttyUSB|ttyACM|ttyTHS|MSP|motor|encoder|LiDAR|lidar|IMU|I2C|DROP|FORWARD|TURN|STOP|watchdog" \
. --exclude-dir=.git
```

7. Inventory connected hardware without changing configuration.

```bash
lsusb
ls -l /dev/serial/by-id/ 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>/dev/null || true
ls -l /dev/video* 2>/dev/null || true
i2cdetect -l
```

8. Report findings before implementing motor movement.

The report must clearly identify:

* Active LiDAR device
* Active STM32 serial device
* Current IMU model and I2C bus
* Current motor protocol
* Current STM32 board or MCU model
* Current motor pin mapping
* Whether encoder code exists
* Conflicting serial protocols
* Dangerous temporary test code
* Missing watchdogs
* Proposed minimum-change implementation plan

---

## 15. Modification policy

Codex may autonomously:

* Read all project files
* Run non-destructive inspection commands
* Create backups
* Create a new Git branch
* Install small required packages after explaining them
* Add diagnostic scripts
* Improve error handling
* Add configuration files
* Add logging
* Add unit tests
* Add dry-run and simulation modes

Codex must request explicit confirmation before:

* Moving motors
* Flashing STM32 firmware
* Changing Jetson pinmux
* Editing boot configuration
* Changing system networking
* Deleting files
* Force-pushing Git
* Reformatting partitions
* Modifying motor power wiring
* Running autonomous movement on the floor

---

## 16. Definition of the first successful milestone

The first milestone is complete only when all of the following work:

```text
[ ] LiDAR is detected and produces current sector distances.
[ ] IMU is detected and produces plausible readings.
[ ] STM32 serial communication is stable.
[ ] A STOP command can always be sent.
[ ] STM32 watchdog stops motors on lost communication.
[ ] Four motors have been tested individually at low speed with wheels lifted.
[ ] Motor direction mapping is documented.
[ ] Four encoders are read by STM32.
[ ] Encoder directions are documented.
[ ] Deterministic safety controller runs in dry-run mode.
[ ] Random wandering runs in dry-run mode.
[ ] Offline AI runs only in shadow mode.
[ ] No persistent mapping or SLAM is enabled.
```

The first real floor test must use very low speed and a large clear area, with a human ready to cut motor power immediately.
