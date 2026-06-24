# Four-Wheel Encoder Demo Wiring

This wiring is for the proposed demo firmware at:

`stm32/proposed_firmware/four_wheel_encoder_demo/four_wheel_encoder_demo.ino`

It is a starter map, not a final PCB schematic. Verify the exact STM32 board,
pin labels, voltage tolerance, motor driver module, and encoder output type
before powering motors.

## Safety

- Do not power motors from the Jetson or the STM32 USB port.
- Use a separate motor battery or current-limited bench supply for motor VM.
- Share signal ground: Jetson/STM32 ground, motor driver ground, and motor
  battery negative must have a common reference where required.
- Keep wheels lifted for the first motor tests.
- The firmware boots with motor output disabled and PWM limited to +/-15.
- First encoder tests should be done with motor power disconnected: turn each
  wheel by hand and watch encoder counts.

## Current Used STM32 Pins

From the current motor-shield/Jetson setup, these pins are already occupied:

```text
Motor shield / motor outputs:
PB1 PB0 PA7 PA6 PA3 PA2 PA1 PA0 PB6 PB7 PB8 PB9 PB3

Jetson Nano UART:
PA9 PA10
```

Do not connect encoders to those pins.

## Jetson to STM32 Link

Your current wiring uses STM32 USART1. The proposed firmware now sends commands
and telemetry on this UART, not USB CDC:

| Jetson Nano | STM32 |
| --- | --- |
| Pin 8 TX | PA10 RX |
| Pin 10 RX | PA9 TX |
| GND | GND |

The Jetson UART is 3.3 V logic. Do not put 5 V UART signals into the Jetson.

On Jetson this is normally:

```text
/dev/ttyTHS1
115200 baud
```

## Motor Driver Wiring

The firmware assumes DRV8833-style IN1/IN2 motor control pins and matches the
current motor-shield pin usage you reported.

Known actual shield wiring:

| Shield | Channel | STM32 input pins | Driver output | Motor wires |
| --- | --- | --- | --- | --- |
| 1 | IN1/IN2 | PA0/PA1 | OUT1/OUT2 | Front-left, brown then blue |
| 1 | IN3/IN4 | PA2/PA3 | OUT3/OUT4 | Front-right, blue then brown, reversed vs front-left |

The front-right reversed output order may already compensate for the opposite
side of the drivetrain. Confirm by lifted-wheel low-PWM test before changing
`MOTOR_SIGN`.

| Wheel | STM32 IN1 | STM32 IN2 | Driver output |
| --- | --- | --- | --- |
| Front-left | PA0 | PA1 | FL motor |
| Front-right | PA2 | PA3 | FR motor |
| Back-left | PA6 | PA7 | BL motor |
| Back-right | PB0 | PB1 | BR motor |

The firmware also drives `PB3` high as a shield-enable style pin, matching the
old `STM32_UART.ino` pattern. If your shield uses `PB3` for something else,
change `SHIELD_ENABLE_PIN` before flashing.

Driver power:

| Driver pin | Connect to |
| --- | --- |
| VM / VIN motor power | Motor battery positive |
| GND | Motor battery negative and STM32 GND |
| SLEEP / STBY, if present | Logic high according to the driver module |
| OUT1/OUT2 | Motor terminals |

If a wheel spins the wrong way during a lifted-wheel low-speed test, either swap
that motor's two output wires or set the matching `MOTOR_SIGN` entry to `-1`.

## Encoder Wiring

Use 3.3 V encoder signals if possible. If your encoder board outputs 5 V logic,
use level shifting or a divider before the STM32 input pins unless your exact
STM32 pin is confirmed 5 V tolerant.

The firmware enables `INPUT_PULLUP` on every encoder input. If your encoder
board already has pullups to 5 V, remove those pullups or level-shift the
signals.

| Wheel | Encoder A | Encoder B | Encoder VCC | Encoder GND |
| --- | --- | --- | --- | --- |
| Front-left | PB10 | PB11 | STM32 3V3 | STM32 GND |
| Front-right | PB12 | PB13 | STM32 3V3 | STM32 GND |
| Back-left | PB14 | PB15 | STM32 3V3 | STM32 GND |
| Back-right | PA4 | PA5 | STM32 3V3 | STM32 GND |

These pins are chosen because they avoid the motor-shield pins and PA9/PA10.
Confirm they are broken out on your exact STM32 board and not used by the
shield before wiring.

If a wheel count goes negative when you turn the wheel in the intended forward
direction, either swap its A/B wires or set the matching `ENCODER_SIGN` entry
to `-1`.

## Bring-Up Order

1. Motor battery disconnected.
2. Connect STM32 PA9/PA10 to Jetson UART and connect common ground.
3. Flash the proposed firmware only after explicit approval.
4. Run the Jetson helper in telemetry-only mode:
   `python3 robot_control/stm32_encoder_demo.py --port /dev/ttyTHS1 --show-raw`
5. Turn each wheel by hand and confirm only the matching encoder count changes.
6. Document encoder direction for each wheel.
7. Only after approval, connect motor power with wheels lifted.
8. Start with one wheel at low PWM:
   `python3 robot_control/stm32_encoder_demo.py --port /dev/ttyTHS1 --bench-test --enable-motors --mode pwm --values 10 0 0 0 --duration 2 --send-stop-on-exit`
9. Repeat one wheel at a time and document motor direction.
10. Try closed-loop velocity mode only after encoder signs and motor signs are
    correct.

## Why Encoder Feedback Is Better Than Pulsed PWM Alone

PWM sets electrical effort, not actual wheel speed. Real speed changes with
battery voltage, floor friction, load, wheel contact, driver heating, and motor
variation. A pulsed PWM scheme can look consistent on a bench but drift badly
on the floor.

Encoder feedback measures what the wheel actually did. That allows the STM32 to
close the loop around ticks-per-second, detect stalled wheels, compare commanded
speed against measured speed, and eventually report odometry or fault telemetry
to the Jetson.
