# Jetson Kill Switch and Red/Green LED Wiring

Use Jetson 40-pin physical pin numbers. Do not use BCM numbering.

## Default pins

```text
Momentary button:
  one side -> Jetson physical pin 7
  other side -> Jetson GND, physical pin 30 or 34

Red LED:
  Jetson physical pin 29 -> 220-1000 ohm resistor -> red LED anode
  red LED cathode -> Jetson GND, physical pin 30 or 34

Green LED:
  Jetson physical pin 31 -> 220-1000 ohm resistor -> green LED anode
  green LED cathode -> Jetson GND, physical pin 30 or 34
```

The button uses Jetson internal pull-up. Released is HIGH, pressed is LOW.

## If using an RGB LED

Use only red and green.

For a common-cathode RGB LED:

```text
common cathode -> GND
red anode      -> resistor -> physical pin 29
green anode    -> resistor -> physical pin 31
blue           -> not connected
```

For a common-anode RGB LED:

```text
common anode -> Jetson 3.3V
red cathode  -> resistor -> physical pin 29
green cathode-> resistor -> physical pin 31
blue         -> not connected
```

Then run safety I/O with `led_active_low:=true` or set:

```bash
UNIBOTS_LED_ACTIVE_LOW=1
```

Do not connect any Jetson GPIO pin to 5V. The Jetson GPIO pins are 3.3V logic.

## LED behavior

```text
Boot/service starting: red
Main ready, no decision state yet: solid green
Main running/resumed: blinking green
Button pressed once: red, motion paused, STM32 STOP held
Button pressed again: blinking green, mission resumes without resetting ball count
```

The pause is in-process. It preserves the current mission state while the Jetson
process remains alive. A Jetson reboot still starts a fresh mission.
