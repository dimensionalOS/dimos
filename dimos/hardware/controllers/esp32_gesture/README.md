# ESP32 gesture controller

Any ESP32 with a distance sensor becomes a hands-free controller. Hand gestures
over the sensor arrive in dimOS as standard `sensor_msgs/Joy` messages on `gesture_joy`,
the same message type the keyboard teleop publishes.

This module is input only. It listens to Bluetooth LE advertisements from one board and
never connects to it or pairs with it, loads a robot connection or sends robot commands.
Wire `gesture_joy` to whatever should react.

## Run

```bash
dimos run teleop-esp32 --esp32gestureteleop.address AA:BB:CC:DD:EE:FF
```

The address is the board's Bluetooth MAC. Tested on Linux (BlueZ, Raspberry Pi 5).

## Output

Each gesture is a tap: one `Joy` with that gesture's button at 1, then, `press_hold_s`
later (default 0.1 s), one with all buttons at 0. A new gesture or `stop()` sends a
pending release first, so a button is never left pressed.

| Field | Meaning |
|---|---|
| `buttons[0]` | gesture A |
| `buttons[1]` | gesture B |
| `axes[0]` | hand distance, scaled 0..1 over 0..1 m (1 also means no echo) |
| `frame_id` | `esp32_gesture` |

Repeated advertisements of the same gesture publish once. The first packet, a packet
after the board was silent for 2 s, and a firmware reset set a baseline and publish
nothing; a gesture made during such a gap is logged as a warning, not published. If the
Bluetooth scanner fails, the module logs it and retries with backoff.

## Board protocol

Manufacturer data, company id `0xFFFF`, then five bytes:

| Byte | Field | Values |
|---|---|---|
| 0 | state | 0 or 1 |
| 1 | sequence | 0-255, increments every advertisement |
| 2 | gesture_id | 0 reset, 1 A, 2 B (the board alternates A, B, A...) |
| 3 | distance_cm | 0-255 |
| 4 | gesture count | 0-255, increments once per gesture, wraps |

## Firmware

The reference firmware is [`firmware/esp32_gesture`](firmware/esp32_gesture/esp32_gesture.ino). It has no
robot-specific code. Every pin, timing and distance threshold is a top-level constant that can also be
set from the build command, so it runs on any ESP32 and any wiring:

```bash
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_gesture
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 firmware/esp32_gesture
# other pins or chip, e.g.:
arduino-cli compile --fqbn esp32:esp32:esp32c3 \
  --build-property "build.extra_flags=-DTRIG_PIN=4 -DECHO_PIN=5 -DLED_PIN=-1" firmware/esp32_gesture
```

The default sensor is an HC-SR04-style ultrasonic module (VCC to 3V3, TRIG to GPIO5, ECHO to GPIO18).
For a different distance sensor, replace `readCm()`. The board's MAC is printed on the serial port at boot.

What A and B do is up to whatever subscribes to `gesture_joy`; a robot blueprint can map them
to its own commands (for example sit and stand).
