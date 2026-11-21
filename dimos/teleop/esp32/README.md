# ESP32 gesture teleop

A cheap ESP32 with an ultrasonic sensor becomes a handheld controller. Hand gestures
over the sensor arrive in dimOS as standard `sensor_msgs/Joy` messages on `gesture_joy`,
the same message type the keyboard teleop and the Go2's own handheld remote publish.

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
| `buttons[0]` | SIT gesture |
| `buttons[1]` | STAND gesture |
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
| 2 | posture | 0 reset, 1 SIT, 2 STAND |
| 3 | distance_cm | 0-255 |
| 4 | gesture count | 0-255, increments once per gesture, wraps |

Reference firmware: [`esp32/go2_leash`](https://github.com/StewartalsopIII/dimos-esp32-go2/tree/main/esp32/go2_leash).
