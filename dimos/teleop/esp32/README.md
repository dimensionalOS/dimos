# ESP32 gesture teleop

A cheap ESP32 with an ultrasonic sensor becomes a handheld controller. Hand gestures
over the sensor arrive in dimOS as standard `sensor_msgs/Joy` messages on `joystick`,
the same message the keyboard and the Go2's own handheld remote use.

This module is input only. It listens to Bluetooth LE advertisements from one board and
never connects to it, loads a robot connection or sends robot commands. Wire `joystick`
to whatever should react.

## Run

```bash
dimos run teleop-esp32 --esp32gestureteleop.address D4:E9:F4:E2:79:02
```

On Linux the address is the board's MAC. macOS hides MACs and gives each device a
CoreBluetooth UUID instead; pass that UUID.

## Output

Each gesture is a tap: one `Joy` with that gesture's button at 1, then one with all
buttons at 0.

| Field | Meaning |
|---|---|
| `buttons[0]` | SIT gesture |
| `buttons[1]` | STAND gesture |
| `axes[0]` | hand distance from the sensor, metres |

Repeated advertisements of the same gesture publish once. The first packet, a packet
after the board was silent for 2 s, and a firmware reset set a baseline and publish
nothing.

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
