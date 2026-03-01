# Phone Teleop

Teleoperation via smartphone motion sensors. Tilt to drive.

## Architecture

```
Phone Browser  ──WebSocket──→  Embedded HTTPS Server  ──→  PhoneTeleopModule
(sensors + button)              (port 8444)                  (delta → velocity)
```

## Running

```bash
dimos run phone-go2-teleop     # Go2
dimos run simple-phone-teleop  # Generic ground robot
```

Open `https://<host-ip>:8444/teleop` on phone. Accept cert, allow sensors, connect, hold to drive.

## Subclassing

| Method | Purpose |
|--------|---------|
| `_handle_engage()` | Customize engage/disengage logic |
| `_publish_msg()` | Change output format |

`self._lock` is already held — don't acquire it in overrides.

## File Structure

```
phone/
├── phone_teleop_module.py   # Base module
├── phone_extensions.py      # SimplePhoneTeleop
├── blueprints.py
└── web/static/index.html    # Mobile web app
```
