# Phone Teleop

Teleoperation via smartphone motion sensors. Tilt to drive.

## Architecture

```
Phone Browser  ──WebSocket──→  Embedded HTTPS Server  ──→  PhoneTeleopModule
(sensors + button)              (port 8444)                  (delta → velocity)
```

## Running

```bash
dimos run teleop-phone-go2     # Go2
dimos run teleop-phone-go2-agentic  # Go2 agentic stack with phone override
dimos run teleop-phone         # Generic ground robot
```

Open `https://<host-ip>:8444/teleop` on phone. Accept cert, allow sensors, connect, hold to drive.

## Agentic Go2 Override

`teleop-phone-go2-agentic` adds the phone controller to the full agentic Go2
stack. Phone commands are remapped into `MovementManager.tele_cmd_vel`, so
holding the phone teleop button cancels active navigation goals and temporarily
takes over velocity control. Release the button to stop sending manual commands
and allow autonomous navigation commands to resume after the movement manager's
teleop cooldown.

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
