# Quest Teleop

Teleoperation via Meta Quest 3 VR controllers. Dual-hand tracking with WebXR.

## Architecture

```
Quest Browser  ──WebSocket──→  Embedded HTTPS Server  ──→  QuestTeleopModule
(WebXR poses + Joy)             (port 8443)                  (delta → PoseStamped)
```

## Running

```bash
dimos run teleop-quest-rerun    # Quest teleop + Rerun viz
dimos run teleop-quest-xarm7   # XArm7
dimos run teleop-quest-piper   # Piper
dimos run teleop-quest-dual    # Dual arm
```

Open `https://<host-ip>:8443/teleop` on Quest browser. Accept cert, tap Connect.

## Subclassing

| Method | Purpose |
|--------|---------|
| `_handle_engage()` | Customize engage/disengage logic |
| `_should_publish()` | Add conditions for publishing |
| `_get_output_pose()` | Customize pose computation |
| `_publish_msg()` | Change output format |

`self._lock` is already held — don't acquire it in overrides.

## Joy Message Format

**Axes**: thumbstick X, thumbstick Y, trigger (analog), grip (analog)

**Buttons**: trigger, grip, touchpad, thumbstick, X/A, Y/B, menu

## File Structure

```
quest/
├── quest_teleop_module.py   # Base module
├── quest_extensions.py      # ArmTeleop, TwistTeleop
├── quest_types.py           # QuestControllerState, Buttons
├── blueprints.py
└── web/static/index.html    # WebXR client
```
