# Teleop Stack

Teleoperation modules for DimOS. Supports Meta Quest 3 VR controllers and phone motion sensors.

## Architecture

```
Quest/Phone Browser
    │
    │  LCM-encoded binary via WebSocket
    ▼
Embedded FastAPI Server (HTTPS)
    │
    │  Fingerprint-based message dispatch
    ▼
TeleopModule (Quest or Phone)
    │  Frame transforms + pose/twist computation
    ▼
PoseStamped / TwistStamped / Buttons outputs
```

Each teleop module embeds a `RobotWebInterface` (FastAPI + uvicorn) that:
- Serves the teleop web app at `/teleop`
- Accepts WebSocket connections at `/ws`
- Handles SSL certificate generation for HTTPS (required by mobile sensor APIs)

## Modules

### QuestTeleopModule
Base Quest teleop module. Gets controller data via WebSocket, computes output poses, and publishes them. Default engage: hold primary button (X/A). Subclass to customize.

### ArmTeleopModule
Toggle-based engage — press primary button once to engage, press again to disengage.

### TwistTeleopModule
Outputs TwistStamped (linear + angular velocity) instead of PoseStamped.

### PhoneTeleopModule
Base phone teleop module. Receives orientation + gyro data from phone motion sensors, computes velocity commands from orientation deltas.

### SimplePhoneTeleop
Filters to mobile-base axes (linear.x, linear.y, angular.z) and publishes as `Twist`.

## Subclassing

`QuestTeleopModule` is designed for extension. Override these methods:

| Method | Purpose |
|--------|---------|
| `_handle_engage()` | Customize engage/disengage logic |
| `_should_publish()` | Add conditions for when to publish |
| `_get_output_pose()` | Customize pose computation |
| `_publish_msg()` | Change output format |
| `_publish_button_state()` | Change button output |

### Rules for subclasses

- **Do not acquire `self._lock` in overrides.** The control loop already holds it.
  Access `self._controllers`, `self._current_poses`, `self._is_engaged`, etc. directly.
- **Keep overrides fast** — they run inside the control loop at `control_loop_hz`.

## File Structure

```
teleop/
├── quest/
│   ├── quest_teleop_module.py   # Base Quest teleop module
│   ├── quest_extensions.py      # ArmTeleop, TwistTeleop
│   ├── quest_types.py           # QuestControllerState, Buttons
│   └── web/
│       └── static/index.html    # WebXR client
├── phone/
│   ├── phone_teleop_module.py   # Base Phone teleop module
│   ├── phone_extensions.py      # SimplePhoneTeleop
│   ├── blueprints.py            # Pre-wired configurations
│   └── web/
│       └── static/index.html    # Mobile sensor web app
├── utils/
│   ├── teleop_transforms.py     # WebXR → robot frame math
└── blueprints.py                # Module blueprints for easy instantiation
```

## Quick Start

```bash
dimos run teleop-quest-rerun     # Quest teleop + Rerun viz
dimos run teleop-phone-go2      # Phone → Go2
```

Open `https://<host-ip>:<port>/teleop` on device. Accept the self-signed certificate.
- Quest: port 8443
- Phone: port 8444
