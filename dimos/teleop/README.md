# Teleop Stack

Teleoperation modules for DimOS. Currently supports Meta Quest 3 VR controllers.

## Architecture

```
Quest Browser (WebXR)
    │
    │  PoseStamped + Joy via WebSocket
    ▼
Deno Bridge (teleop_server.ts)
    │
    │  LCM topics
    ▼
QuestTeleopModule
    │  WebXR → robot frame transform
    │  Pose computation + button state packing
    ▼
PoseStamped / TwistStamped / QuestButtons outputs
```

## Modules

### QuestTeleopModule
Base teleop module. Gets controller data, computes output poses, and publishes them. Default engage: hold primary button (X/A). Subclass to customize.

### ArmTeleopModule
Toggle-based engage — press primary button once to engage, press again to disengage.

### TwistTeleopModule
Outputs TwistStamped (linear + angular velocity) instead of PoseStamped.

### VisualizingTeleopModule
Adds Rerun visualization for debugging. Extends ArmTeleopModule (toggle engage).

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
├── base/
│   └── teleop_protocol.py      # TeleopProtocol interface
├── quest/
│   ├── quest_teleop_module.py   # Base Quest teleop module
│   ├── quest_extensions.py      # ArmTeleop, TwistTeleop, VisualizingTeleop
│   ├── quest_types.py           # QuestControllerState, QuestButtons
│   └── web/                     # Deno bridge + WebXR client
│       ├── teleop_server.ts
│       └── static/index.html
├── utils/
│   ├── teleop_transforms.py     # WebXR → robot frame math
│   └── teleop_visualization.py  # Rerun visualization helpers
└── blueprints.py                # Module blueprints for easy instantiation
```

## Quick Start

See [Quest Web README](quest/web/README.md) for running the Deno bridge and connecting the Quest headset.
