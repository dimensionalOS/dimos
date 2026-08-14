# Teleop Stack

Teleoperation tools and modules for DimOS. Supports a headless terminal, Meta Quest 3 VR
controllers, and phone motion sensors.

## Terminal cmd_vel teleop

From a second terminal on a machine participating in the running stack's pub/sub network:

```bash
dimos teleop
```

This is suitable for an SSH session on a headless robot. It publishes typed `Twist` messages
to `cmd_vel` and does not open a robot connection. Press Enter to engage, use W/S to move,
A/D to turn, Q/E to strafe, Space to stop and disengage, and Escape to stop and exit. Movement
expires 200 ms after the last movement key event, so holding a key relies on terminal key
repeat and a lost terminal does not leave a persistent command publisher.

Speeds and the logical topic are configurable:

```bash
dimos teleop --linear-speed 0.10 --angular-speed 0.25 --topic cmd_vel
```

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
├── terminal.py                   # SSH-friendly Twist publisher with a dead-man timeout
├── quest/
│   ├── quest_teleop_module.py   # Base Quest teleop module (local WebSocket)
│   ├── quest_extensions.py      # ArmTeleop, TwistTeleop
│   ├── quest_types.py           # QuestControllerState, Buttons
│   └── web/
│       └── static/index.html    # WebXR client
├── hosted/                      # Hosted teleop (transport-swap, per-concern modules)
│   ├── go2_command.py           # Go2CommandModule: command/E-STOP dispatch + drive guard
│   ├── arm_command.py           # ArmCommandModule: VR poses / EE-twist → coordinator tasks
│   ├── command_executor.py      # SerializedCommandExecutor: serialized cmds + safety fence
│   ├── camera_mux.py            # CameraMuxModule: N cameras → one composited video track
│   ├── map_compress.py          # MapCompressModule: costmap/odom → minimap datachannel
│   ├── hosted_stats.py          # HostedStatsModule: telemetry + acks + cmd-link stats
│   ├── robot_type.py            # RobotType: broker config → operator view auto-select
│   ├── README.md                # Broker session / aiortc + Cloudflare WebRTC internals
│   └── blueprints/              # cloudflare.py (Go2 transport + multicam, xArm6/7)
├── phone/
│   ├── phone_teleop_module.py   # Base Phone teleop module
│   ├── phone_extensions.py      # SimplePhoneTeleop
│   ├── blueprints.py            # Pre-wired configurations
│   └── web/
│       └── static/index.html    # Mobile sensor web app
├── utils/
│   ├── teleop_transforms.py     # WebXR → robot frame math
│   ├── recorder.py              # Generic SQLite recorder (writes .db + report_<ts>.json on stop)
│   ├── report.py                # generate_report(db_path) — read .db, emit report_<ts>.json
│   ├── stream_stats.py          # LiveStreamStats + pcts (shared latency/jitter math)
│   └── video_stats.py           # VideoStats msg + loss_pct (operator-reported video health)
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
