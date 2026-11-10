# Teleop Stack

Teleoperation modules for DimOS. Supports browser-based WebXR devices, including
Meta Quest and PICO headsets, plus phone motion sensors.

## Architecture

```
WebXR/Phone Browser
    │
    │  LCM-encoded binary via WebSocket
    ▼
Embedded FastAPI Server (HTTPS)
    │
    │  Fingerprint-based message dispatch
    ▼
TeleopModule (WebXR or Phone)
    │  Frame transforms + pose/twist computation
    ▼
PoseStamped / TwistStamped / Buttons outputs
```

Each teleop module embeds a `RobotWebInterface` (FastAPI + uvicorn) that:
- Serves the teleop web app at `/teleop`
- Accepts WebSocket connections at `/ws`
- Handles SSL certificate generation for HTTPS (required by mobile sensor APIs)

## Modules

### WebXRTeleopModule
Base WebXR teleop module. Gets controller data via WebSocket, computes output poses, and publishes them. Default engage: hold primary button (X/A). Subclass to customize.

### ArmTeleopModule
Publishes controller poses and buttons to `TeleopIKTask`. Hold the middle-finger
grip to engage arm control; release it to disengage. Bimanual control requires
both grips held, and releasing either grip disengages both arms. The
index-finger trigger controls gripper opening while that hand's grip is held.

### TwistTeleopModule
Outputs TwistStamped (linear + angular velocity) instead of PoseStamped.

### PhoneTeleopModule
Base phone teleop module. Receives orientation + gyro data from phone motion sensors, computes velocity commands from orientation deltas.

### SimplePhoneTeleop
Filters to mobile-base axes (linear.x, linear.y, angular.z) and publishes as `Twist`.

### LeaderTeleopModule
Reads Feetech leader arms over serial and publishes follower joint targets plus
per-side gripper openings. The robot is config, not code: a `SideMapping` per
side names which leader motor drives which follower joint, with limits, scale
and offset. `LeaderCoordinator` adds the `leader_joint_command` port to the
Damiao coordinator and `leader_task` binds it to a `trajectory` task at priority
20, above a policy's lane. Calibrate with `dimos hardware feetech-leader
calibrate`; the `leader-teleop` extra installs the Feetech SDK.

## Subclassing

`WebXRTeleopModule` is designed for extension. Override these methods:

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
├── webxr/
│   ├── module.py                # Base WebXR teleop module (local WebSocket)
│   ├── extensions.py            # ArmTeleop, TwistTeleop
│   ├── controller_types.py      # WebXRControllerState, Buttons
│   └── web/
│       └── static/index.html    # WebXR client
├── hosted/                      # Hosted teleop (transport-swap, per-concern modules)
│   ├── go2_command.py           # Go2CommandModule: command/E-STOP dispatch + drive guard
│   ├── arm_command.py           # ArmCommandModule: tracked poses / EE-twist → coordinator tasks
│   ├── command_executor.py      # SerializedCommandExecutor: serialized cmds + safety fence
│   ├── camera_mux.py            # CameraMuxModule: N cameras → one composited video track
│   ├── map_compress.py          # MapCompressModule: costmap/odom → minimap datachannel
│   ├── hosted_stats.py          # HostedStatsModule: telemetry + acks + cmd-link stats
│   ├── robot_type.py            # RobotType: broker config → operator view auto-select
│   ├── README.md                # Broker session / aiortc + Cloudflare WebRTC internals
│   └── blueprints/              # cloudflare.py (Go2 transport + multicam, xArm6/7)
├── leader/
│   ├── module.py                # LeaderTeleopModule: Feetech leaders -> joint targets
│   ├── mapping.py               # SideMapping/JointMapping/GripperMapping (config, not code)
│   ├── coordinator.py           # LeaderCoordinator, leader_task, leader_gripper_tasks
│   ├── feetech.py               # Feetech serial readers + mock reader
│   ├── calibration.py           # Per-side calibration files under STATE_DIR/teleop/leader
│   └── cli/                     # dimos hardware feetech-leader {calibrate,joint-tui,setup-motor-id}
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
dimos run teleop-webxr-rerun     # WebXR teleop + Rerun viz
dimos run teleop-phone-go2      # Phone → Go2
```

Open `https://<host-ip>:<port>/teleop` on device. Accept the self-signed certificate.
- WebXR headset: port 8443
- Phone: port 8444
