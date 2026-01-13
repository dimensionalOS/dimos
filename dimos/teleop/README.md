# Teleoperation [TODO: Update]

Teleoperation modules for controlling robots with various input devices.

## Overview

The teleoperation system consists of:
- **Base module** ([`devices/base/`](devices/base/)) - Common functionality for all devices
- **Device modules** (e.g., [`devices/vr_headset/`](devices/vr_headset/)) - Device-specific implementations
- **Robot controllers** ([`robot_controllers/`](robot_controllers/)) - Applies deltas to robot

## Folder Structure

```
teleop/
├── README.md                       # This file
├── devices/                        # Teleop devices
│   ├── base/                       # Base teleoperation module
│   │   ├── __init__.py
│   │   ├── base_teleop_module.py  # BaseTeleopModule (calibration, deltas, publishing)
│   │   └── README.md               # Guide for creating new devices
│   └── vr_headset/                 # VR headset implementation (Quest 3 tested)
│       ├── __init__.py
│       ├── vr_teleop_module.py    # VRTeleopModule (WebSocket server)
│       ├── README.md               # VR headset setup and usage
│       ├── control/
│       │   ├── fastapi_server.py  # FastAPI/WebSocket server
│       │   └── tracking_processor.py  # VR tracking data processing
│       ├── static/
│       │   └── index.html         # VR client (HTML/JS)
│       └── certs/                  # SSL certificates (auto-generated)
├── robot_controllers/              # Robot-side controllers
│   ├── teleop_arm_controller.py   # Applies deltas to robot arm
│   └── __init__.py
└── teleop_blueprints.py            # Pre-built system blueprints
```

## Architecture

```
BaseTeleopModule (base class)
    ├── Calibration (capture initial poses)
    ├── Delta computation (current - initial)
    ├── Publishes delta poses (LCM)
    ├── Rerun visualization
    └── update_controller_poses() ← Device modules call this
        ↑
Device Module (VR headset, SpaceMouse, etc.)
    ├── Inherits from BaseTeleopModule
    ├── Device-specific connection (WebSocket, USB, etc.)
    └── Receives tracking data → calls update_controller_poses()
        ↓
TeleopArmController
    ├── Receives delta poses
    ├── Auto-calibrates robot
    └── Applies: target = initial_robot + delta
        ↓
Robot Driver → Robot
```

## Supported Devices

### VR Headset (Quest 3 tested)
**Module**: [`devices/vr_headset/`](devices/vr_headset/)
**Type**: VR controllers (6DOF dual-arm)
**Connection**: WebSocket/HTTPS (WebXR)
**Features**: AR mode, multi-controller support, analog trigger values

```python
from dimos.teleop.teleop_blueprints import quest3_teleop

coordinator = quest3_teleop.build()
coordinator.loop()
# Open https://your-ip:8443 on Quest 3
```
`quest3_teleop` uses `VRTeleopModule` under the hood with the updated `devices/vr_headset` layout.

## Quick Start

**Use the pre-built blueprint** - it's ready to run:

```python
from dimos.teleop.teleop_blueprints import quest3_teleop

# Pre-built blueprint with VR headset + TeleopArmController
coordinator = quest3_teleop.build()
coordinator.loop()
```

That's it! The blueprint includes:
- VRTeleopModule (VR calibration, delta computation)
- TeleopArmController (robot calibration, delta application)
- All LCM topic connections configured

Then:
1. Open `https://your-ip:8443/` on Quest 3
2. Press X button to calibrate and start teleoperation
3. Move controllers to control robot

## How It Works

### Two-Stage Calibration

1. **Device Calibration** (press calibration button):
   - Captures initial controller poses
   - Starts publishing **delta poses**: `delta = current - initial`

2. **Robot Calibration** (automatic on first delta):
   - TeleopArmController captures robot's initial pose
   - Applies deltas: `target_pose = robot_initial + delta`

### Data Flow

```
Controllers → Device Module → Delta Poses → Arm Controller → Robot
            (absolute)      (relative)     (absolute)
```

**Key insight**: Device publishes deltas, not absolute poses. This makes the system:
- ✅ Device-agnostic (any coordinate frame works)
- ✅ Robot-agnostic (works with any robot)
- ✅ Recalibrable (just press button again)


## LCM Topics

All devices publish per-controller topics:

- `controller_delta_{i}: Out[PoseStamped]` - Controller i delta pose
- `trigger_value_{i}: Out[Float32]` - Controller i trigger/gripper value (0.0-1.0)


## Example: Full System

```python
from dimos.teleop.teleop_blueprints import quest3_teleop

# Pre-built blueprint with VR headset + TeleopArmController
coordinator = quest3_teleop.build()
coordinator.loop()
```

Or build custom:

```python
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.std_msgs import Float32
from dimos.teleop.devices.vr_headset import vr_teleop_module
from dimos.teleop.robot_controllers import teleop_arm_controller

my_system = (
    autoconnect(
        vr_teleop_module(signaling_port=8443),
        teleop_arm_controller(driver_module_name="RobotDriver"),
        your_robot,
    )
    .transports({
        ("controller_delta_0", PoseStamped): LCMTransport("/teleop/left", PoseStamped),
        ("controller_delta_1", PoseStamped): LCMTransport("/teleop/right", PoseStamped),
        ("trigger_value_0", Float32): LCMTransport("/teleop/left_trigger", Float32),
        ("trigger_value_1", Float32): LCMTransport("/teleop/right_trigger", Float32),
    })
)

coordinator = my_system.build()
coordinator.loop()
```

## Related Documentation

- [Base Module](devices/base/README.md) - Creating new devices
- [VR Headset Module](devices/vr_headset/README.md) - VR headset setup
- [Blueprints](../../docs/api/blueprints.md) - System composition
