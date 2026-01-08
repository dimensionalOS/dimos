# Teleoperation

Teleoperation modules for controlling robots with various input devices.

## Overview

The teleoperation system consists of:
- **Base module** ([`base/`](base/)) - Common functionality for all devices
- **Device modules** (e.g., [`quest3/`](quest3/)) - Device-specific implementations
- **Robot controller** ([`teleop_robot_controller.py`](teleop_robot_controller.py)) - Applies deltas to robot

## Folder Structure

```
teleop/
├── __init__.py                    # Package exports
├── README.md                       # This file
├── base/                           # Base teleoperation module
│   ├── __init__.py
│   ├── base_teleop_module.py      # BaseTeleopModule (calibration, deltas, publishing)
│   └── README.md                   # Guide for creating new devices
├── quest3/                         # Quest 3 VR implementation
│   ├── __init__.py
│   ├── quest3_teleop_module.py    # Quest3TeleopModule (WebSocket server)
│   ├── README.md                   # Quest3 setup and usage
│   ├── control/
│   │   ├── fastapi_server.py      # FastAPI/WebSocket server
│   │   └── tracking_processor.py  # VR tracking data processing
│   ├── static/
│   │   └── index.html             # VR client (HTML/JS)
│   └── certs/                      # SSL certificates (auto-generated)
├── teleop_robot_controller.py     # Applies deltas to robot
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
Device Module (Quest3, SpaceMouse, etc.)
    ├── Inherits from BaseTeleopModule
    ├── Device-specific connection (WebSocket, USB, etc.)
    └── Receives tracking data → calls update_controller_poses()
        ↓
TeleopRobotController
    ├── Receives delta poses
    ├── Auto-calibrates robot
    └── Applies: target = initial_robot + delta
        ↓
Robot Driver → Robot
```

## Supported Devices

### Quest 3 VR Headset [Any VR headset]
**Module**: [`quest3/`](quest3/)
**Type**: VR controllers (6DOF dual-arm)
**Connection**: WebSocket/HTTPS (WebXR)
**Features**: AR mode, dual-arm control, trigger buttons

```python
from dimos.teleop.teleop_blueprints import quest3_teleop

coordinator = quest3_teleop.build()
coordinator.loop()
# Open https://your-ip:8443 on Quest 3
```

## Quick Start

**Use the pre-built blueprint** - it's ready to run:

```python
from dimos.teleop.teleop_blueprints import quest3_teleop

# Pre-built blueprint with Quest3 + TeleopRobotController
coordinator = quest3_teleop.build()
coordinator.loop()
```

That's it! The blueprint includes:
- Quest3TeleopModule (VR calibration, delta computation)
- TeleopRobotController (robot calibration, delta application)
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
   - TeleopRobotController captures robot's initial pose
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

All devices publish:

- `left_controller_delta: Out[PoseStamped]` - Left delta pose
- `right_controller_delta: Out[PoseStamped]` - Right delta pose
- `left_trigger: Out[Float32]` - Left trigger/gripper value (0.0-1.0)
- `right_trigger: Out[Float32]` - Right trigger/gripper value (0.0-1.0)


## Example: Full System

```python
from dimos.teleop.teleop_blueprints import quest3_teleop

# Pre-built blueprint with Quest3 + TeleopRobotController
coordinator = quest3_teleop.build()
coordinator.loop()
```

Or build custom:

```python
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.teleop.quest3 import quest3_teleop_module
from dimos.teleop import teleop_robot_controller

my_system = (
    autoconnect(
        quest3_teleop_module(signaling_port=8443),
        teleop_robot_controller(driver_module_name="RobotDriver"),
        your_robot,
    )
    .transports({
        ("left_controller_delta", PoseStamped): LCMTransport("/teleop/left", PoseStamped),
        ("right_controller_delta", PoseStamped): LCMTransport("/teleop/right", PoseStamped),
    })
)

coordinator = my_system.build()
coordinator.loop()
```

## Related Documentation

- [Base Module](base/README.md) - Creating new devices
- [Quest3 Module](quest3/README.md) - Quest 3 VR setup
- [Blueprints](../../docs/api/blueprints.md) - System composition
