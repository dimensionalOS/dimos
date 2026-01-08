# Teleoperation

Teleoperation modules for controlling robots with various input devices.

## Overview

The teleoperation system consists of:
- **Base module** ([`base/`](base/)) - Common functionality for all devices
- **Device modules** (e.g., [`quest3/`](quest3/)) - Device-specific implementations
- **Robot controller** ([`teleop_robot_controller.py`](teleop_robot_controller.py)) - Applies deltas to robot

## Architecture

```
Device Module (Quest3, SpaceMouse, etc.)
    ├── Inherits from BaseTeleopModule
    ├── Device-specific connection (WebSocket, USB, etc.)
    └── Calls: update_controller_poses()
        ↓
BaseTeleopModule
    ├── Calibration (capture initial poses)
    ├── Delta computation (current - initial)
    ├── Publishes delta poses (LCM)
    └── Rerun visualization
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
from dimos.teleop.quest3 import Quest3TeleopModule

teleop = Quest3TeleopModule()
teleop.start()
# Open https://your-ip:8443 on Quest 3
```

See [Quest3 README](quest3/README.md) for details.

## Quick Start

### 1. Choose Your Device

```python
# Quest 3 VR
from dimos.teleop.quest3 import Quest3TeleopModule
device = Quest3TeleopModule()

# Future: SpaceMouse, other VR headsets, etc.
```

### 2. Start Teleoperation

```python
device.start()
# Press calibration button (X button for Quest3)
# Move controllers to control robot
```

### 3. Use with Blueprint

```python
from dimos.core.blueprints import autoconnect
from dimos.teleop.quest3 import quest3_teleop_module
from dimos.teleop import teleop_robot_controller

system = autoconnect(
    quest3_teleop_module(
        enable_left_arm=True,
        enable_right_arm=True,
    ),
    teleop_robot_controller(
        driver_module_name="YourDriver",
    ),
    your_robot_blueprint,
)

coordinator = system.build()
coordinator.loop()
```

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

## Creating New Devices

See [`base/README.md`](base/README.md) for instructions on creating new teleoperation devices.

Example devices you could add:
- SpaceMouse (USB 6DOF controller)
- Other VR headsets (HTC Vive, Valve Index)
- Haptic devices (Force Dimension Omega)
- Game controllers with custom mapping
- Mobile phone IMU-based control



## LCM Topics

All devices publish:

- `left_controller_delta: Out[PoseStamped]` - Left delta pose
- `right_controller_delta: Out[PoseStamped]` - Right delta pose
- `left_trigger: Out[Bool]` - Left trigger state
- `right_trigger: Out[Bool]` - Right trigger state

## Components

- **[`base/`](base/)** - Base teleoperation module (abstract)
- **[`quest3/`](quest3/)** - Quest 3 VR implementation
- **[`teleop_robot_controller.py`](teleop_robot_controller.py)** - Applies deltas to robot
- **[`teleop_blueprints.py`](teleop_blueprints.py)** - Pre-built system blueprints

## Benefits of This Architecture

1. **Reusability**: Common code shared across all devices
2. **Consistency**: Same behavior and interface for all devices
3. **Maintainability**: Bug fixes in base class benefit all devices
4. **Rapid Development**: New devices in ~50-100 lines of code
5. **Device Agnostic**: Robot doesn't know/care what device is used

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
        teleop_robot_controller(driver_module_name="XArmDriver"),
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

## Troubleshooting

### Device Not Publishing Deltas
1. Press calibration button to capture initial poses
2. Check `is_vr_calibrated()` returns True
3. Move controllers and check LCM topics

### Robot Not Moving
1. Ensure TeleopRobotController received first delta (auto-calibrates)
2. Check robot driver is running
3. Verify LCM topic connections in blueprint

### Want to Recalibrate
Just call `reset_calibration()` and press calibration button again.

## Related Documentation

- [Base Module](base/README.md) - Creating new devices
- [Quest3 Module](quest3/README.md) - Quest 3 VR setup
- [Blueprints](../../docs/api/blueprints.md) - System composition
