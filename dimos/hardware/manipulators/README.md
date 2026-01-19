# Manipulator Drivers

This module provides manipulator arm drivers using the **B-lite architecture**: Protocol-only with injectable backends.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      Driver (Module)                        │
│  - Owns threading (control loop, monitor loop)              │
│  - Publishes joint_state, robot_state                       │
│  - Subscribes to joint_position_command, joint_velocity_cmd │
│  - Exposes RPC methods (move_joint, enable_servos, etc.)    │
└─────────────────────┬───────────────────────────────────────┘
                      │ uses
┌─────────────────────▼───────────────────────────────────────┐
│              Backend (implements Protocol)                   │
│  - Handles SDK communication                                 │
│  - Unit conversions (radians ↔ vendor units)                │
│  - Swappable: XArmBackend, PiperBackend, MockBackend        │
└─────────────────────────────────────────────────────────────┘
```

## Key Benefits

- **Testable**: Inject `MockBackend` for unit tests without hardware
- **Flexible**: Each arm controls its own threading/timing
- **Simple**: No ABC inheritance required - just implement the Protocol
- **Type-safe**: Full type checking via `ManipulatorBackend` Protocol

## Directory Structure

```
manipulators/
├── spec.py              # ManipulatorBackend Protocol + shared types
├── mock/
│   └── backend.py       # MockBackend for testing
├── xarm/
│   ├── backend.py       # XArmBackend (SDK wrapper)
│   ├── arm.py           # XArm driver module
│   └── blueprints.py    # Pre-configured blueprints
└── piper/
    ├── backend.py       # PiperBackend (SDK wrapper)
    ├── arm.py           # Piper driver module
    └── blueprints.py    # Pre-configured blueprints
```

## Quick Start

### Using a Driver Directly

```python
from dimos.hardware.manipulators.xarm import XArm

arm = XArm(ip="192.168.1.185", dof=6)
arm.start()
arm.enable_servos()
arm.move_joint([0, 0, 0, 0, 0, 0])
arm.stop()
```

### Using Blueprints

```python
from dimos.hardware.manipulators.xarm.blueprints import xarm_trajectory

coordinator = xarm_trajectory.build()
coordinator.loop()
```

### Testing Without Hardware

```python
from dimos.hardware.manipulators.mock import MockBackend
from dimos.hardware.manipulators.xarm import XArm

arm = XArm(backend=MockBackend(dof=6))
arm.start()  # No hardware needed!
arm.move_joint([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
```

## Adding a New Arm

1. **Create the backend** (`backend.py`):

```python
class MyArmBackend:  # No inheritance needed - just match the Protocol
    def __init__(self, ip: str = "192.168.1.100", dof: int = 6) -> None:
        self._ip = ip
        self._dof = dof

    def connect(self) -> bool: ...
    def disconnect(self) -> None: ...
    def read_joint_positions(self) -> list[float]: ...
    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool: ...
    # ... implement other Protocol methods
```

2. **Create the driver** (`arm.py`):

```python
from dimos.core import Module, ModuleConfig, In, Out, rpc
from .backend import MyArmBackend

class MyArm(Module[MyArmConfig]):
    joint_state: Out[JointState]
    robot_state: Out[RobotState]
    joint_position_command: In[JointCommand]

    def __init__(self, backend=None, **kwargs):
        super().__init__(**kwargs)
        self.backend = backend or MyArmBackend(
            ip=self.config.ip,
            dof=self.config.dof,
        )
        # ... setup control loops
```

3. **Create blueprints** (`blueprints.py`) for common configurations.

## ManipulatorBackend Protocol

All backends must implement these core methods:

| Category | Methods |
|----------|---------|
| Connection | `connect()`, `disconnect()`, `is_connected()` |
| Info | `get_info()`, `get_dof()`, `get_limits()` |
| State | `read_joint_positions()`, `read_joint_velocities()`, `read_joint_efforts()` |
| Motion | `write_joint_positions()`, `write_joint_velocities()`, `write_stop()` |
| Servo | `write_enable()`, `read_enabled()`, `write_clear_errors()` |
| Mode | `set_control_mode()`, `get_control_mode()` |

Optional methods (return `None`/`False` if unsupported):
- `read_cartesian_position()`, `write_cartesian_position()`
- `read_gripper_position()`, `write_gripper_position()`
- `read_force_torque()`

## Unit Conventions

All backends convert to/from SI units:

| Quantity | Unit |
|----------|------|
| Angles | radians |
| Angular velocity | rad/s |
| Torque | Nm |
| Position | meters |
| Force | Newtons |

## Available Blueprints

### XArm
- `xarm_servo` - Basic servo control (6-DOF)
- `xarm5_servo`, `xarm7_servo` - 5/7-DOF variants
- `xarm_trajectory` - Driver + trajectory controller
- `xarm_cartesian` - Driver + cartesian controller

### Piper
- `piper_servo` - Basic servo control
- `piper_servo_gripper` - With gripper support
- `piper_trajectory` - Driver + trajectory controller
- `piper_left`, `piper_right` - Dual arm configurations
