# Manipulator Drivers

This module provides manipulator arm drivers: Protocol-only with injectable adapters.

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
│              Adapter (implements Protocol)                   │
│  - Handles SDK communication                                 │
│  - Unit conversions (radians ↔ vendor units)                │
│  - Swappable: XArmAdapter, PiperAdapter, MockAdapter        │
└─────────────────────────────────────────────────────────────┘
```

## Key Benefits

- **Testable**: Inject `MockAdapter` for unit tests without hardware
- **Flexible**: Each arm controls its own threading/timing
- **Simple**: No ABC inheritance required - just implement the Protocol
- **Type-safe**: Full type checking via `ManipulatorAdapter` Protocol

## Directory Structure

```
manipulators/
├── spec.py              # ManipulatorAdapter Protocol + shared types
├── registry.py          # Adapter registry with auto-discovery
├── mock/
│   └── adapter.py       # MockAdapter for testing
├── xarm/
│   ├── adapter.py       # XArmAdapter (SDK wrapper)
└── piper/
    ├── adapter.py       # PiperAdapter (SDK wrapper)
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
from dimos.hardware.manipulators.mock import MockAdapter
from dimos.hardware.manipulators.xarm import XArm

arm = XArm(adapter=MockAdapter(dof=6))
arm.start()  # No hardware needed!
arm.move_joint([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
```

## Adding a New Arm

1. **Create the adapter** (`adapter.py`):

```python
class MyArmAdapter:  # No inheritance needed - just match the Protocol
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
from .adapter import MyArmAdapter

class MyArm(Module[MyArmConfig]):
    joint_state: Out[JointState]
    robot_state: Out[RobotState]
    joint_position_command: In[JointCommand]

    def __init__(self, adapter=None, **kwargs):
        super().__init__(**kwargs)
        self.adapter = adapter or MyArmAdapter(
            ip=self.config.ip,
            dof=self.config.dof,
        )
        # ... setup control loops
```

3. **Create blueprints** (`blueprints.py`) for common configurations.

## ManipulatorAdapter Protocol

All adapters must implement these core methods:

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

All adapters convert to/from SI units:

| Quantity | Unit |
|----------|------|
| Angles | radians |
| Angular velocity | rad/s |
| Torque | Nm |
| Position | meters |
| Force | Newtons |
