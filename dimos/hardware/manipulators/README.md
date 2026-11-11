# Manipulator Drivers

This module provides manipulator arm adapters: Protocol-only, run by the `ControlCoordinator`.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                ControlCoordinator (Module)                  │
│  - Runs the control loop                                    │
│  - Creates one adapter per HardwareComponent of a blueprint │
│  - Runs tasks (trajectory, velocity, gripper, ...)          │
│  - Publishes coordinator_joint_state                        │
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
- **Flexible**: One coordinator can run several arms, each with its own adapter
- **Simple**: No ABC inheritance required - just implement the Protocol
- **Type-safe**: Full type checking via `ManipulatorAdapter` Protocol

## Directory Structure

```
manipulators/
├── spec.py              # ManipulatorAdapter Protocol + shared types
├── registry.py          # Adapter registry (lazy _registry.py manifests)
├── mock/
│   ├── _registry.py  # Declares "mock" → adapter import path
│   └── adapter.py       # MockAdapter for testing
├── xarm/
│   ├── _registry.py
│   ├── adapter.py       # XArmAdapter (SDK wrapper)
└── piper/
    ├── _registry.py
    ├── adapter.py       # PiperAdapter (SDK wrapper)
```

## Quick Start

### Running an Arm

```bash
dimos run coordinator-mock    # A mock arm, no hardware needed
dimos run coordinator-xarm7 --controlcoordinator.addresses '{"arm": "192.168.1.185"}'
```

### Using an Adapter Directly

The mock adapter shows the API without hardware. Drive real arms through the coordinator, which streams small steps
to them.

```python
from dimos.hardware.manipulators.registry import adapter_registry

arm = adapter_registry.create("mock", dof=6)
arm.connect()
arm.activate()
arm.write_joint_positions([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
print(arm.read_joint_positions())
arm.deactivate()
arm.disconnect()
```

## Adding a New Arm

1. **Create the adapter** (`adapter.py`):

```python
class MyArmAdapter:  # No inheritance needed - just match the Protocol
    def __init__(self, address: str, dof: int = 6, **_: object) -> None:
        self._address = address
        self._dof = dof

    def connect(self) -> bool: ...
    def disconnect(self) -> None: ...
    def read_joint_positions(self) -> list[float]: ...
    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool: ...
    # ... implement other Protocol methods
```

2. **Declare it in a manifest** (`_registry.py`, stdlib imports only):

```python
ADAPTER_FACTORIES = {
    "myarm": "dimos.hardware.manipulators.myarm.adapter:MyArmAdapter",
}
```

3. **Add it to a blueprint** with a `HardwareComponent` whose `adapter_type` is the manifest name. The coordinator
   creates the adapter with `address`, `dof` (the number of joints), `hardware_id` and the component's
   `adapter_kwargs`:

```python
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator

coordinator_myarm = ControlCoordinator.blueprint(
    hardware=[HardwareComponent(hardware_id="arm", hardware_type=HardwareType.MANIPULATOR,
                                joints=make_joints("arm", 6), adapter_type="myarm", address="192.168.1.100")],
)
```

See [Adding a Custom Arm](/docs/capabilities/manipulation/adding_a_custom_arm.md) for the full guide.

## ManipulatorAdapter Protocol

All adapters must implement these core methods:

| Category | Methods |
|----------|---------|
| Connection | `connect()`, `disconnect()`, `is_connected()` |
| Lifecycle | `activate()`, `deactivate()` |
| Info | `get_info()`, `get_dof()`, `get_limits()` |
| State | `read_state()`, `read_error()`, `read_joint_positions()`, `read_joint_velocities()`, `read_joint_efforts()` |
| Motion | `write_joint_positions()`, `write_joint_velocities()`, `write_stop()` |
| Servo | `write_enable()`, `read_enabled()`, `write_clear_errors()` |
| Mode | `set_control_mode()`, `get_control_mode()` |

Optional methods (return `None`/`False` if unsupported):
- `read_cartesian_position()`, `write_cartesian_position()`
- `read_force_torque()`

A gripper has no methods of its own: it is the last entry of the joint arrays (`read_joint_positions()`, `write_joint_positions()`, ...), in the adapter's own gripper unit, the one `get_limits()` declares.

## Unit Conventions

All adapters convert to/from SI units:

| Quantity | Unit |
|----------|------|
| Angles | radians |
| Angular velocity | rad/s |
| Torque | Nm |
| Position | meters |
| Force | Newtons |
