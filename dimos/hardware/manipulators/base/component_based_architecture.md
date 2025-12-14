# Component-Based Architecture for Manipulator Drivers

## Overview

This architecture provides maximum code reuse through standardized SDK wrappers and reusable components. Each new manipulator requires only an SDK wrapper (~200-500 lines) and a thin driver assembly (~30-50 lines).

## Architecture Layers

```
┌─────────────────────────────────────────────────────┐
│                   RPC Interface                      │
│              (Standardized across all arms)          │
└─────────────────────────────────────────────────────┘
                           ▲
┌─────────────────────────────────────────────────────┐
│              Driver Instance (XArmDriver)            │
│          Extends DIMOS Module, assembles components  │
└─────────────────────────────────────────────────────┘
                           ▲
┌─────────────────────────────────────────────────────┐
│              Standard Components                     │
│        (Motion, Servo, Status) - reused everywhere   │
└─────────────────────────────────────────────────────┘
                           ▲
┌─────────────────────────────────────────────────────┐
│              SDK Wrapper (XArmSDKWrapper)            │
│        Implements BaseManipulatorSDK interface       │
└─────────────────────────────────────────────────────┘
                           ▲
┌─────────────────────────────────────────────────────┐
│              Native Vendor SDK (XArmAPI)             │
└─────────────────────────────────────────────────────┘
```

## Core Interfaces

### BaseManipulatorSDK

Abstract interface that all SDK wrappers must implement. See `sdk_interface.py` for full specification.

**Required methods:** `connect()`, `disconnect()`, `is_connected()`, `get_joint_positions()`, `get_joint_velocities()`, `set_joint_positions()`, `enable_servos()`, `disable_servos()`, `emergency_stop()`, `get_error_code()`, `clear_errors()`, `get_info()`

**Optional methods:** `get_force_torque()`, `get_gripper_position()`, `set_cartesian_position()`, etc.

### ManipulatorCapabilities

Dataclass defining arm properties: DOF, joint limits, velocity limits, feature flags.

## Component System

### @component_api Decorator

Methods marked with `@component_api` are automatically exposed as RPC endpoints on the driver:

```python
from dimos.hardware.manipulators.base.components import component_api

class StandardMotionComponent:
    @component_api
    def move_joint(self, positions: list[float], velocity: float = 1.0) -> dict:
        """Auto-exposed as driver.move_joint()"""
        ...
```

### Dependency Injection

Components receive dependencies via setter methods, not constructor:

```python
class StandardMotionComponent:
    def __init__(self):
        self.sdk = None
        self.shared_state = None
        self.command_queue = None
        self.capabilities = None

    def set_sdk(self, sdk): self.sdk = sdk
    def set_shared_state(self, state): self.shared_state = state
    def set_command_queue(self, queue): self.command_queue = queue
    def set_capabilities(self, caps): self.capabilities = caps
    def initialize(self): pass  # Called after all setters
```

### Standard Components

| Component | Purpose | Key Methods |
|-----------|---------|-------------|
| `StandardMotionComponent` | Joint/cartesian motion | `move_joint()`, `move_joint_velocity()`, `get_joint_state()`, `stop_motion()` |
| `StandardServoComponent` | Motor control | `enable_servo()`, `disable_servo()`, `emergency_stop()`, `set_control_mode()` |
| `StandardStatusComponent` | Monitoring | `get_robot_state()`, `get_error_state()`, `get_health_metrics()` |

## Threading Model

The driver runs **2 threads**:

1. **Control Loop (100Hz)** - Process commands, read joint state, publish feedback
2. **Monitor Loop (10Hz)** - Read robot state, errors, optional sensors (F/T, gripper)

```
RPC Call → Command Queue → Control Loop → SDK → Hardware
                              ↓
                         SharedState (thread-safe)
                              ↓
                         LCM Publisher → External Systems
```

## DIMOS Module Integration

The driver extends `Module` for pub/sub integration:

```python
class BaseManipulatorDriver(Module):
    def __init__(self, sdk, components, config, capabilities):
        super().__init__()
        self.shared_state = SharedState()
        self.command_queue = Queue(maxsize=10)

        # Inject dependencies into components
        for component in components:
            component.set_sdk(sdk)
            component.set_shared_state(self.shared_state)
            component.set_command_queue(self.command_queue)
            component.set_capabilities(capabilities)
            component.initialize()

        # Auto-expose @component_api methods
        self._auto_expose_component_apis()
```

## Adding a New Manipulator

### Step 1: SDK Wrapper

```python
class YourArmSDKWrapper(BaseManipulatorSDK):
    def get_joint_positions(self) -> list[float]:
        degrees = self._sdk.get_angles()
        return [math.radians(d) for d in degrees]  # Convert to radians

    def set_joint_positions(self, positions, velocity, acceleration) -> bool:
        return self._sdk.move_joints(positions, velocity)

    # ... implement remaining required methods
```

### Step 2: Driver Assembly

```python
class YourArmDriver(BaseManipulatorDriver):
    def __init__(self, config: dict):
        sdk = YourArmSDKWrapper()
        capabilities = ManipulatorCapabilities(
            dof=6,
            joint_limits_lower=[-3.14] * 6,
            joint_limits_upper=[3.14] * 6,
        )
        components = [
            StandardMotionComponent(),
            StandardServoComponent(),
            StandardStatusComponent(),
        ]
        super().__init__(sdk, components, config, capabilities)
```

## Unit Conventions

All SDK wrappers must convert to standard units:

| Quantity | Unit |
|----------|------|
| Positions | radians |
| Velocities | rad/s |
| Accelerations | rad/s^2 |
| Torques | Nm |
| Cartesian | meters |

## Testing Strategy

```python
# Test SDK wrapper with mocked native SDK
def test_wrapper_positions():
    mock = Mock()
    mock.get_angles.return_value = [0, 90, 180]
    wrapper = YourArmSDKWrapper()
    wrapper._sdk = mock
    assert wrapper.get_joint_positions() == [0, math.pi/2, math.pi]

# Test component with mocked SDK wrapper
def test_motion_component():
    mock_sdk = Mock(spec=BaseManipulatorSDK)
    component = StandardMotionComponent()
    component.set_sdk(mock_sdk)
    component.move_joint([0, 0, 0])
    # Verify command was queued
```

## Advantages

- **Maximum reuse**: Components tested once, used by 100+ arms
- **Consistent behavior**: All arms identical at RPC level
- **Centralized fixes**: Fix once in component, all arms benefit
- **Team scalability**: Developers work on wrappers independently
- **Strong contracts**: SDK interface defines exact requirements

## Reference Implementations

- **XArm**: `xarm/xarm_wrapper.py` - Full-featured, converts degrees→radians
- **Piper**: `piper/piper_wrapper.py` - Shows velocity integration workaround
