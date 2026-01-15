# Manipulator Drivers

Component-based framework for integrating robotic manipulators into DIMOS.

## Quick Start: Adding a New Manipulator

Adding support for a new robot arm requires **two files**:
1. **SDK Wrapper** (~200-500 lines) - Translates vendor SDK to standard interface
2. **Driver** (~30-50 lines) - Assembles components and configuration

## Directory Structure

```
manipulators/
├── base/                      # Framework (don't modify)
│   ├── sdk_interface.py       # BaseManipulatorSDK abstract class
│   ├── driver.py              # BaseManipulatorDriver base class
│   ├── spec.py                # ManipulatorCapabilities dataclass
│   └── components/            # Reusable standard components
├── xarm/                      # XArm implementation (reference)
└── piper/                     # Piper implementation (reference)
```

## Hardware Requirements

Your manipulator **must** support:

| Requirement | Description |
|-------------|-------------|
| Joint Position Feedback | Read current joint angles |
| Joint Position Control | Command target joint positions |
| Servo Enable/Disable | Enable and disable motor power |
| Error Reporting | Report error codes/states |
| Emergency Stop | Hardware or software e-stop |

**Optional:** velocity control, torque control, cartesian control, F/T sensor, gripper

## Step 1: Implement SDK Wrapper

Create `your_arm/your_arm_wrapper.py` implementing `BaseManipulatorSDK`:

```python
from dimos.hardware.manipulators.base.sdk_interface import BaseManipulatorSDK, ManipulatorInfo

class YourArmSDKWrapper(BaseManipulatorSDK):
    def __init__(self):
        self._sdk = None

    def connect(self, config: dict) -> bool:
        self._sdk = YourNativeSDK(config['ip'])
        return self._sdk.connect()

    def get_joint_positions(self) -> list[float]:
        """Return positions in RADIANS."""
        degrees = self._sdk.get_angles()
        return [math.radians(d) for d in degrees]

    def set_joint_positions(self, positions: list[float],
                           velocity: float, acceleration: float) -> bool:
        return self._sdk.move_joints(positions, velocity)

    def enable_servos(self) -> bool:
        return self._sdk.motor_on()

    # ... implement remaining required methods (see sdk_interface.py)
```

### Unit Conventions

**All SDK wrappers must use these standard units:**

| Quantity | Unit |
|----------|------|
| Joint positions | radians |
| Joint velocities | rad/s |
| Joint accelerations | rad/s^2 |
| Joint torques | Nm |
| Cartesian positions | meters |
| Forces | N |

## Step 2: Create Driver Assembly

Create `your_arm/your_arm_driver.py`:

```python
from dimos.hardware.manipulators.base.driver import BaseManipulatorDriver
from dimos.hardware.manipulators.base.spec import ManipulatorCapabilities
from dimos.hardware.manipulators.base.components import (
    StandardMotionComponent,
    StandardServoComponent,
    StandardStatusComponent,
)
from .your_arm_wrapper import YourArmSDKWrapper

class YourArmDriver(BaseManipulatorDriver):
    def __init__(self, config: dict):
        sdk = YourArmSDKWrapper()

        capabilities = ManipulatorCapabilities(
            dof=6,
            has_gripper=False,
            has_force_torque=False,
            joint_limits_lower=[-3.14, -2.09, -3.14, -3.14, -3.14, -3.14],
            joint_limits_upper=[3.14, 2.09, 3.14, 3.14, 3.14, 3.14],
            max_joint_velocity=[2.0] * 6,
            max_joint_acceleration=[4.0] * 6,
        )

        components = [
            StandardMotionComponent(),
            StandardServoComponent(),
            StandardStatusComponent(),
        ]

        super().__init__(sdk, components, config, capabilities)
```

## Component API Decorator

Use `@component_api` to expose methods as RPC endpoints:

```python
from dimos.hardware.manipulators.base.components import component_api

class StandardMotionComponent:
    @component_api
    def move_joint(self, positions: list[float], velocity: float = 1.0):
        """Auto-exposed as driver.move_joint()"""
        ...
```

## Threading Architecture

The driver runs **2 threads**:
1. **Control Loop (100Hz)** - Processes commands, reads joint state, publishes feedback
2. **Monitor Loop (10Hz)** - Reads robot state, errors, optional sensors

```
RPC Call → Command Queue → Control Loop → SDK → Hardware
                              ↓
                         SharedState → LCM Publisher
```

## Testing Your Driver

```python
driver = YourArmDriver({"ip": "192.168.1.100"})
driver.start()
driver.enable_servo()
driver.move_joint([0, 0, 0, 0, 0, 0], velocity=0.5)
state = driver.get_joint_state()
driver.stop()
```

## Common Issues

| Issue | Solution |
|-------|----------|
| Unit mismatch | Verify wrapper converts to radians/meters |
| Commands ignored | Ensure servos are enabled before commanding |
| Velocity not working | Some arms need mode switch via `set_control_mode()` |

## Architecture Details

For complete architecture documentation including full SDK interface specification,
component details, and testing strategies, see:

**[component_based_architecture.md](base/component_based_architecture.md)**

## Reference Implementations

- **XArm**: [xarm/xarm_wrapper.py](xarm/xarm_wrapper.py) - Full-featured wrapper
- **Piper**: [piper/piper_wrapper.py](piper/piper_wrapper.py) - Shows velocity workaround
