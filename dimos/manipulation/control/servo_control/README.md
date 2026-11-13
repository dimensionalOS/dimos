# Cartesian Motion Controller

Hardware-agnostic Cartesian space motion controller for robotic manipulators.

## Overview

The `CartesianMotionController` provides closed-loop Cartesian pose tracking by:
1. **Subscribing** to target poses (PoseStamped)
2. **Computing** Cartesian error (position + orientation)
3. **Generating** velocity commands using PID control
4. **Converting** to joint space via IK
5. **Publishing** joint commands to the hardware driver

## Architecture

```
  PoseStamped (/target_pose)
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│  CartesianMotionController                                  │
│  - Computes FK (current pose)                               │
│  - Computes Cartesian error                                 │
│  - PID control → Cartesian velocity                         │
│  - Integrates velocity → next desired pose                  │
│  - Computes IK → target joint angles                        │
│  - Publishes current pose for feedback                      │
└──────────┬────────────────────────────────┬─────────────────┘
           │ JointCommand                   │ PoseStamped
           │                                │ (current_pose)
           ▼                                ▼
┌─────────────────────────────────┐    (feedback to caller)
│  Arm driver (ArmDriverSpec)     │
│  - 100Hz control loop           │
│  - Sends commands to robot      │
│  - Publishes JointState         │
└─────────────────────────────────┘
           │ JointState
           │ (feedback)
           ▼
  (back to controller)
```

## Key Features

### ✓ Hardware Agnostic
- Works with **any** arm driver implementing `ArmDriverSpec` protocol (none ships with dimos yet)
- Only requires `get_inverse_kinematics()` and `get_forward_kinematics()` RPC methods

### ✓ PID-Based Control
- Separate PIDs for position (X, Y, Z) and orientation (roll, pitch, yaw)
- Configurable gains and velocity limits
- Smooth, stable motion with damping

### ✓ Safety Features
- Configurable position/orientation error limits
- Automatic emergency stop on excessive errors
- Command timeout detection
- Convergence monitoring

### ✓ Flexible Input
- RPC method: `set_target_pose(position, orientation, frame_id)`
- Topic subscription: `target_pose` (PoseStamped messages)
- Supports both Euler angles and quaternions

## Usage

### Basic Example

The controller needs an arm driver module that implements `ArmDriverSpec`. No arm in dimos ships one today (the xArm
driver this example used was removed in January 2026), so `MyArmDriver` below is yours: see [Adding a New Arm](#adding-a-new-arm).

```python
import time

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.manipulation.control.servo_control.cartesian_motion_controller import CartesianMotionController

# Modules run in worker processes that import this script again, so everything
# that builds or drives them goes under the __main__ guard.
if __name__ == "__main__":
    # 1. Build your arm driver and the controller; autoconnect connects their streams by name
    coordinator = ModuleCoordinator.build(autoconnect(
        MyArmDriver.blueprint(),
        CartesianMotionController.blueprint(
            control_frequency=20.0,
            position_kp=1.0,
            max_linear_velocity=0.15,  # m/s
        ),
    ))
    controller = coordinator.get_instance(CartesianMotionController)

    # 2. Send Cartesian goal (move 10cm in X)
    controller.set_target_pose(
        position=[0.3, 0.0, 0.5],  # xyz in meters
        orientation=[0, 0, 0],     # roll, pitch, yaw in radians
        frame_id="world"
    )

    # 3. Wait for convergence
    while not controller.is_converged():
        time.sleep(0.1)

    print("Target reached!")
    coordinator.stop()
```

The snippets below continue from this one: in a script, put them inside the `__main__` block, before `coordinator.stop()`.

### Using Quaternions

```python
from dimos.msgs.geometry_msgs.Quaternion import Quaternion

# Create quaternion (identity rotation)
quat = Quaternion(x=0, y=0, z=0, w=1)

controller.set_target_pose(
    position=[0.4, 0.1, 0.6],
    orientation=[quat.x, quat.y, quat.z, quat.w],  # 4-element list
)
```

### Using PoseStamped Messages

```python
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

# Create target pose
target = PoseStamped(
    frame_id="world",
    position=[0.3, 0.2, 0.5],
    orientation=[0, 0, 0, 1]  # quaternion
)

# Option 1: Via RPC
controller.set_target_pose(
    position=list(target.position),
    orientation=list(target.orientation)
)

# Option 2: Via topic (if connected)
controller.target_pose.publish(target)
```

### Sending Target Poses

Target poses can be published to the controller's `/target_pose` topic on the active transport. The controller will track the target using its PID loop.


## Configuration

```python
class CartesianMotionControllerConfig(ModuleConfig):
    # Control loop
    control_frequency: float = 20.0  # Hz (recommend 10-50Hz)
    command_timeout: float = 30.0    # seconds - timeout for stale targets

    # PID gains (position)
    position_kp: float = 5.0   # m/s per meter of error
    position_ki: float = 0.1   # Integral gain
    position_kd: float = 0.1   # Derivative gain (damping)

    # PID gains (orientation)
    orientation_kp: float = 2.0   # rad/s per radian of error
    orientation_ki: float = 0.0
    orientation_kd: float = 0.2

    # Safety limits
    max_linear_velocity: float = 0.2   # m/s
    max_angular_velocity: float = 1.0  # rad/s
    max_position_error: float = 0.7    # m (emergency stop threshold)
    max_orientation_error: float = 6.28  # rad (~360°, allows any orientation)

    # Convergence
    position_tolerance: float = 0.001  # m (1mm)
    orientation_tolerance: float = 0.01  # rad (~0.57°)

    # Control mode
    velocity_control_mode: bool = True  # Use velocity-based control
    control_frame: str = "world"  # Frame for target poses
```

## Hardware Abstraction

The controller uses the **Protocol pattern** for hardware abstraction:

```python
# The RPCs are ArmDriverSpec (dimos/manipulation/control/arm_driver_spec.py); the topics are what autoconnect connects
class ArmDriverSpec(Protocol):
    # Required RPC methods
    def get_inverse_kinematics(self, pose: list[float]) -> tuple[int, list[float] | None]: ...
    def get_forward_kinematics(self, angles: list[float]) -> tuple[int, list[float] | None]: ...

    # Required topics
    joint_state: Out[JointState]
    robot_state: Out[RobotState]
    joint_position_command: In[JointCommand]
```

**Any driver implementing this protocol works with the controller!**

### Adding a New Arm

1. Implement `ArmDriverSpec` protocol:
   ```python
   class MyArmDriver(Module):
       @rpc
       def get_inverse_kinematics(self, pose: list[float]) -> tuple[int, list[float] | None]:
           # Your IK implementation; pose is [x, y, z] in mm, then roll, pitch, yaw in radians
           return (0, joint_angles)

       @rpc
       def get_forward_kinematics(self, angles: list[float]) -> tuple[int, list[float] | None]:
           # Your FK implementation; return the pose in the same units as IK's
           return (0, tcp_pose)
   ```

2. Use with controller:
   ```python
   blueprint = autoconnect(MyArmDriver.blueprint(), CartesianMotionController.blueprint())
   ```

**That's it! No changes to the controller needed.**

## RPC Methods

### Control Methods

```python
@rpc
def set_target_pose(
    position: list[float],           # [x, y, z] in meters
    orientation: list[float],         # [qx, qy, qz, qw] or [roll, pitch, yaw]
    frame_id: str = "world"
) -> None
```

```python
@rpc
def clear_target() -> None
```

### Query Methods

```python
@rpc
def get_current_pose() -> Optional[Pose]
```

```python
@rpc
def is_converged() -> bool
```

## Topics

### Inputs (Subscriptions)

| Topic | Type | Description |
|-------|------|-------------|
| `joint_state` | `JointState` | Current joint positions/velocities (from driver) |
| `robot_state` | `RobotState` | Robot status (from driver) |
| `target_pose` | `PoseStamped` | Desired TCP pose (from planner) |

### Outputs (Publications)

| Topic | Type | Description |
|-------|------|-------------|
| `joint_position_command` | `JointCommand` | Target joint angles (to driver) |
| `cartesian_velocity` | `Twist` | Debug: Cartesian velocity commands |
| `current_pose` | `PoseStamped` | Current TCP pose (for feedback and monitoring) |

## Control Algorithm

```
1. Read current joint state from driver
2. Compute FK: joint angles → TCP pose
3. Compute error: e = target_pose - current_pose
4. PID control: velocity = PID(e, dt)
5. Integrate: next_pose = current_pose + velocity * dt
6. Compute IK: next_pose → target_joints
7. Publish target_joints to driver
```

### Why This Works

- **Outer loop (Cartesian)**: Runs at 10-50Hz, computes IK
- **Inner loop (Joint)**: Driver runs at 100Hz, executes smoothly
- **Decoupling**: Separates high-level planning from low-level control

## Tuning Guide

### Conservative (Safe)
```python
config = CartesianMotionControllerConfig(
    control_frequency=10.0,
    position_kp=0.5,
    max_linear_velocity=0.1,  # Slow!
)
```

### Moderate (Recommended)
```python
config = CartesianMotionControllerConfig(
    control_frequency=20.0,
    position_kp=1.0,
    position_kd=0.1,
    max_linear_velocity=0.15,
)
```

### Aggressive (Fast)
```python
config = CartesianMotionControllerConfig(
    control_frequency=50.0,
    position_kp=2.0,
    position_kd=0.2,
    max_linear_velocity=0.3,
)
```

### Tips

- **Increase Kp**: Faster response, but may oscillate
- **Increase Kd**: More damping, smoother motion
- **Increase Ki**: Eliminates steady-state error (usually not needed)
- **Lower frequency**: Less CPU load, smoother
- **Higher frequency**: Faster response, more accurate

## Extending

### Next Steps (Phase 2+)

1. **Trajectory Following**: Add waypoint tracking
   ```python
   controller.follow_trajectory(waypoints: list[Pose], duration: float)
   ```

2. **Collision Avoidance**: Integrate with planning
   ```python
   controller.set_collision_checker(checker: CollisionChecker)
   ```

3. **Impedance Control**: Add force/torque feedback
   ```python
   controller.set_impedance(stiffness: float, damping: float)
   ```

4. **Visual Servoing**: Integrate with perception
   ```python
   controller.track_object(object_id: int)
   ```

## Troubleshooting

### Controller not moving
- Check `arm_driver` is started and publishing `joint_state`
- Verify topic connections are set up
- Check robot is in correct mode (servo mode for xArm)

### Oscillation / Instability
- Reduce `position_kp` or `orientation_kp`
- Increase `position_kd` or `orientation_kd`
- Lower `control_frequency`

### IK failures
- Target pose may be unreachable
- Check joint limits
- Verify pose is within workspace
- Check singularity avoidance

### Not converging
- Increase `position_tolerance` / `orientation_tolerance`
- Check for workspace limits
- Increase `max_linear_velocity`

## Files

```
dimos/manipulation/control/
├── __init__.py                          # Module exports
├── cartesian_motion_controller.py       # Main controller
├── example_cartesian_control.py         # Usage example
└── README.md                            # This file
```

## Related Modules

- [simple_controller.py](/dimos/utils/simple_controller.py) - PID implementation

## License

Copyright 2025 Dimensional Inc. - Apache 2.0 License
