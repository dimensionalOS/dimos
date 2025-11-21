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
┌─────────────────────────────────────────────────────────────┐
│  TargetSetter (Interactive CLI)                             │
│  - User inputs target positions                             │
│  - Preserves orientation when left blank                    │
└───────────────────────┬─────────────────────────────────────┘
                        │ PoseStamped (/target_pose)
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
┌─────────────────────────────────┐  (back to TargetSetter
│  Hardware Driver (xArm, etc.)   │   for orientation preservation)
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
- Works with **any** arm driver implementing `ArmDriverSpec` protocol
- Only requires `get_inverse_kinematics()` and `get_forward_kinematics()` RPC methods
- Supports xArm, Piper, UR, Franka, or custom arms

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

```python
from dimos.hardware.manipulators.xarm import XArmDriver, XArmDriverConfig
from dimos.manipulation.control import CartesianMotionController, CartesianMotionControllerConfig

# 1. Create hardware driver
arm_driver = XArmDriver(config=XArmDriverConfig(ip_address="192.168.1.235"))

# 2. Create Cartesian controller (hardware-agnostic!)
controller = CartesianMotionController(
    arm_driver=arm_driver,
    config=CartesianMotionControllerConfig(
        control_frequency=20.0,
        position_kp=1.0,
        max_linear_velocity=0.15,  # m/s
    )
)

# 3. Set up topic connections (shared memory)
from dimos.core.transport import pSHMTransport

transport_joint_state = pSHMTransport("joint_state")
transport_joint_cmd = pSHMTransport("joint_cmd")

arm_driver.joint_state.connection = transport_joint_state
controller.joint_state.connection = transport_joint_state
controller.joint_position_command.connection = transport_joint_cmd
arm_driver.joint_position_command.connection = transport_joint_cmd

# 4. Start modules
arm_driver.start()
controller.start()

# 5. Send Cartesian goal (move 10cm in X)
controller.set_target_pose(
    position=[0.3, 0.0, 0.5],  # xyz in meters
    orientation=[0, 0, 0],     # roll, pitch, yaw in radians
    frame_id="world"
)

# 6. Wait for convergence
while not controller.is_converged():
    time.sleep(0.1)

print("Target reached!")
```

### Using Quaternions

```python
from dimos.msgs.geometry_msgs import Quaternion

# Create quaternion (identity rotation)
quat = Quaternion(x=0, y=0, z=0, w=1)

controller.set_target_pose(
    position=[0.4, 0.1, 0.6],
    orientation=[quat.x, quat.y, quat.z, quat.w],  # 4-element list
)
```

### Using PoseStamped Messages

```python
from dimos.msgs.geometry_msgs import PoseStamped

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

### Using the TargetSetter Tool

The `TargetSetter` is an interactive CLI tool that makes it easy to manually send target poses to the controller. It provides a user-friendly interface for testing and teleoperation.

**Key Features:**
- **Interactive terminal UI** - prompts for x, y, z coordinates
- **Orientation preservation** - automatically uses current orientation when left blank
- **Live feedback** - subscribes to controller's current pose
- **Simple workflow** - just enter coordinates and press Enter

**Setup:**

```python
# Terminal 1: Start the controller (as shown in Basic Example above)
arm_driver = XArmDriver(config=XArmDriverConfig(ip_address="192.168.1.235"))
controller = CartesianMotionController(arm_driver=arm_driver)

# Set up LCM transports for target_pose and current_pose
from dimos.core import LCMTransport
controller.target_pose.connection = LCMTransport("/target_pose", PoseStamped)
controller.current_pose.connection = LCMTransport("/xarm/current_pose", PoseStamped)

arm_driver.start()
controller.start()

# Terminal 2: Run the target setter
python -m dimos.manipulation.control.target_setter
```

**Usage Example:**

```
================================================================================
Interactive Target Setter
================================================================================
Mode: WORLD FRAME (absolute coordinates)

Enter target coordinates (Ctrl+C to quit)
================================================================================

--------------------------------------------------------------------------------

Enter target position (in meters):
  x (m): 0.3
  y (m): 0.0
  z (m): 0.5

Enter orientation (in degrees, leave blank to preserve current orientation):
  roll (°):
  pitch (°):
  yaw (°):

✓ Published target (preserving current orientation):
  Position: x=0.3000m, y=0.0000m, z=0.5000m
  Orientation: roll=0.0°, pitch=0.0°, yaw=0.0°
```

**How It Works:**

1. **TargetSetter** subscribes to `/xarm/current_pose` from the controller
2. User enters target position (x, y, z) in meters
3. User can optionally enter orientation (roll, pitch, yaw) in degrees
4. If orientation is left blank (0, 0, 0), TargetSetter uses the current orientation from the controller
5. TargetSetter publishes the target pose to `/target_pose` topic
6. **CartesianMotionController** receives the target and tracks it

**Benefits:**

- **No orientation math** - just move positions without worrying about quaternions
- **Safe testing** - manually verify each move before sending
- **Quick iteration** - test different positions interactively
- **Educational** - see the controller respond in real-time

## Configuration

```python
@dataclass
class CartesianMotionControllerConfig:
    # Control loop
    control_frequency: float = 20.0  # Hz (recommend 10-50Hz)
    command_timeout: float = 1.0     # seconds

    # PID gains (position)
    position_kp: float = 1.0   # m/s per meter of error
    position_ki: float = 0.0   # Integral gain
    position_kd: float = 0.1   # Derivative gain (damping)

    # PID gains (orientation)
    orientation_kp: float = 2.0   # rad/s per radian of error
    orientation_ki: float = 0.0
    orientation_kd: float = 0.2

    # Safety limits
    max_linear_velocity: float = 0.2   # m/s
    max_angular_velocity: float = 1.0  # rad/s
    max_position_error: float = 0.5    # m (emergency stop threshold)
    max_orientation_error: float = 1.57  # rad (~90°)

    # Convergence
    position_tolerance: float = 0.001  # m (1mm)
    orientation_tolerance: float = 0.01  # rad (~0.57°)

    # Control mode
    velocity_control_mode: bool = True  # Use velocity-based control
```

## Hardware Abstraction

The controller uses the **Protocol pattern** for hardware abstraction:

```python
# spec.py
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
           # Your IK implementation
           return (0, joint_angles)

       @rpc
       def get_forward_kinematics(self, angles: list[float]) -> tuple[int, list[float] | None]:
           # Your FK implementation
           return (0, tcp_pose)
   ```

2. Use with controller:
   ```python
   my_driver = MyArmDriver()
   controller = CartesianMotionController(arm_driver=my_driver)
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
| `current_pose` | `PoseStamped` | Current TCP pose (for TargetSetter and other tools) |

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
├── target_setter.py                     # Interactive target pose publisher
├── example_cartesian_control.py         # Usage example
└── README.md                            # This file
```

## Related Modules

- [xarm_driver.py](../../hardware/manipulators/xarm/xarm_driver.py) - Hardware driver for xArm
- [spec.py](../../hardware/manipulators/xarm/spec.py) - Protocol specification
- [simple_controller.py](../../utils/simple_controller.py) - PID implementation

## License

Copyright 2025 Dimensional Inc. - Apache 2.0 License
