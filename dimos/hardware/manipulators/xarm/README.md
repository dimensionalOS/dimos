# xArm Driver for dimos

Real-time driver for UFACTORY xArm5/6/7 manipulators integrated with the dimos framework.

## Quick Start

### 1. Set Robot IP
```bash
export XARM_IP=192.168.1.235
```

### 2. Basic Usage

```python
from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.msgs.sensor_msgs import JointState, JointCommand

# Start dimos and deploy driver
dimos = core.start(1)
xarm = dimos.deploy(XArmDriver, ip_address="192.168.1.235", num_joints=6)

# Configure LCM transports
xarm.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
xarm.joint_position_command.transport = core.LCMTransport("/xarm/joint_commands", JointCommand)

# Start and enable servo mode
xarm.start()
xarm.enable_servo_mode()

# Control via RPC
xarm.set_joint_angles([0, 0, 0, 0, 0, 0], speed=50, mvacc=100, mvtime=0)

# Cleanup
xarm.stop()
dimos.stop()
```

### 3. With Trajectory Generator

```python
from dimos.hardware.manipulators.xarm.sample_trajectory_generator import SampleTrajectoryGenerator

# Deploy driver and trajectory generator
xarm = dimos.deploy(XArmDriver, ip_address="192.168.1.235", num_joints=6)
traj_gen = dimos.deploy(SampleTrajectoryGenerator, num_joints=6, control_mode="position")

# Connect via LCM
xarm.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
xarm.joint_position_command.transport = core.LCMTransport("/xarm/joint_commands", JointCommand)
traj_gen.joint_state_input.transport = core.LCMTransport("/xarm/joint_states", JointState)
traj_gen.joint_position_command.transport = core.LCMTransport("/xarm/joint_commands", JointCommand)

# Start and execute trajectory
xarm.start()
traj_gen.start()
xarm.enable_servo_mode()
traj_gen.enable_publishing()
traj_gen.move_joint(joint_index=5, delta_degrees=10.0, duration=2.0)
```

## Key Features

- **100Hz control loop** for real-time position/velocity control
- **LCM pub/sub** for distributed system integration
- **RPC methods** for direct hardware control
- **Position mode** (radians) and **velocity mode** (deg/s)
- **Component-based API**: motion, kinematics, system, gripper control

## Topics

**Subscribed:**
- `/xarm/joint_position_command` - JointCommand (positions in radians)
- `/xarm/joint_velocity_command` - JointCommand (velocities in deg/s)

**Published:**
- `/xarm/joint_states` - JointState (100Hz)
- `/xarm/robot_state` - RobotState (10Hz)
- `/xarm/ft_ext`, `/xarm/ft_raw` - WrenchStamped (force/torque)

## Common RPC Methods

```python
# System control
xarm.enable_servo_mode()           # Enable position control (mode 1)
xarm.enable_velocity_control_mode() # Enable velocity control (mode 4)
xarm.motion_enable(True)           # Enable motors
xarm.clean_error()                 # Clear errors

# Motion control
xarm.set_joint_angles([...], speed=50, mvacc=100, mvtime=0)
xarm.set_servo_angle(joint_id=5, angle=0.5, speed=50)

# State queries
state = xarm.get_joint_state()
position = xarm.get_position()
```

## Configuration

Key parameters for `XArmDriver`:
- `ip_address`: Robot IP (default: "192.168.1.235")
- `num_joints`: 5, 6, or 7 (default: 6)
- `control_frequency`: Control loop rate in Hz (default: 100.0)
- `is_radian`: Use radians vs degrees (default: True)
- `enable_on_start`: Auto-enable servo mode (default: True)
- `velocity_control`: Use velocity vs position mode (default: False)

## Testing

```bash
# Interactive control (recommended)
venv/bin/python dimos/hardware/manipulators/xarm/interactive_control.py

# Run driver standalone
venv/bin/python dimos/hardware/manipulators/xarm/test_xarm_driver.py
```

## License

Copyright 2025 Dimensional Inc. - Apache License 2.0
