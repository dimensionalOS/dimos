# Sample Trajectory Generator

A dimos module that demonstrates how to create a controller/trajectory generator for the xArm manipulator.

## Overview

The `SampleTrajectoryGenerator` module:
- **Subscribes** to joint states and robot states from the xArm driver
- **Publishes** joint position commands OR velocity commands (never both)
- Runs a control loop at a configurable rate (default 10 Hz)
- Currently sends zero commands (safe for testing)

## Architecture

```
┌─────────────────────────┐
│   XArmDriver            │
│                         │
│  Publishes:             │
│   - joint_states (100Hz)│──────┐
│   - robot_state (10Hz)  │──────┤
│                         │      │
│  Subscribes:            │      │
│   - joint_position_cmd  │◄─────┤
│   - joint_velocity_cmd  │      │
└─────────────────────────┘      │
                                 │
                                 │
┌─────────────────────────┐      │
│ TrajectoryGenerator     │      │
│                         │      │
│  Subscribes:            │      │
│   - joint_states        │◄─────┘
│   - robot_state         │◄─────┘
│                         │
│  Publishes (one of):    │
│   - joint_position_cmd  │──────┐
│   - joint_velocity_cmd  │      │
└─────────────────────────┘      │
                                 │
                                 ▼
                          LCM Topics
```

## Usage

### Basic Example

```python
from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.hardware.manipulators.xarm.sample_trajectory_generator import SampleTrajectoryGenerator
from dimos.msgs.sensor_msgs import JointState, RobotState

# Start cluster
cluster = core.start(1)

# Deploy xArm driver
xarm = cluster.deploy(XArmDriver, ip_address="192.168.1.235", num_joints=6)

# Set up driver transports
xarm.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
xarm.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
xarm.joint_position_command.transport = core.LCMTransport("/xarm/joint_position_command", list)

xarm.start()

# Deploy trajectory generator
traj_gen = cluster.deploy(
    SampleTrajectoryGenerator,
    num_joints=6,
    control_mode="position",  # or "velocity"
    publish_rate=10.0,
    enable_on_start=False,  # Start in safe mode
)

# Set up trajectory generator transports
traj_gen.joint_state_input.transport = core.LCMTransport("/xarm/joint_states", JointState)
traj_gen.robot_state_input.transport = core.LCMTransport("/xarm/robot_state", RobotState)
traj_gen.joint_position_command.transport = core.LCMTransport("/xarm/joint_position_command", list)

traj_gen.start()

# Enable command publishing (when ready)
traj_gen.enable_publishing()

# Get current state
state = traj_gen.get_current_state()
print(f"Publishing enabled: {state['publishing_enabled']}")
print(f"Joint positions: {state['joint_state'].position}")
print(f"Robot state: {state['robot_state'].state}")
```

### Run Complete Example

```bash
export XARM_IP=192.168.1.235
venv/bin/python dimos/hardware/manipulators/xarm/example_with_trajectory_gen.py
```

## Configuration

### TrajectoryGeneratorConfig

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_joints` | int | 6 | Number of robot joints (5, 6, or 7) |
| `control_mode` | str | "position" | Control mode: "position" or "velocity" |
| `publish_rate` | float | 10.0 | Command publishing rate (Hz) |
| `enable_on_start` | bool | False | Start publishing commands immediately |

## Topics

### Inputs (Subscriptions)

- `joint_state_input: In[JointState]` - Current joint positions, velocities, efforts
- `robot_state_input: In[RobotState]` - Current robot state, mode, errors

### Outputs (Publications)

**Important**: Only ONE command topic should be published at a time!

- `joint_position_command: Out[List[float]]` - Target joint positions (radians)
- `joint_velocity_command: Out[List[float]]` - Target joint velocities (rad/s)

## RPC Methods

### Control Methods

- `start()` - Start the trajectory generator
- `stop()` - Stop the trajectory generator
- `enable_publishing()` - Enable command publishing
- `disable_publishing()` - Disable command publishing

### Query Methods

- `get_current_state() -> dict` - Get current joint/robot state and publishing status

Returns:
```python
{
    "joint_state": JointState,      # Latest joint state
    "robot_state": RobotState,      # Latest robot state
    "publishing_enabled": bool      # Whether commands are being published
}
```

## Extending the Generator

The `_generate_command()` method is where you implement trajectory generation logic:

```python
def _generate_command(self) -> Optional[List[float]]:
    """Generate command for the robot."""
    # Get current state
    with self._state_lock:
        current_js = self._current_joint_state
        current_rs = self._current_robot_state

    if current_js is None:
        return None  # Not ready yet

    # Example: Generate sinusoidal trajectory for first joint
    t = time.time()
    amplitude = 0.1  # radians
    frequency = 0.5  # Hz

    command = list(current_js.position)  # Start with current position
    command[0] = amplitude * math.sin(2 * math.pi * frequency * t)

    return command
```

## Safety Features

1. **Safe by Default**: Publishing is disabled on start (`enable_on_start=False`)
2. **Zero Commands**: Currently sends zeros (robot stays in place)
3. **State Monitoring**: Subscribes to robot state for safety checks
4. **Enable/Disable**: Can enable/disable publishing via RPC

## Important Notes

### Command Publishing

- **Never publish both position AND velocity commands simultaneously**
- The driver will use whichever command arrives last
- Choose one control mode and stick to it

### Control Modes

**Position Mode** (`control_mode="position"`):
- Publishes to `joint_position_command`
- Robot moves to target positions
- Good for: Point-to-point motion, trajectory following

**Velocity Mode** (`control_mode="velocity"`):
- Publishes to `joint_velocity_command`
- Robot moves at target velocities
- Good for: Continuous motion, teleoperation

### LCM Topic Naming

Following ROS naming conventions:
- ✅ `joint_position_command` (clear, descriptive)
- ✅ `joint_velocity_command` (clear, descriptive)
- ❌ `joint_cmd` (ambiguous)
- ❌ `velocity_cmd` (ambiguous)

## Example Output

```
================================================================================
xArm Driver + Trajectory Generator Example
================================================================================

Using xArm at IP: 192.168.1.235

Starting dimos cluster...
Deploying XArmDriver...
Setting up driver transports...
Starting xArm driver...
Deploying SampleTrajectoryGenerator...
Setting up trajectory generator transports...
Starting trajectory generator...

================================================================================
✓ System is running!
================================================================================

Topology:
  XArmDriver:
    Publishes: /xarm/joint_states (~100 Hz)
               /xarm/robot_state (~10 Hz)
    Subscribes: /xarm/joint_position_command
                /xarm/joint_velocity_command

  TrajectoryGenerator:
    Subscribes: /xarm/joint_states
                /xarm/robot_state
    Publishes: /xarm/joint_position_command (~10 Hz)

Commands:
  - Publishing is DISABLED by default (safe mode)
  - Call traj_gen.enable_publishing() to start sending commands
  - Currently sends zero commands (safe)

Press Ctrl+C to stop...
```

## Files

- `sample_trajectory_generator.py` - Trajectory generator module implementation
- `example_with_trajectory_gen.py` - Complete example with xArm driver
- `TRAJECTORY_GENERATOR.md` - This file
