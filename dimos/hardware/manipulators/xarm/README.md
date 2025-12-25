# xArm Driver for dimos

Complete driver implementation for UFACTORY xArm robotic manipulators integrated with the dimos framework.

## Features

- **Full dimos Integration**: Uses `dimos.deploy()` with proper LCM transports
- **Dual-Threaded Architecture**: Separate 100Hz loops for joint state reading and control
- **Position & Velocity Control**: Support for both servo position control (mode 1) and velocity control (mode 4)
- **Trajectory Generation**: Sample trajectory generator with position and velocity trajectory support
- **Interactive Control**: User-friendly CLI for manual robot control
- **ROS-Compatible Messages**: JointState, RobotState, JointCommand
- **Comprehensive RPC API**: Full access to xArm SDK functionality
- **Hardware Monitoring**: Joint states, robot state, force/torque sensors
- **Firmware Version Detection**: Automatic API selection based on firmware

## Architecture

```
XArmDriver (dimos Module)
├── Joint State Thread (100Hz)
│   ├── Reads: position, velocity, effort
│   └── Publishes: /xarm/joint_states (LCM)
├── Robot State Thread (10Hz)
│   ├── Reads: state, mode, error_code, warn_code
│   └── Publishes: /xarm/robot_state (LCM)
├── Control Thread (100Hz)
│   ├── Subscribes: /xarm/joint_position_command, /xarm/joint_velocity_command
│   ├── Mode-aware: Switches between position (mode 1) and velocity (mode 4)
│   ├── Timeout protection: Stops robot if no commands for 1 second
│   └── Sends to hardware: set_servo_angle_j() or vc_set_joint_velocity()
├── Report Callback (event-driven)
│   ├── Updates state variables when SDK pushes data
│   └── Publishes: /xarm/ft_ext, /xarm/ft_raw (LCM)
└── RPC Methods
    ├── State queries: get_joint_state(), get_position()
    ├── Motion control: set_joint_angles(), set_servo_angle()
    ├── Mode switching: enable_velocity_control_mode(), disable_velocity_control_mode()
    └── System control: motion_enable(), clean_error()

SampleTrajectoryGenerator (dimos Module)
├── Control Loop (100Hz)
│   ├── Generates: Position or velocity commands
│   ├── Publishes to: /xarm/joint_position_command OR /xarm/joint_velocity_command
│   └── Auto-switches topic based on trajectory type
├── Position Trajectories
│   └── Linear interpolation between start and end positions
├── Velocity Trajectories
│   └── Constant velocity for specified duration
└── RPC Methods
    ├── move_joint(joint_index, delta_degrees, duration)
    └── move_joint_velocity(joint_index, velocity_deg_s, duration)
```

## Quick Start

### 1. Set xArm IP Address

```bash
export XARM_IP=192.168.1.235  # Your xArm's IP
```

### 2. Interactive Control (Recommended)

The easiest way to control the robot with both position and velocity modes:

```bash
venv/bin/python dimos/hardware/manipulators/xarm/interactive_control.py
```

This provides:
- **Mode selection**: Choose position or velocity control for each motion
- **Joint selection**: Move individual joints
- **Position mode**: Move by angle (degrees) over a duration
- **Velocity mode**: Move at constant velocity (deg/s) for a duration
- **Safety**: Automatic mode switching and state management

Example session:
```
Select control mode:
  1. Position control (move by angle)
  2. Velocity control (move with velocity)
Mode (1 or 2): 2

Which joint to move? (1-6): 6
Velocity (deg/s): 10
Duration (seconds): 2.0

⚙ Preparing for velocity control...
  Velocity control mode enabled (code: 0)
✓ Started velocity control on joint 6: 10.0°/s for 2.0s
```

### 3. Run Driver (Continuous Mode)

Start the driver and keep it running (publishes on LCM topics):

```bash
venv/bin/python dimos/hardware/manipulators/xarm/test_xarm_driver.py
```

The driver will publish:
- Joint states at ~100 Hz on `/xarm/joint_states`
- Robot state at ~10 Hz on `/xarm/robot_state`
- Force/torque data on `/xarm/ft_ext` and `/xarm/ft_raw`

Press `Ctrl+C` to stop the driver.

### 4. Velocity Control Test

Test velocity control with a simple script:

```bash
venv/bin/python dimos/hardware/manipulators/xarm/test_velocity_control.py
```

This sends a constant velocity command to joint 6 for 1 second, then stops.

### 5. Deploy in Your Application

#### Position Control Example

```python
from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.hardware.manipulators.xarm.sample_trajectory_generator import SampleTrajectoryGenerator
from dimos.msgs.sensor_msgs import JointState, RobotState, JointCommand

# Start dimos cluster
cluster = core.start(1)

# Deploy xArm driver
xarm = cluster.deploy(
    XArmDriver,
    ip_address="192.168.1.235",
    control_frequency=100.0,
    num_joints=6,
    enable_on_start=False,
)

# Set up driver transports
xarm.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
xarm.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
xarm.joint_position_command.transport = core.LCMTransport("/xarm/joint_position_command", JointCommand)

# Deploy trajectory generator
traj_gen = cluster.deploy(
    SampleTrajectoryGenerator,
    num_joints=6,
    control_mode="position",
    publish_rate=100.0,
)

# Set up trajectory generator transports
traj_gen.joint_state_input.transport = core.LCMTransport("/xarm/joint_states", JointState)
traj_gen.joint_position_command.transport = core.LCMTransport("/xarm/joint_position_command", JointCommand)

# Start modules
xarm.start()
traj_gen.start()

# Enable servo mode
xarm.enable_servo_mode()
traj_gen.enable_publishing()

# Move joint 6 by 10 degrees over 2 seconds
result = traj_gen.move_joint(joint_index=5, delta_degrees=10.0, duration=2.0)
print(result)

# Cleanup
traj_gen.stop()
xarm.stop()
cluster.stop()
```

#### Velocity Control Example

```python
from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.hardware.manipulators.xarm.sample_trajectory_generator import SampleTrajectoryGenerator
from dimos.msgs.sensor_msgs import JointState, RobotState, JointCommand

# Start dimos cluster
cluster = core.start(1)

# Deploy xArm driver
xarm = cluster.deploy(
    XArmDriver,
    ip_address="192.168.1.235",
    control_frequency=100.0,
    num_joints=6,
)

# Set up driver transports (note: both position AND velocity)
xarm.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
xarm.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
xarm.joint_position_command.transport = core.LCMTransport("/xarm/joint_position_command", JointCommand)
xarm.joint_velocity_command.transport = core.LCMTransport("/xarm/joint_velocity_command", JointCommand)

# Deploy trajectory generator
traj_gen = cluster.deploy(
    SampleTrajectoryGenerator,
    num_joints=6,
    control_mode="position",  # Will auto-switch to velocity when needed
    publish_rate=100.0,
)

# Set up trajectory generator transports (both topics)
traj_gen.joint_state_input.transport = core.LCMTransport("/xarm/joint_states", JointState)
traj_gen.joint_position_command.transport = core.LCMTransport("/xarm/joint_position_command", JointCommand)
traj_gen.joint_velocity_command.transport = core.LCMTransport("/xarm/joint_velocity_command", JointCommand)

# Start modules
xarm.start()
traj_gen.start()

# Enable velocity control mode (sets robot to mode 4, state 0)
code, msg = xarm.enable_velocity_control_mode()
print(f"Velocity mode: {msg}")

# Move joint 6 at 20 deg/s for 2 seconds
result = traj_gen.move_joint_velocity(joint_index=5, velocity_deg_s=20.0, duration=2.0)
print(result)

# Wait for completion, then return to position mode
time.sleep(2.5)
code, msg = xarm.disable_velocity_control_mode()
print(f"Position mode: {msg}")

# Cleanup
traj_gen.stop()
xarm.stop()
cluster.stop()
```

## Configuration

### XArmDriverConfig Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ip_address` | str | "192.168.1.235" | xArm controller IP address |
| `num_joints` | int | 6 | Number of joints (5, 6, or 7) |
| `control_frequency` | float | 100.0 | Control loop rate (Hz) |
| `joint_state_rate` | float | 100.0 | Joint state publishing rate (Hz) |
| `robot_state_rate` | float | 10.0 | Robot state publishing rate (Hz) |
| `report_type` | str | "dev" | SDK report type ("normal", "rich", "dev") |
| `enable_on_start` | bool | False | Enable servo mode on startup |
| `is_radian` | bool | True | Use radians for positions (True) or degrees (False) |
| `velocity_control` | bool | False | Enable velocity control mode on startup |
| `velocity_duration` | float | 0.1 | Duration parameter for vc_set_joint_velocity (seconds) |

## Message Types

### Input Topics (Commands)

- `joint_position_command: In[JointCommand]` - Target joint positions (radians)
- `joint_velocity_command: In[JointCommand]` - Target joint velocities (deg/s)

**Note**: Velocity commands are in **degrees/second**, not radians/second. This is due to the xArm SDK `vc_set_joint_velocity()` API expecting degrees/second.

### Output Topics (State)

- `joint_state: Out[JointState]` - Joint positions, velocities, efforts
- `robot_state: Out[RobotState]` - Robot state, mode, errors, warnings
- `ft_ext: Out[WrenchStamped]` - External force/torque (compensated)
- `ft_raw: Out[WrenchStamped]` - Raw force/torque sensor data

## RPC Methods

### XArmDriver RPC Methods

#### State Queries
- `get_joint_state() -> JointState` - Get current joint state
- `get_robot_state() -> RobotState` - Get robot status
- `get_position() -> Tuple[int, List[float]]` - Get TCP position/orientation
- `get_version() -> Tuple[int, str]` - Get firmware version

#### Motion Control
- `set_joint_angles(angles, speed, mvacc, mvtime) -> Tuple[int, str]`
- `set_servo_angle(joint_id, angle, speed, mvacc, mvtime) -> Tuple[int, str]`

#### Mode Switching
- `enable_servo_mode() -> Tuple[int, str]` - Enable servo mode (mode 1)
- `disable_servo_mode() -> Tuple[int, str]` - Disable servo mode
- `enable_velocity_control_mode() -> Tuple[int, str]` - Enable velocity control (mode 4, state 0)
- `disable_velocity_control_mode() -> Tuple[int, str]` - Return to position control (mode 1, state 0)

#### System Control
- `motion_enable(enable) -> Tuple[int, str]` - Enable/disable motors
- `set_mode(mode) -> Tuple[int, str]` - Set control mode
- `set_state(state) -> Tuple[int, str]` - Set robot state
- `clean_error() -> Tuple[int, str]` - Clear error codes
- `clean_warn() -> Tuple[int, str]` - Clear warning codes

### SampleTrajectoryGenerator RPC Methods

- `move_joint(joint_index, delta_degrees, duration) -> str` - Move joint by relative angle
- `move_joint_velocity(joint_index, velocity_deg_s, duration) -> str` - Move joint at constant velocity
- `enable_publishing() -> None` - Start publishing commands
- `disable_publishing() -> None` - Stop publishing commands
- `get_current_state() -> dict` - Get trajectory generator state

## Test Suite

The test suite validates full dimos deployment:

1. **Basic Connection** - Deploy, connect, get firmware version
2. **Joint State Reading** - Read joint states via RPC (30 samples)
3. **Command Sending** - Send motion commands via RPC
4. **RPC Methods** - Test all RPC method calls

### Expected Results

```
✓ TEST 1: Basic Connection - PASSED
✓ TEST 2: Joint State Reading - PASSED
⚠ TEST 3: Command Sending - May fail if robot not in correct state
✓ TEST 4: RPC Methods - PASSED

Total: 3/4 tests passed (Test 3 requires specific robot state)
```

## Troubleshooting

### GLIBC Version Error

```
OSError: /lib/x86_64-linux-gnu/libc.so.6: version `GLIBC_2.36' not found
```

This is caused by Open3D's system dependencies conflicting with your system's GLIBC version.

**Solution**: Run tests directly using the venv Python binary (do NOT activate the venv first):
```bash
# From repo root
venv/bin/python dimos/hardware/manipulators/xarm/test_xarm_driver.py

# Or use the wrapper script (which does this automatically)
./dimos/hardware/manipulators/xarm/test_xarm_deploy.sh
```

**Important**: Do NOT run `source venv/bin/activate` before running the tests. Use the Python binary directly.

### Connection Timeout

**Check**:
1. xArm is powered on
2. Network connection: `ping 192.168.1.235`
3. Firewall allows TCP connections
4. IP address is correct in `XARM_IP` environment variable

### Transport Not Specified Errors

These are expected when running without LCM transports configured. The driver will:
- Store state internally
- Provide RPC access to state
- Log at DEBUG level (not ERROR)

## Control Modes

The xArm supports different control modes:

- **Mode 0**: Position mode (basic)
- **Mode 1**: Servo mode (position control with continuous commands)
- **Mode 4**: Velocity control mode

### Position Control Workflow

1. Set robot to **mode 1** (servo mode) with `enable_servo_mode()`
2. Set robot **state to 0** (ready)
3. Send position commands via `/xarm/joint_position_command`
4. Commands are in **radians**

### Velocity Control Workflow

1. Set robot to **mode 4** with `enable_velocity_control_mode()` (also sets state to 0)
2. Send velocity commands via `/xarm/joint_velocity_command`
3. Commands are in **degrees/second** (not radians!)
4. After trajectory, call `disable_velocity_control_mode()` to return to position control

**Important**: The driver automatically switches between reading position and velocity commands based on the `velocity_control` config flag, which is set by the `enable_velocity_control_mode()` / `disable_velocity_control_mode()` RPC methods.

## Files

- `xarm_driver.py` - Main driver implementation with position and velocity control
- `sample_trajectory_generator.py` - Trajectory generator with position and velocity support
- `interactive_control.py` - Interactive CLI for manual robot control
- `test_velocity_control.py` - Velocity control test script
- `spec.py` - RobotState dataclass definition
- `test_xarm_driver.py` - Full dimos deployment test suite
- `test_xarm_driver_simple.py` - Lightweight SDK-only tests
- `test_xarm_minimal.py` - Minimal connection test
- `README.md` - This file

## Dependencies

- `xarm-python-sdk >= 1.17.0` - xArm Python SDK
- `dimos` - dimos framework
- Python 3.10+

## License

Copyright 2025 Dimensional Inc. - Apache License 2.0
