# xArm Real-time Driver

A real-time controller for the xArm manipulator family (xArm5, xArm6, xArm7) compatible with the xArm Python SDK.

## Architecture Overview

The driver implements a **dual-threaded, callback-driven architecture** for real-time control:

```
┌─────────────────────────────────────────────────────────────────┐
│                      XArmDriver Module                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  MAIN THREAD (Event Loop)                                        │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Input Topics (Non-blocking Callbacks):                    │ │
│  │  ┌──────────────┐         ┌──────────────────┐            │ │
│  │  │  joint_cmd   │────────▶│  _on_joint_cmd() │            │ │
│  │  │List[float]   │         │  (stores latest) │            │ │
│  │  └──────────────┘         └─────────┬────────┘            │ │
│  │                                      │                      │ │
│  │  ┌──────────────┐         ┌──────────▼───────┐            │ │
│  │  │ velocity_cmd │────────▶│ _on_velocity_cmd()│            │ │
│  │  │List[float]   │         │  (stores latest) │            │ │
│  │  └──────────────┘         └─────────┬────────┘            │ │
│  │                                      │                      │ │
│  │  RPC Methods (Callable):             ▼                      │ │
│  │  • set_joint_angles()     ┌────────────────┐              │ │
│  │  • enable_servo_mode()    │ Shared State   │              │ │
│  │  • clean_error()          │ (Thread-safe)  │              │ │
│  │  • get_position()         │                │              │ │
│  │  • move_gohome()          │ • joint_cmd_   │◀─────────┐   │ │
│  │  • emergency_stop()       │ • vel_cmd_     │          │   │ │
│  │  • etc...                 │ • joint_states_│──────────┼─┐ │ │
│  │                           │ • robot_state_ │◀─────┐   │ │ │ │
│  │  SDK Callback (Event-Driven):  └─────────────┘      │   │ │ │ │
│  │  ┌──────────────────────────────────┐               │   │ │ │ │
│  │  │ _report_data_callback()          │───────────────┘   │ │ │ │
│  │  │ (100Hz if report_type='dev')     │                   │ │ │ │
│  │  │ • Update curr_state, curr_err    │───────────────────┼─┼─┤ │
│  │  │ • Publish robot_state topic      │                   │ │ │ │
│  │  │ • Publish FT sensor data         │                   │ │ │ │
│  │  └──────────────────────────────────┘                   │ │ │ │
│  │                                                        │ │ │ │
│  └────────────────────────────────────────────────────────┘ │ │ │
│                                       ▲                      │ │ │
│                                       │                      │ │ │
│  CONTROL THREAD (100Hz Real-time Loop)                      │ │ │
│  ┌────────────────────────────────────┼────────────────┐  │ │ │ │
│  │                                     │                │  │ │ │ │
│  │  ┌──────────────────────────────────┴──────────┐    │  │ │ │ │
│  │  │ 1. Read joint_cmd_ from shared state        │    │  │ │ │ │
│  │  └──────────────────┬──────────────────────────┘    │  │ │ │ │
│  │                     ▼                                │  │ │ │ │
│  │  ┌──────────────────────────────────────────────┐   │  │ │ │ │
│  │  │ 2. Send command via set_servo_angle_j()      │   │  │ │ │ │
│  │  └──────────────────┬───────────────────────────┘   │  │ │ │ │
│  │                     ▼                                │  │ │ │ │
│  │  ┌──────────────────────────────────────────────┐   │  │ │ │ │
│  │  │ 3. Read joint state via get_servo_angle()    │   │  │ │ │ │
│  │  └──────────────────┬───────────────────────────┘   │  │ │ │ │
│  │                     ▼                                │  │ │ │ │
│  │  ┌──────────────────────────────────────────────┐   │  │ │ │ │
│  │  │ 4. Write to joint_states_ & publish          │───┼──┘ │ │ │
│  │  └──────────────────┬───────────────────────────┘   │    │ │ │
│  │                     ▼                                │    │ │ │
│  │  ┌──────────────────────────────────────────────┐   │    │ │ │
│  │  │ 5. Sleep to maintain 100Hz                   │   │    │ │ │
│  │  └──────────────────┬───────────────────────────┘   │    │ │ │
│  │                     │                                │      │ │
│  │                     └────────────────────────────────┘      │ │
│  └─────────────────────────────────────────────────────────────┘ │
│                               │                                   │
│                               ▼                                   │
│                    ┌────────────────────┐                         │
│                    │   xArm SDK API     │                         │
│                    │   (XArmAPI)        │                         │
│                    └─────────┬──────────┘                         │
│                              │                                    │
│  Output Topics:              │                                    │
│  ┌──────────────────────────▼┐         ┌──────────────┐          │
│  │  joint_state              │────────▶│ JointState   │          │
│  │  Out[JointState]          │         │ subscribers  │          │
│  └───────────────────────────┘         └──────────────┘          │
│                                                                   │
│  ┌───────────────────────────┐         ┌──────────────┐          │
│  │  robot_state              │────────▶│ RobotState   │          │
│  │  Out[RobotState]          │         │ subscribers  │          │
│  └───────────────────────────┘         └──────────────┘          │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

**Key Design Principles:**
- **2 Threads Total**: Main thread (callbacks + RPC + SDK callback) + Control thread (RT loop)
- **Event-Driven State Updates**: SDK callback provides robot state at ~100Hz (dev mode)
- **Separation of Concerns**:
  - Control thread: Critical real-time joint control (100Hz)
  - SDK callback: Robot state, errors, FT sensor (~100Hz if dev mode)
- **Lock-Protected Shared State**: All shared variables use `threading.Lock()`
- **Non-blocking Callbacks**: All callbacks just store/publish data, never block

## Key Components

### 1. **Shared State Management**
- `joint_cmd_`: Latest joint position command (protected by `_joint_cmd_lock`)
- `vel_cmd_`: Latest velocity command (protected by `_joint_cmd_lock`)
- `joint_states_`: Latest joint state reading (protected by `_joint_state_lock`)
- `robot_state_`: Latest robot state reading (protected by `_joint_state_lock`)

All shared state uses `threading.Lock()` for thread-safe access.

### 2. **Control Thread (100Hz)**
```python
def _control_loop(self):
    # Critical real-time loop at 100Hz (configurable)
    # ONLY handles time-critical operations:
    # 1. Read latest joint_cmd_ from shared state
    # 2. Send command via arm.set_servo_angle_j(angles)
    # 3. Read joint state via arm.get_servo_angle()
    # 4. Write to joint_states_ shared state
    # 5. Publish joint_state to topic
    # 6. Sleep to maintain frequency
```

**Key Requirements:**
- Servo mode (mode 1) must be enabled
- Uses `set_servo_angle_j()` which executes only the last instruction
- Maintains precise timing with next_time tracking
- **Only joint commands and joint states** - no robot state reading here

### 3. **SDK Report Callback (Main Thread, Event-Driven)**
```python
def _report_data_callback(self, data: dict):
    # Called by SDK at configured frequency (report_type)
    # Receives robot state data from SDK:
    # 1. Update curr_state, curr_err, curr_mode, curr_cmdnum, curr_warn
    # 2. Create and publish RobotState message
    # 3. Publish force/torque sensor data (if available)
```

**Key Points:**
- **Event-driven** - SDK calls this automatically
- **Frequency depends on report_type**:
  - `'dev'`: ~100Hz (high frequency, recommended)
  - `'rich'`: ~5Hz (includes torque data)
  - `'normal'`: ~5Hz (basic state only)
- Runs in **SDK's background thread** (not control loop)
- Provides all state in one callback (state, mode, errors, warnings, cmdnum, mtbrake, mtable)

### 4. **Topic Subscriptions (Non-blocking)**
```python
def _on_joint_cmd(self, joint_cmd: List[float]):
    # Non-blocking callback
    # Just store the latest command in shared state
    with self._joint_cmd_lock:
        self._joint_cmd_ = list(joint_cmd)
```

**Design Pattern:**
- Callbacks are non-blocking
- Store latest data in shared state
- Control loop processes at fixed frequency

### 5. **RPC Methods**
All xArm SDK API functions are exposed as RPC methods:
- Return `Tuple[int, str]` for commands (code, message)
- Return `Tuple[int, Optional[T]]` for queries (code, result)
- Thread-safe access to shared state

## Files Created

1. **[JointState.py](../../../msgs/sensor_msgs/JointState.py)** - ROS sensor_msgs/JointState message type
2. **[spec.py](spec.py)** - Protocol specification with RobotState dataclass
3. **[xarm_driver.py](xarm_driver.py)** - Main driver implementation

## Configuration

```python
@dataclass
class XArmDriverConfig(ModuleConfig):
    ip_address: str = "192.168.1.185"      # xArm IP address
    is_radian: bool = True                  # Use radians (True) or degrees (False)
    control_frequency: float = 100.0        # Control loop frequency in Hz (joint cmds & states)
    report_type: str = "dev"                # SDK report type: 'dev'=100Hz, 'rich'=5Hz+torque, 'normal'=5Hz
    enable_on_start: bool = True            # Enable servo mode on start
    num_joints: int = 7                     # Number of joints (5, 6, or 7)
    check_joint_limit: bool = True          # Check joint limits
    check_cmdnum_limit: bool = True         # Check command queue limit
    max_cmdnum: int = 512                   # Maximum command queue size
```

## Usage Example

```python
from dimos.hardware.manipulators.xarm import XArmDriver, XArmDriverConfig
from dimos.core import LCMTransport
from dimos.msgs.sensor_msgs import JointState

# Configure driver
config = XArmDriverConfig(
    ip_address="192.168.1.185",
    num_joints=7,
    control_frequency=100.0,
    enable_on_start=True
)

# Create driver instance
driver = XArmDriver(config=config)

# Set up transports
driver.joint_cmd.transport = LCMTransport("/xarm/joint_cmd", list)
driver.joint_state.transport = LCMTransport("/xarm/joint_state", JointState)

# Start the driver
driver.start()

# Send commands via topic
driver.joint_cmd.publish([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Or use RPC methods
code, msg = driver.set_joint_angles([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
print(f"Code: {code}, Message: {msg}")

# Get state via RPC
joint_state = driver.get_joint_state()
print(f"Current position: {joint_state.position}")

# Stop the driver
driver.stop()
```

## Servo Mode Control

The driver requires servo mode (mode 1) for real-time control:

```python
# Enable servo mode (done automatically on start if enable_on_start=True)
driver.enable_servo_mode()

# Send commands - executed immediately at 100Hz
driver.joint_cmd.publish(target_angles)

# Disable servo mode when done
driver.disable_servo_mode()
```

## Thread Safety

All shared state access is protected:
- `_joint_cmd_lock` protects command state
- `_joint_state_lock` protects sensor state
- Callbacks are non-blocking (store and return)
- Control loop reads at fixed frequency

## Error Handling

```python
# Clear errors via RPC
driver.clean_error()
driver.clean_warn()

# Emergency stop
driver.emergency_stop()

# Check robot state
robot_state = driver.get_robot_state()
if robot_state.error_code != 0:
    print(f"Error: {robot_state.error_code}")
```

## API Code Reference

See [xarm_api_code.md](../../../../xArm-Python-SDK/doc/api/xarm_api_code.md) for error code definitions.

Common codes:
- `0`: Success
- `1`: Emergency stop activated
- `2`: Servo error
- `3`: Servo not enabled
- And more...

## Future Enhancements

1. **Velocity Control**: Currently placeholders - needs implementation using `vc_set_joint_velocity()`
2. **Joint Velocity Computation**: Currently returns zeros - could compute from position differences
3. **Effort Reading**: Currently zeros - xArm SDK may not provide direct torque reading
4. **Trajectory Following**: Could add spline interpolation for smooth trajectories
5. **Collision Detection**: Integrate xArm collision sensitivity settings

## Notes

- Compatible with xArm5, xArm6, and xArm7 (set `num_joints` in config)
- Requires xArm Python SDK installed (`pip install xarm-python-sdk`)
- Network connection to xArm required
- Default IP: 192.168.1.185 (configured in xArm settings)
