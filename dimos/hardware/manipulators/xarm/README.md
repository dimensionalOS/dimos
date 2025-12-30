# xArm Driver for dimos

Real-time driver for UFACTORY xArm5/6/7 manipulators integrated with the dimos framework.

## Quick Start

### 1. Specify Robot IP

**On boot** (Important)
```bash
sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
```

**Option A: Command-line argument** (recommended)
```bash
python test_xarm_driver.py --ip 192.168.1.235
python interactive_control.py --ip 192.168.1.235
```

**Option B: Environment variable**
```bash
export XARM_IP=192.168.1.235
python test_xarm_driver.py
```

**Option C: Use default** (192.168.1.235)
```bash
python test_xarm_driver.py  # Uses default
```

**Note:** Command-line `--ip` takes precedence over `XARM_IP` environment variable.

### 2. Basic Usage

```python
from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.msgs.sensor_msgs import JointState, JointCommand

# Start dimos and deploy driver
dimos = core.start(1)
xarm = dimos.deploy(XArmDriver, ip_address="192.168.1.235", xarm_type="xarm6")

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
- `xarm_type`: Robot model - "xarm5", "xarm6", or "xarm7" (default: "xarm6")
- `control_frequency`: Control loop rate in Hz (default: 100.0)
- `is_radian`: Use radians vs degrees (default: True)
- `enable_on_start`: Auto-enable servo mode (default: True)
- `velocity_control`: Use velocity vs position mode (default: False)

## Testing

### With Mock Hardware (No Physical Robot)

```bash
# Unit tests with mocked xArm hardware
python tests/test_xarm_rt_driver.py
```

### With Real Hardware

**⚠️ Note:** Interactive control and hardware tests require a physical xArm connected to the network. Interactive control, and sample_trajectory_generator are part of test suite, and will be deprecated.

**Using Alfred Embodiment:**

To test with real hardware using the current Alfred embodiment:

1. **Turn on the Flowbase** (xArm controller)
2. **SSH into dimensional-cpu-2:**
   ```
3. **Verify PC is connected to the controller:**
   ```bash
   ping 192.168.1.235  # Should respond
   ```
4. **Run the interactive control:**
   ```bash
   # Interactive control (recommended)
   venv/bin/python dimos/hardware/manipulators/xarm/interactive_control.py --ip 192.168.1.235

   # Run driver standalone
   venv/bin/python dimos/hardware/manipulators/xarm/test_xarm_driver.py --ip 192.168.1.235

   # Run automated test suite
   venv/bin/python dimos/hardware/manipulators/xarm/test_xarm_driver.py --ip 192.168.1.235 --run-tests

   # Specify xArm model type (if using xArm7)
   venv/bin/python dimos/hardware/manipulators/xarm/interactive_control.py --ip 192.168.1.235 --type xarm7
   ```

## License

Copyright 2025 Dimensional Inc. - Apache License 2.0
