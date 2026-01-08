# Quest3 VR Teleoperation

VR teleoperation for Quest 3 headset with robot manipulation.

## Quick Start

### 1. Start Teleoperation

```python
from dimos.teleop.quest3 import Quest3TeleopModule

# Create and start
teleop = Quest3TeleopModule()
teleop.start()

# Output:
# Quest3 Teleoperation Module started on https://0.0.0.0:8443
# Open this URL on Quest 3: https://<your-ip>:8443/
```

### 2. Connect Quest 3

1. **Find your server IP**:
   ```bash
   hostname -I
   # Example: 10.0.0.217
   ```

2. **Open Quest 3 browser** and go to:
   ```
   https://10.0.0.217:8443/
   ```

3. **Accept security warning** (self-signed certificate is safe)

4. **Click "Connect"** button and put on headset

5. **Press X button** on controller to start teleoperation
   - First press: Calibrates (captures initial poses) and starts control
   - Press again: Stops control (calibration preserved)
   - Press again: Resumes control

## Architecture

```
Quest 3 Browser
    ↓ Opens https://your-ip:8443/ (WebXR)
    ↓ X button → calibrate_vr() via WebSocket
    ↓
FastAPI Server
    ↓ Receives tracking data (controller poses)
    ↓
Quest3TeleopModule (inherits BaseTeleopModule)
    ↓ Computes delta poses (current - initial)
    ↓ Publishes delta poses (PoseStamped)
    ↓
TeleopRobotController
    ↓ Auto-calibrates robot on first delta
    ↓ Applies: target = initial_robot + delta
    ↓ Publishes cartesian commands (Pose)
    ↓
Robot Driver
    ↓ Executes commands
```


## Key Features

Press X to start/stop teleoperation
Captures initial poses on first X press
Robot follows controller movement relative to initial pose
AR mode

## Configuration

```python
from dimos.teleop.quest3 import Quest3TeleopModule, Quest3TeleopConfig

config = Quest3TeleopConfig(
    # Quest3-specific settings
    signaling_host="0.0.0.0",
    signaling_port=8443,           # HTTPS port (required for WebXR)
    use_https=True,                # Required for Quest 3 WebXR

    # Inherited from BaseTeleopConfig
    enable_left_arm=True,
    enable_right_arm=True,
    visualize_in_rerun=True,       # Visualize controllers in Rerun
    position_scale=1.0,
    max_velocity=0.5,
    workspace_limits={
        "x": (-1.0, 1.0),
        "y": (-0.8, 0.8),
        "z": (0.1, 2.0),
    },
)

module = Quest3TeleopModule(config=config)
module.start()
```


## WebSocket Protocol

### Handshake
```json
// Client → Server
{
  "role": "teleop",
  "robot_ip": ""
}

// Server → Client
{
  "type": "handshake_ack",
  "status": "connected"
}
```

### X Button Commands
```json
// Client → Server (when X button pressed)
{
  "type": "start_teleop"  // or "stop_teleop"
}

// Server → Client (response)
{
  "type": "teleop_started",  // or "teleop_stopped"
  "message": "Control active - move controllers to control robot"
}
```

## Output Topics

Inherited from `BaseTeleopModule`:

- `left_controller_delta` (PoseStamped) - Left controller **delta** pose (current - initial)
- `right_controller_delta` (PoseStamped) - Right controller **delta** pose (current - initial)
- `left_trigger` (Bool) - Left trigger button state
- `right_trigger` (Bool) - Right trigger button state

**Note**: These are **delta poses**, not absolute poses. TeleopRobotController applies these deltas to the robot's initial pose.


## Development

### Standalone Server Testing
```bash
cd dimos/teleop/quest3/control
python fastapi_server.py

# Server starts on https://0.0.0.0:8443
# Test: https://localhost:8443/health
```

### Custom Integration

```python
from dimos.core.blueprints import autoconnect
from dimos.teleop.quest3 import quest3_teleop_module
from dimos.teleop import teleop_robot_controller
from your_robot import your_robot_blueprint

custom_stack = autoconnect(
    quest3_teleop_module(
        signaling_port=8443,
        enable_left_arm=True,
        enable_right_arm=False,
    ),
    teleop_robot_controller(
        driver_module_name="YourDriver",
        enable_left_arm=True,
        enable_right_arm=False,
    ),
    your_robot_blueprint,
)

coordinator = custom_stack.build()
coordinator.loop()
```

## Related

- [`BaseTeleopModule`](../base/) - Base class for all teleoperation devices
- [`TeleopRobotController`](../teleop_robot_controller.py) - Applies deltas to robot
