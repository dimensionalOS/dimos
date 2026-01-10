# VR Headset Teleoperation

VR teleoperation for headsets (Quest 3 tested).

## Quick Start

### 1. Start Teleoperation

```python
from dimos.teleop.devices.vr_headset import VRTeleopModule

# Create and start
teleop = VRTeleopModule()
teleop.start()

# Output:
# VR Teleoperation Module started on https://0.0.0.0:8443
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
   - Press again: Stops control and resets calibration
   - Press again: Calibrates again and resumes control

## Architecture

```
Quest 3 Browser
    ↓ Opens https://your-ip:8443/ (WebXR)
    ↓ X button → start_teleop/stop_teleop via WebSocket
    ↓
FastAPI Server
    ↓ Receives tracking data (controller poses)
    ↓
VRTeleopModule (inherits BaseTeleopModule)
    ↓ Computes delta poses (current - initial)
    ↓ Publishes delta poses (PoseStamped)
    ↓
TeleopArmController
    ↓ Auto-calibrates robot on first delta
    ↓ Applies: target = initial_robot + delta
    ↓ Publishes cartesian commands (Pose)
    ↓
Robot Driver
    ↓ Executes commands
```


## Key Features

- Press X to start/stop teleoperation (start = calibrate, stop = reset calibration)
- Captures initial poses on start
- Robot follows controller movement relative to initial pose
- AR mode

The module exposes a blueprint helper as `vr_teleop_module`.

## Configuration

```python
from dimos.teleop.devices.vr_headset import VRTeleopModule, VRTeleopConfig

config = VRTeleopConfig(
    # VR headset settings
    signaling_host="0.0.0.0",
    signaling_port=8443,           # HTTPS port (required for WebXR)
    use_https=True,                # Required for Quest 3 WebXR

    # Inherited from BaseTeleopConfig
    num_inputs=2,
    enable_inputs=[True, True],
    input_labels=["left_vr", "right_vr"],
    visualize_in_rerun=True,       # Visualize controllers in Rerun
    log_input_data=False,          # Log input pose/gripper data
    log_input_data_interval=100,   # Log every N publishes when enabled
    position_scale=1.0,
    max_velocity=0.5,
    workspace_limits={
        "x": (-1.0, 1.0),
        "y": (-0.8, 0.8),
        "z": (0.1, 2.0),
    },
)

module = VRTeleopModule(config=config)
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

- `controller_delta_0` (PoseStamped) - Controller 0 **delta** pose (current - initial)
- `controller_delta_1` (PoseStamped) - Controller 1 **delta** pose (current - initial)
- `trigger_value_0` (Float32) - Controller 0 trigger value (0.0-1.0)
- `trigger_value_1` (Float32) - Controller 1 trigger value (0.0-1.0)

**Note**: These are **delta poses**, not absolute poses. TeleopArmController applies these deltas to the robot's initial pose. `input_labels` are used in frame IDs and logging.


## Development

### Standalone Server Testing
```bash
cd dimos/teleop/devices/vr_headset/control
python fastapi_server.py

# Server starts on https://0.0.0.0:8443
# Test: https://localhost:8443/health
```

### Custom Integration

```python
from dimos.core.blueprints import autoconnect
from dimos.teleop.devices.vr_headset import VRTeleopModule
from dimos.teleop.robot_controllers import teleop_arm_controller
from your_robot import your_robot_blueprint

custom_stack = autoconnect(
    VRTeleopModule(
        signaling_port=8443,
        enable_inputs=[True, False],
        input_labels=["left_vr", "right_vr"],
    ),
    teleop_arm_controller(
        driver_module_name="YourDriver",
    ),
    your_robot_blueprint,
)

coordinator = custom_stack.build()
coordinator.loop()
```

## Related

- [`BaseTeleopModule`](../base/) - Base class for all teleoperation devices
- [`TeleopArmController`](../../robot_controllers/teleop_arm_controller.py) - Applies deltas to robot
