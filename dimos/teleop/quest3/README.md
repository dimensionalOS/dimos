# Quest3 VR Teleoperation

**NPM-Free** VR teleoperation system for Quest 3 headset with robot manipulation.

## Quick Start

### 1. Start Teleoperation

```python
from dimos.teleop.quest3 import Quest3TeleopModule

# Create and start
teleop = Quest3TeleopModule()
teleop.start()

# Output:
# ✅ Quest3 Teleoperation Module started on https://0.0.0.0:8443
# 📱 Open this URL on Quest 3: https://<your-ip>:8443/
```

### 2. Connect Quest 3

1. **Find your server IP**:
   ```bash
   hostname -I
   # Example: 192.168.1.100
   ```

2. **Open Quest 3 browser** and go to:
   ```
   https://192.168.1.100:8443/
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
    ↓ Opens: https://your-ip:8443/
    ↓ Loads: Standalone HTML/JS VR client
    ↓ Connects: wss://your-ip:8443/ws
    ↓ X button pressed → sends start_teleop command
    ↓
FastAPI Server (Python, single process)
    ↓ Serves HTML + WebSocket
    ↓ Auto-generates SSL certificates
    ↓ Processes tracking data (20Hz)
    ↓ Routes X button commands to TeleopArmController
    ↓
Quest3TeleopModule
    ↓ Publishes controller pose topics (PoseStamped)
    ↓
TeleopArmController
    ↓ Receives X button command → start_control()
    ↓ Calibrates (captures initial controller + robot poses)
    ↓ Activates control loop (runs continuously, gated by flags)
    ↓ Calculates relative control: target = initial_robot + (current_controller - initial_controller)
    ↓ Publishes cartesian commands (Pose)
    ↓
XArmDriver / Robot Driver
    ↓ Executes cartesian commands
    ↓
Robot
```

## Key Features

✅ **Zero npm/Node.js dependencies** - Pure Python + vanilla JS
✅ **Single HTML file** - No build process, instant updates
✅ **Auto-generated SSL** - Certificates created automatically
✅ **One Python process** - Everything in FastAPI
✅ **X button control** - Press X to start/stop teleoperation
✅ **Auto-calibration** - Captures initial poses on first X press
✅ **Relative control** - Robot follows controller movement relative to initial pose
✅ **20Hz tracking rate** - Low latency controller data
✅ **WebXR passthrough** - AR mode support

## Configuration

```python
from dimos.teleop.quest3 import Quest3TeleopModule, Quest3TeleopConfig

config = Quest3TeleopConfig(
    signaling_host="0.0.0.0",
    signaling_port=8443,           # HTTPS port
    use_https=True,                # Required for WebXR
    driver_module_name="XArmDriver",
    position_scale=1.0,
    enable_left_arm=True,
    enable_right_arm=False,
    max_velocity=0.5,
    workspace_limits={
        "x": (0.1, 1.0),
        "y": (-0.8, 0.8),
        "z": (0.1, 0.7),
    },
)

module = Quest3TeleopModule(config=config)
module.start()
```

## Available Blueprints

### Basic Teleoperation (No Camera)

```bash
# XArm6
dimos run quest3-xarm6

# XArm7
dimos run quest3-xarm7

# Piper Robot
dimos run quest3-piper
```

### With Camera Streaming

```bash
# XArm6 + RealSense D435i
dimos run quest3-xarm6-camera

# XArm7 + RealSense
dimos run quest3-xarm7-camera

# Piper + RealSense
dimos run quest3-piper-camera
```

## Files Structure

```
dimos/teleop/quest3/
├── static/
│   └── index.html              # Standalone VR client (vanilla JS)
├── certs/                      # Auto-generated SSL certificates
│   ├── cert.pem
│   └── key.pem
├── control/
│   ├── fastapi_server.py       # FastAPI server with HTTPS + WebSocket
│   └── tracking_processor.py   # VR tracking data processor
├── teleop_module.py            # Main teleoperation module
├── README.md                   # This file
└── README_NO_NPM.md           # Detailed setup guide
```

## How It Works

1. **Control Loop**: Runs continuously from `start()`, processing controller poses at 20Hz
2. **X Button Press**:
   - Sends `start_teleop` command via WebSocket
   - Triggers `TeleopArmController.start_control()`
   - If not calibrated, automatically calibrates first
   - Then activates control (sets `control_active = True`)
3. **Calibration**: Captures initial controller pose and robot end-effector pose
4. **Relative Control**: Calculates target pose as: `target = initial_robot + (current_controller - initial_controller)`
5. **Safety Gates**: Control loop only sends commands when both `calibrated` and `control_active` are True

## RPC Methods

**Quest3TeleopModule**:
- `start()` → Start module and FastAPI server
- `stop()` → Stop module and server
- `get_status()` → Get current status

**TeleopArmController**:
- `start_control()` → Calibrate (if needed) and activate control
- `stop_control()` → Stop sending commands (preserves calibration)
- `calibrate()` → Manually calibrate
- `is_control_active()` → Check if control is active

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

### Tracking Data (20Hz)
```json
{
  "type": "controller",
  "left": {
    "targetLocation": [16 floats],  // 4x4 transformation matrix
    "trigger": 0.0,                  // 0.0 - 1.0
    "grip": 0.0,
    "joystickX": 0.0,
    "joystickY": 0.0,
    "buttons": [true, false, ...]
  },
  "right": { /* same structure */ }
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

The module publishes these topics:

- `left_controller_pose` (PoseStamped) - Left controller pose
- `right_controller_pose` (PoseStamped) - Right controller pose
- `left_trigger` (Bool) - Left trigger button state
- `right_trigger` (Bool) - Right trigger button state

## Troubleshooting

### WebXR Not Available
**Solution**: Make sure you're using HTTPS (required by Quest 3):
```python
config = Quest3TeleopConfig(use_https=True)  # Must be True!
```

### Certificate Warning on Quest 3
**Solution**: Accept the self-signed certificate:
1. Click "Advanced" on warning page
2. Click "Proceed to <your-ip> (unsafe)"
3. This is safe for local development

### Port Already in Use
```bash
# Check what's using the port
sudo lsof -i :8443

# Kill the process
sudo kill -9 <PID>

# Or use a different port
config = Quest3TeleopConfig(signaling_port=9443)
```

### Can't Generate SSL Certificates
```bash
# Install OpenSSL
sudo apt-get install openssl

# Verify
openssl version
```

### WebSocket Connection Fails
**Check**:
1. Server is running: `curl https://your-ip:8443/health`
2. Firewall allows port: `sudo ufw allow 8443`
3. Same network (Quest 3 and server)
4. Correct IP (not localhost)

### No Tracking Data / Robot Not Moving
1. **Press X button** to start teleoperation (calibrates and activates)
2. Verify WebSocket connection in browser console
3. Check server logs for calibration/control status
4. Ensure controllers are being tracked in VR
5. Check that `control_active` and `calibrated` are both True in logs

## Development

### Edit VR Client
Simply edit `static/index.html` and refresh browser (no build needed!):

```html
<!-- Add custom UI -->
<div id="custom-status">Custom Info</div>

<script>
  // Your custom JavaScript
  console.log('VR client loaded');
</script>
```

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
from your_robot import your_robot_blueprint

custom_stack = autoconnect(
    your_robot_blueprint,
    quest3_teleop_module(
        driver_module_name="YourDriver",
        position_scale=1.0,
    ),
)

coordinator = custom_stack.build()
coordinator.loop()
```

## Performance (Need to be updated)

- **Latency**: ~50-70ms end-to-end (Quest 3 → Python → Robot)
- **Tracking Rate**: 20Hz (50ms interval)
- **CPU Usage**: <5% (FastAPI server)
- **Memory**: ~50MB (Python process)
- **Network**: ~2KB/s (tracking data)
