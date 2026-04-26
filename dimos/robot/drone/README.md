# DimOS Drone Module

DJI drone integration via RosettaDrone MAVLink bridge, with visual servoing, autonomous tracking, and LLM agent control.

## Quick Start

```bash
# Replay mode (no hardware needed)
dimos --replay run drone-basic

# Agentic mode with replay
dimos --replay run drone-agentic

# Real drone — indoor (velocity-based odometry)
dimos run drone-basic

# Real drone — outdoor (GPS-based odometry)
dimos run drone-basic --set outdoor=true

# Agentic with LLM control
dimos run drone-agentic
```

To interact with the agent, run `dimos humancli` in a separate terminal.

## Blueprints

### `drone-basic`
Connection + camera + visualization. The foundation layer.

| Module | Purpose |
|--------|---------|
| `DroneConnectionModule` | MAVLink communication, movement skills |
| `DroneCameraModule` | Camera intrinsics, image processing |
| `WebsocketVisModule` | Web-based visualization |
| `RerunBridgeModule` / `FoxgloveBridge` | 3D viewer (selected by `--viewer`) |

**Indoor vs Outdoor:** By default, the drone uses velocity integration for odometry (indoor mode). For outdoor flights with GPS, set `outdoor=true` — this switches to GPS-only positioning which is more reliable in open environments but less precise for close-range maneuvers.

### `drone-agentic`
Composes on top of `drone-basic`, adding autonomous capabilities:

| Module | Purpose |
|--------|---------|
| `DroneTrackingModule` | Visual servoing & object tracking |
| `GoogleMapsSkillContainer` | GPS-based navigation skills |
| `OsmSkill` | OpenStreetMap queries |
| `McpServer` + `McpClient` | LLM agent (default: GPT-4o) via MCP |
| `WebInput` | Web/CLI interface for human commands |

## Installation

### Python (included with DimOS)
```bash
pip install -e ".[drone]"
```

### System Dependencies
```bash
# GStreamer for video streaming
sudo apt-get install -y gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-libav python3-gi python3-gi-cairo

# LCM for communication
sudo apt-get install liblcm-dev
```

### Environment
```bash
# Required for agentic blueprint
export OPENAI_API_KEY=sk-...

# Optional
export GOOGLE_MAPS_API_KEY=...  # For GoogleMapsSkillContainer
```

## RosettaDrone Setup (Critical)

RosettaDrone is an Android app that bridges DJI SDK to MAVLink protocol. Without it, the drone cannot communicate with DimOS.

### Option 1: Pre-built APK
1. Download latest release: https://github.com/RosettaDrone/rosettadrone/releases
2. Install on Android device connected to DJI controller
3. Configure in app:
   - MAVLink Target IP: Your computer's IP
   - MAVLink Port: 14550
   - Video Port: 5600
   - Enable video streaming

### Option 2: Build from Source

#### Prerequisites
- Android Studio
- DJI Developer Account: https://developer.dji.com/
- Git

#### Build Steps
```bash
# Clone repository
git clone https://github.com/RosettaDrone/rosettadrone.git
cd rosettadrone

# Build with Gradle
./gradlew assembleRelease

# APK will be in: app/build/outputs/apk/release/
```

#### Configure DJI API Key
1. Register app at https://developer.dji.com/user/apps
   - Package name: `sq.rogue.rosettadrone`
2. Add key to `app/src/main/AndroidManifest.xml`:
```xml
<meta-data
    android:name="com.dji.sdk.API_KEY"
    android:value="YOUR_API_KEY_HERE" />
```

#### Install APK
```bash
adb install -r app/build/outputs/apk/release/rosettadrone-release.apk
```

### Hardware Connection
```
DJI Drone ← Wireless → DJI Controller ← USB → Android Device ← WiFi → DimOS Computer
```

1. Connect Android to DJI controller via USB
2. Start RosettaDrone app
3. Wait for "DJI Connected" status
4. Verify "MAVLink Active" shows in app

## Architecture

### Module Structure
```
dimos/robot/drone/
├── blueprints/
│   ├── basic/drone_basic.py              # Base blueprint (connection + camera + vis)
│   └── agentic/drone_agentic.py          # Agentic blueprint (composes on basic)
├── connection_module.py                   # MAVLink communication & skills
├── camera_module.py                       # Camera processing & intrinsics
├── drone_tracking_module.py               # Visual servoing & object tracking
├── drone_visual_servoing_controller.py    # PID-based visual servoing
├── mavlink_connection.py                  # Low-level MAVLink protocol
└── dji_video_stream.py                    # GStreamer video capture + replay
```

### Communication Flow
```
DJI Drone → RosettaDrone → MAVLink UDP → connection_module → LCM Topics
                         → Video UDP   → dji_video_stream → tracking_module
```

### LCM Topics
- `/video` — Camera frames (`sensor_msgs.Image`)
- `/odom` — Position and orientation (`geometry_msgs.PoseStamped`)
- `/movecmd_twist` — Velocity commands (`geometry_msgs.Twist`)
- `/gps_location` — GPS coordinates (`LatLon`)
- `/gps_goal` — GPS navigation target (`LatLon`)
- `/tracking_status` — Tracking module state
- `/follow_object_cmd` — Object tracking commands
- `/color_image` — Processed camera image
- `/camera_info` — Camera intrinsics
- `/camera_pose` — Camera pose in world frame

## Visual Servoing & Tracking

### Object Tracking
```python
# Track specific object
result = drone.tracking.track_object("red flag", duration=60)

# Track nearest/most prominent object
result = drone.tracking.track_object(None, duration=60)

# Stop tracking
drone.tracking.stop_tracking()
```

### PID Tuning
```python
# Indoor (gentle, precise)
x_pid_params=(0.001, 0.0, 0.0001, (-0.5, 0.5), None, 30)

# Outdoor (aggressive, wind-resistant)
x_pid_params=(0.003, 0.0001, 0.0002, (-1.0, 1.0), None, 10)
```

Parameters: `(Kp, Ki, Kd, (min_output, max_output), integral_limit, deadband_pixels)`

### Visual Servoing Flow
1. Qwen model detects object → bounding box
2. CSRT tracker initialized on bbox
3. PID controller computes velocity from pixel error
4. Velocity commands sent via LCM stream
5. Connection module converts to MAVLink commands

## Available Skills

All skills are exposed to the LLM agent via the `@skill` decorator on `DroneConnectionModule`:

### Movement & Control
- `move(x, y, z, duration)` — Move with velocity (m/s)
- `takeoff(altitude)` — Takeoff to altitude
- `land()` — Land at current position
- `arm()` / `disarm()` — Arm/disarm motors
- `set_mode(mode)` — Set flight mode (GUIDED, LOITER, etc.)
- `fly_to(lat, lon, alt)` — Fly to GPS coordinates

### Perception
- `observe()` — Get current camera frame
- `follow_object(description, duration)` — Follow object with visual servoing
- `is_flying_to_target()` — Check if navigating to GPS target

## Replay Mode

Replay data includes:
- **2,148 video frames** (640×360 RGB, ~71s at 30fps)
- **4,098 MAVLink telemetry frames** (~136s)

Stored as `TimedSensorStorage` pickle files in `data/drone/`. Downloaded automatically on first use.

```bash
# Basic replay
dimos --replay run drone-basic

# Agentic replay (requires OPENAI_API_KEY)
dimos --replay run drone-agentic
```

## Visualization

### Rerun Viewer (Recommended)
```bash
dimos --viewer rerun run drone-basic
```
Split layout with camera feed + 3D world view. Includes static drone body visualization and LCM transport integration.

### Foxglove Studio
```bash
dimos --viewer foxglove run drone-basic
```
Connect Foxglove Studio to `ws://localhost:8765` to see:
- Live video with tracking overlay
- 3D drone position
- Telemetry plots
- Transform tree

### Web Visualization
Always available at `http://localhost:7779` via `WebsocketVisModule`.

## Testing

```bash
# Unit tests
pytest -s dimos/robot/drone/

# Replay integration test
dimos --replay run drone-basic
```

## Troubleshooting

### No MAVLink Connection
- Check Android and computer are on same network
- Verify IP address in RosettaDrone matches computer
- Test with: `nc -lu 14550` (should see data)
- Check firewall: `sudo ufw allow 14550/udp`

### No Video Stream
- Enable video in RosettaDrone settings
- Test with: `nc -lu 5600` (should see data)
- Verify GStreamer installed: `gst-launch-1.0 --version`

### Tracking Issues
- Increase lighting for better detection
- Adjust PID gains for environment
- Check `max_lost_frames` in tracking module

### Agent Not Responding
- Check `OPENAI_API_KEY` is set
- Run `dimos humancli` to send commands
- Check logs for `on_system_modules` errors

### Wrong Movement Direction
- Don't modify coordinate conversions
- Verify with: `pytest test_drone.py::test_ned_to_ros_coordinate_conversion`
- Check camera orientation assumptions

## Network Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 14550 | UDP | MAVLink commands/telemetry |
| 5600 | UDP | Video stream |
| 7779 | WebSocket | DimOS web visualization |
| 8765 | WebSocket | Foxglove bridge |
| 7667 | UDP | LCM messaging |

## Coordinate Systems
- **MAVLink/NED**: X=North, Y=East, Z=Down
- **ROS/DimOS**: X=Forward, Y=Left, Z=Up
- Automatic conversion handled internally

## Modifying PID Control
- Increase Kp for faster response
- Add Ki for steady-state error
- Increase Kd for damping
- Adjust limits for max velocity

## Safety Notes
- Always test in simulator or with propellers removed first
- Set conservative PID gains initially
- Implement geofencing for outdoor flights
- Monitor battery voltage continuously
- Have manual override ready
