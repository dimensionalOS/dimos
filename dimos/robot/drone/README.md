# DimOS Drone Module

Complete integration for DJI drones via RosettaDrone MAVLink bridge with visual servoing and autonomous tracking capabilities.

## Quick Start

### Test the System
```bash
# Test with replay mode (no hardware needed)
python dimos/robot/drone/drone.py --replay

# Real drone - indoor (IMU odometry)
python dimos/robot/drone/drone.py

# Real drone - outdoor (GPS odometry)
python dimos/robot/drone/drone.py --outdoor
```

### Python API Usage
```python
from dimos.robot.drone.drone import Drone

# Connect to drone
drone = Drone(connection_string='udp:0.0.0.0:14550', outdoor=True)  # Use outdoor=True for GPS
drone.start()

# Basic operations
drone.arm()
drone.takeoff(altitude=5.0)
drone.move(Vector3(1.0, 0, 0), duration=2.0)  # Forward 1m/s for 2s

# Visual tracking
drone.tracking.track_object("person", duration=120)  # Track for 2 minutes

# Land and cleanup
drone.land()
drone.stop()
```

## Installation

### Python Package
```bash
# Install DimOS with drone support
pip install -e .[drone]
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

### Environment Setup
```bash
export DRONE_IP=0.0.0.0  # Listen on all interfaces
export DRONE_VIDEO_PORT=5600
export DRONE_MAVLINK_PORT=14550
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
drone.py                    # Main orchestrator
├── connection_module.py    # MAVLink communication & skills
├── camera_module.py        # Video processing & depth estimation
├── tracking_module.py      # Visual servoing & object tracking
├── mavlink_connection.py   # Low-level MAVLink protocol
└── dji_video_stream.py     # GStreamer video capture
```

### Communication Flow
```
DJI Drone → RosettaDrone → MAVLink UDP → connection_module → LCM Topics
                         → Video UDP → dji_video_stream → tracking_module
```

### LCM Topics
- `/drone/odom` - Position and orientation
- `/drone/status` - Armed state, battery
- `/drone/video` - Camera frames
- `/drone/tracking/cmd_vel` - Tracking velocity commands
- `/drone/tracking/overlay` - Visualization with tracking box

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
Configure in `drone.py` initialization:
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

### Movement & Control
- `move(vector, duration)` - Move with velocity vector
- `takeoff(altitude)` - Takeoff to altitude
- `land()` - Land at current position
- `arm()/disarm()` - Arm/disarm motors
- `fly_to(lat, lon, alt)` - Fly to GPS coordinates

### Perception
- `observe()` - Get current camera frame
- `follow_object(description, duration)` - Follow object with servoing

### Tracking Module
- `track_object(name, duration)` - Track and follow object
- `stop_tracking()` - Stop current tracking
- `get_status()` - Get tracking status

## Testing

### Unit Tests
```bash
pytest -s dimos/robot/drone/
```

### Replay Mode (No Hardware)
```python
# Use recorded data for testing
drone = Drone(connection_string='replay')
drone.start()
# All operations work with recorded data
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
- Monitor with Foxglove on `ws://localhost:8765`

### Wrong Movement Direction
- Don't modify coordinate conversions
- Verify with: `pytest test_drone.py::test_ned_to_ros_coordinate_conversion`
- Check camera orientation assumptions

## Advanced Features

### Coordinate Systems
- **MAVLink/NED**: X=North, Y=East, Z=Down
- **ROS/DimOS**: X=Forward, Y=Left, Z=Up
- Automatic conversion handled internally

### Depth Estimation
Camera module can generate depth maps using Metric3D:
```python
# Depth published to /drone/depth and /drone/pointcloud
# Requires GPU with 8GB+ VRAM
```

### Foxglove Visualization
Connect Foxglove Studio to `ws://localhost:8765` to see:
- Live video with tracking overlay
- 3D drone position
- Telemetry plots
- Transform tree

## Network Ports
- **14550**: MAVLink UDP
- **5600**: Video stream UDP
- **8765**: Foxglove WebSocket
- **7667**: LCM messaging

## Development

### Adding New Skills
Add to `connection_module.py` with `@skill()` decorator:
```python
@skill()
def my_skill(self, param: float) -> str:
    """Skill description for LLM."""
    # Implementation
    return "Result"
```

### Modifying PID Control
Edit gains in `drone.py` `_deploy_tracking()`:
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
