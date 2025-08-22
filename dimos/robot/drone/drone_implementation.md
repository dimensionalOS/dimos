# Drone Implementation Progress

## Overview
Generic MAVLink-based drone implementation for DimOS, tested with DJI drones via RosettaDrone.

## Architecture
- **DroneConnection**: MAVLink wrapper for drone control
- **DroneConnectionModule**: DimOS module with LCM streams  
- **DroneCameraModule**: Video capture and depth estimation
- **Drone**: Main robot class with full integration

## Implementation Progress

### Phase 1: Basic Structure (COMPLETED)
- Created directory structure: `dimos/robot/drone/`
- Subdirectories: `temp/`, `type/`, `multiprocess/`
- Created this documentation file

### Phase 2: Connection Implementation (COMPLETED)
- [x] Create DroneConnection class with MAVLink integration
- [x] Test MAVLink connection - timeout correctly when no drone
- [x] Implement telemetry streams (odom_stream, status_stream)
- [x] Implement movement commands (move, takeoff, land, arm/disarm)

### Phase 3: Module Integration (COMPLETED)
- [x] Create DroneConnectionModule wrapping DroneConnection
- [x] Implement TF transforms (base_link, camera_link)
- [x] Test LCM publishing with proper message types
- [x] Module deploys and configures correctly

### Phase 4: Video Processing (COMPLETED)
- [x] GStreamer video capture via subprocess (gst-launch)
- [x] DroneVideoStream class using gst-launch
- [x] DroneCameraModule with Metric3D depth estimation
- [x] Camera info and pose publishing

### Phase 5: Main Robot Class (COMPLETED)
- [x] Created Drone robot class
- [x] Integrated all modules (connection, camera, visualization)
- [x] LCM transport configuration
- [x] Foxglove bridge integration
- [x] Control methods (takeoff, land, move, arm/disarm)

## Test Files
All temporary test files in `temp/` directory:
- `test_connection.py` - Basic connection test
- `test_real_connection.py` - Real drone telemetry test (WORKING)
- `test_connection_module.py` - Module deployment test (WORKING)
- `test_video_*.py` - Various video capture attempts

## Issues & Solutions

### 1. MAVLink Connection (SOLVED)
- Successfully connects to drone on UDP port 14550
- Receives telemetry: attitude, position, mode, armed status
- Mode changes work (STABILIZE, GUIDED)
- Arm fails with safety on (expected behavior)

### 2. Video Streaming (IN PROGRESS)
- Video stream confirmed on UDP port 5600 (RTP/H.264)
- gst-launch-1.0 command works when run directly:
  ```
  gst-launch-1.0 udpsrc port=5600 ! \
    application/x-rtp,encoding-name=H264,payload=96 ! \
    rtph264depay ! h264parse ! avdec_h264 ! \
    videoconvert ! autovideosink
  ```
- OpenCV with GStreamer backend not working yet
- Need to implement alternative capture method

## Environment
- Python venv: `/home/dimensional5/dimensional/dimos/venv`
- MAVLink connection: `udp:0.0.0.0:14550`
- Video stream: UDP port 5600

## Final Implementation Summary

### Completed Components
1. **DroneConnection** (`connection.py`) - MAVLink communication layer
   - Connects to drone via UDP
   - Receives telemetry (attitude, position, status)
   - Sends movement commands
   - Supports arm/disarm, takeoff/land, mode changes

2. **DroneConnectionModule** (`connection_module.py`) - DimOS module wrapper
   - Publishes odometry and status via LCM
   - Handles TF transforms (base_link, camera_link)
   - RPC methods for drone control

3. **DroneVideoStream** (`video_stream.py`) - Video capture
   - Uses gst-launch subprocess for H.264 decoding
   - Handles RTP/UDP stream on port 5600
   - Publishes Image messages

4. **DroneCameraModule** (`camera_module.py`) - Camera processing
   - Captures video stream
   - Generates depth with Metric3D
   - Publishes camera info and poses

5. **Drone** (`drone.py`) - Main robot class
   - Integrates all modules
   - Configures LCM transports
   - Provides high-level control interface
   - Foxglove visualization support

### Usage

#### Quick Start
```bash
# Set environment variables (optional)
export DRONE_CONNECTION="udp:0.0.0.0:14550"
export DRONE_VIDEO_PORT=5600

# Run the multiprocess example
python dimos/robot/drone/multiprocess/drone.py
```

#### Python API
```python
from dimos.robot.drone import Drone

# Create and start drone
drone = Drone(connection_string='udp:0.0.0.0:14550')
drone.start()

# Control drone
drone.arm()
drone.takeoff(3.0)  # 3 meters
drone.move(Vector3(1, 0, 0), duration=2)  # Forward 1m/s for 2s
drone.land()
```

### Testing
- Unit tests: `pytest dimos/robot/drone/test_drone.py`
- Real drone test: `python dimos/robot/drone/temp/test_real_connection.py`
- Full system test: `python dimos/robot/drone/temp/test_full_system.py`

### Known Issues & Limitations
1. Video capture requires gst-launch-1.0 installed
2. Arm command fails when safety is on (expected)
3. GPS coordinates show as 0,0 until GPS lock acquired
4. Battery telemetry often returns 0 with RosettaDrone

### LCM Topics
- `/drone/odom` - Odometry (PoseStamped)
- `/drone/status` - Status JSON (String)
- `/drone/color_image` - RGB video (Image)
- `/drone/depth_image` - Depth map (Image)
- `/drone/depth_colorized` - Colorized depth (Image)
- `/drone/camera_info` - Camera calibration (CameraInfo)
- `/drone/camera_pose` - Camera pose (PoseStamped)
- `/drone/cmd_vel` - Movement commands (Vector3)

### Foxglove Visualization
Access at http://localhost:8765 when system is running.
Displays video, depth, odometry, and TF transforms.