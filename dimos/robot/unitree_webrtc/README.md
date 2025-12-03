# Unitree WebRTC/Zenoh Architecture

This document describes the new multi-process architecture for Unitree Go2 robot control using WebRTC and Zenoh communication.

## Architecture Overview

The new architecture separates concerns by running WebRTC communication in a dedicated process and using Zenoh for inter-process communication. This design mirrors the ROS control pattern but uses Zenoh instead of ROS topics.

### Components

1. **`run_webrtc.py`** - Standalone WebRTC bridge process
2. **`connection.py`** - Unified WebRTC robot interface with Zenoh communication
3. **`unitree_go2.py`** - Robot implementation using the interface

### Process Separation

```
┌─────────────────┐    Zenoh Topics    ┌──────────────────┐
│   run_webrtc.py │◄──────────────────►│   Robot Process  │
│                 │                    │                  │
│ • WebRTC Conn   │                    │ • Robot Control  │
│ • Sensor Pub    │                    │ • Planning       │
│ • Command Sub   │                    │ • Perception     │
└─────────────────┘                    └──────────────────┘
```

## Zenoh Topic Structure

### Sensor Data (Binary Serialization)
- `sensors/camera/image` - Camera image frames (Image type)
- `sensors/lidar/pointcloud` - LiDAR point cloud data (LidarMessage type)
- `sensors/odometry` - Robot odometry (Odometry type)
- `sensors/lowstate` - Robot low-level state (JSON)

### Command Data (JSON Serialization)
- `commands/movement` - Movement velocity commands
- `commands/api_request` - WebRTC API requests (standup, liedown, etc.)

## Usage

### 1. Start the WebRTC Bridge Process

```bash
python dimos/robot/unitree_webrtc/run_webrtc.py --ip <ROBOT_IP> --mode ai
```

This process:
- Connects to the robot via WebRTC
- Publishes sensor data to Zenoh topics using binary serialization
- Subscribes to command topics from other processes
- Handles the actual robot communication

### 2. Use the Robot in Your Code

```python
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2

# Initialize robot (now connects via Zenoh)
robot = UnitreeGo2(
    ip="192.168.123.161",  # For logging/compatibility
    mode="ai",
    enable_perception=True
)

# Control the robot
robot.standup()
robot.move(x=0.3, y=0.0, yaw=0.0, duration=2.0)
robot.stop()
robot.liedown()

# Access sensor streams
video_stream = robot.get_video_stream()
lidar_stream = robot.lidar_stream
odometry = robot.connection_interface.get_odometry()
```

### 3. Run the Example

```bash
python examples/zenoh_webrtc_robot_example.py
```

## Comparison with ROS Architecture

| Component | ROS Version | WebRTC/Zenoh Version |
|-----------|-------------|---------------------|
| **Control Interface** | `UnitreeROSControl` inherits from `ROSControl` | `WebRTCRobot` with Zenoh communication |
| **Communication** | ROS topics and services | Zenoh topics |
| **Process Model** | Single process with ROS nodes | Multi-process with WebRTC bridge |
| **Sensor Data** | ROS messages | Binary serialized types |
| **Commands** | ROS messages | JSON over Zenoh |
| **Robot Class** | Uses ROS control interface | Uses WebRTC/Zenoh interface |

## Benefits

1. **Process Isolation** - WebRTC connection runs in separate process, isolating failures
2. **Performance** - Binary serialization for high-bandwidth sensor data
3. **Flexibility** - JSON commands are easy to debug and extend
4. **Compatibility** - Same robot interface as ROS version
5. **Scalability** - Multiple robot processes can connect to same WebRTC bridge
6. **Simplicity** - Single unified interface class instead of multiple inheritance layers

## Implementation Details

### Binary Serialization

Large sensor data (images, point clouds, odometry) uses efficient binary serialization:

```python
# Publishing sensor data
image = Image.from_array(frame_array, format="BGR")
binary_data = image.to_zenoh_binary()
publisher.put(binary_data)

# Receiving sensor data  
image = Image.from_zenoh_binary(sample.payload)
frame_array = image.to_array()
```

### JSON Commands

Control commands use JSON for simplicity and debuggability:

```python
# Movement command
command = {
    "x": 0.3,
    "y": 0.0, 
    "yaw": 0.0,
    "duration": 2.0
}
publisher.put(json.dumps(command).encode('utf-8'))

# API request
request = {
    "topic": "rt/api/sport/request",
    "data": {"api_id": 1001}
}
publisher.put(json.dumps(request).encode('utf-8'))
```

## Migration from Old Architecture

To migrate from the old direct WebRTC architecture:

1. **Start WebRTC bridge process** instead of connecting directly
2. **Update imports** to use `WebRTCRobot` from `connection.py`
3. **No code changes** needed in robot usage - interface remains the same
4. **Add process management** to ensure bridge process is running

## Troubleshooting

### Common Issues

1. **"Video stream not available"** - Ensure `run_webrtc.py` is running and connected
2. **"Zenoh connection failed"** - Check that Zenoh is properly installed
3. **"Robot not responding"** - Verify WebRTC bridge process is connected to robot
4. **"Command timeouts"** - Check network connectivity between processes

### Debug Commands

```bash
# Check if WebRTC bridge is publishing sensor data
zenoh -m peer -l /sensors/**

# Check if robot is sending commands  
zenoh -m peer -l /commands/**

# Monitor all Zenoh traffic
zenoh -m peer -l /**
```

## Future Enhancements

1. **State Synchronization** - Share robot state between WebRTC process and control interface
2. **Health Monitoring** - Monitor WebRTC process health and auto-restart
3. **Multi-Robot Support** - Support multiple robots via different Zenoh key expressions  
4. **Configuration Management** - Centralized configuration for both processes
5. **Performance Metrics** - Monitor latency and throughput of Zenoh communication 