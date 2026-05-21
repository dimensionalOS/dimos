# Go2 Patrol Package

## Overview
This package provides autonomous patrol functionality for the Unitree Go2 robot using ROS 2 and Nav2. The robot navigates through a series of predefined waypoints in a loop, stopping at each waypoint for a configurable duration.

## Features
- **Waypoint-based patrol**: Navigate through multiple waypoints in sequence
- **Configurable stop duration**: Set how long the robot stops at each waypoint
- **Loop mode**: Continuously patrol or run once
- **Failure handling**: Retry failed waypoints with configurable attempts
- **YAML configuration**: Define waypoints in an easy-to-edit YAML file

## Usage

### 1. Build the package
```bash
cd /home/utente/lab_ws/unitree-go2-slam-nav2/unitree_go2_slam_nav2
colcon build --packages-select go2_patrol
source install/setup.bash
```

### 2. Configure waypoints
Edit the waypoints file at `config/waypoints.yaml` with your desired patrol route coordinates.

### 3. Run the patrol node

**Basic usage with default waypoints:**
```bash
ros2 launch go2_patrol patrol.launch.py
```

**With custom waypoints file:**
```bash
ros2 launch go2_patrol patrol.launch.py waypoints_file:=/path/to/your/waypoints.yaml
```

**With custom parameters:**
```bash
ros2 launch go2_patrol patrol.launch.py \
    waypoints_file:=/path/to/waypoints.yaml \
    loop_patrol:=true \
    stop_duration:=10.0 \
    max_retries:=5
```

**Run without looping (single patrol):**
```bash
ros2 launch go2_patrol patrol.launch.py loop_patrol:=false
```

### 4. Run directly as a node (without launch file):
```bash
ros2 run go2_patrol patrol_node --ros-args \
    -p waypoints_file:=/path/to/waypoints.yaml \
    -p loop_patrol:=true \
    -p stop_duration:=5.0
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `waypoints_file` | string | config/waypoints.yaml | Path to YAML file with waypoint coordinates |
| `loop_patrol` | bool | true | Whether to continuously loop through waypoints |
| `stop_duration` | double | 5.0 | Time in seconds to stop at each waypoint |
| `max_retries` | int | 3 | Maximum retry attempts for failed waypoints |

## Waypoint Configuration

Waypoints are defined in YAML format with position (x, y, z) and orientation (quaternion):

```yaml
waypoints:
  - x: 2.0
    y: 0.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0
  - x: 2.0
    y: 2.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.707
    qw: 0.707
```

## Integration with Go2 Navigation Stack

This patrol package works with the existing Go2 SLAM and Nav2 setup:

1. **Start the robot and sensors**
2. **Launch SLAM** (if mapping) or **load a map** (if using pre-built map)
3. **Launch Nav2** navigation stack
4. **Launch patrol node** (this package)

Example full workflow:
```bash
# Terminal 1: Launch robot interface and sensors
ros2 launch go2_slam_nav robot.launch.py

# Terminal 2: Launch Nav2 with your map
ros2 launch go2_slam_nav navigation.launch.py map:=/path/to/your/map.yaml

# Terminal 3: Start patrol
ros2 launch go2_patrol patrol.launch.py waypoints_file:=/path/to/waypoints.yaml
```

## Determining Waypoint Coordinates

To get waypoint coordinates for your environment:

1. Use RViz to visualize your map
2. Click "2D Pose Estimate" or "Nav2 Goal" to see coordinates
3. Record the x, y coordinates and orientation from the terminal output
4. Add these to your `waypoints.yaml` file

## Dependencies
- ROS 2 (Humble or later)
- Nav2
- rclpy
- nav2_msgs
- geometry_msgs
- PyYAML

## License
Apache License 2.0
