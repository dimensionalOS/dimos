# Autonomy Stack API Documentation

## Prerequisites

- Ubuntu 24.04
- [ROS 2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)

Add the following line to your `~/.bashrc` to source the ROS 2 Jazzy setup script automatically:

``` echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc```

## MID360 Ethernet Configuration (skip for sim)

### Step 1: Configure Network Interface

1. Open Network Settings in Ubuntu
2. Find your Ethernet connection to the MID360
3. Click the gear icon to edit settings
4. Go to IPv4 tab
5. Change Method from "Automatic (DHCP)" to "Manual"
6. Add the following settings:
   - **Address**: 192.168.1.5
   - **Netmask**: 255.255.255.0
   - **Gateway**: 192.168.1.1
7. Click "Apply"

### Step 2: Configure MID360 IP in JSON

1. Find your MID360 serial number (on sticker under QR code)
2. Note the last 2 digits (e.g., if serial ends in 89, use 189)
3. Edit the configuration file:

```bash
cd ~/autonomy_stack_mecanum_wheel_platform
nano src/utilities/livox_ros_driver2/config/MID360_config.json
```

4. Update line 28 with your IP (192.168.1.1xx where xx = last 2 digits):

```json
"ip" : "192.168.1.1xx",
```

5. Save and exit

### Step 3: Verify Connection

```bash
ping 192.168.1.1xx  # Replace xx with your last 2 digits
```

## Robot Configuration

### Setting Robot Type

The system supports different robot configurations. Set the `ROBOT_CONFIG_PATH` environment variable to specify which robot configuration to use:

```bash
# For Unitree G1 (default if not set)
export ROBOT_CONFIG_PATH="unitree/unitree_g1"

# Add to ~/.bashrc to make permanent
echo 'export ROBOT_CONFIG_PATH="unitree/unitree_g1"' >> ~/.bashrc
```

Available robot configurations:
- `unitree/unitree_g1` - Unitree G1 robot (default)
- Add your custom robot configs in `src/base_autonomy/local_planner/config/`

## Build the system

You must do this every you make a code change, this is not Python

```colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```

## System Launch

### Simulation Mode

```bash
cd ~/autonomy_stack_mecanum_wheel_platform

# Base autonomy only
./system_simulation.sh

# With route planner
./system_simulation_with_route_planner.sh

# With exploration planner
./system_simulation_with_exploration_planner.sh
```

### Real Robot Mode

```bash
cd ~/autonomy_stack_mecanum_wheel_platform

# Base autonomy only
./system_real_robot.sh

# With route planner
./system_real_robot_with_route_planner.sh

# With exploration planner
./system_real_robot_with_exploration_planner.sh
```

## Quick Troubleshooting

- **Cannot ping MID360**: Check Ethernet cable and network settings
- **SLAM drift**: Press clear-terrain-map button on joystick controller
- **Joystick not recognized**: Unplug and replug USB dongle


## ROS Topics

### Input Topics (Commands)

| Topic | Type | Description |
|-------|------|-------------|
| `/way_point` | `geometry_msgs/PointStamped` | Send navigation goal (position only) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Send goal with orientation |
| `/cancel_goal` | `std_msgs/Bool` | Cancel current goal (data: true) |
| `/joy` | `sensor_msgs/Joy` | Joystick input |
| `/stop` | `std_msgs/Int8` | Soft Stop (2=stop all commmand, 0 = release) |
| `/navigation_boundary` | `geometry_msgs/PolygonStamped` | Set navigation boundaries |
| `/added_obstacles` | `sensor_msgs/PointCloud2` | Virtual obstacles |

### Output Topics (Status)

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `nav_msgs/Odometry` | Robot pose from SLAM |
| `/registered_scan` | `sensor_msgs/PointCloud2` | Aligned lidar point cloud |
| `/terrain_map` | `sensor_msgs/PointCloud2` | Local terrain map |
| `/terrain_map_ext` | `sensor_msgs/PointCloud2` | Extended terrain map |
| `/path` | `nav_msgs/Path` | Local path being followed |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to motors |
| `/goal_reached` | `std_msgs/Bool` | True when goal reached, false when cancelled/new goal |

### Map Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/overall_map` | `sensor_msgs/PointCloud2` | Global map (only in sim)|
| `/registered_scan` | `sensor_msgs/PointCloud2` | Current scan in map frame |
| `/terrain_map` | `sensor_msgs/PointCloud2` | Local obstacle map |

## Usage Examples

### Send Goal
```bash
ros2 topic pub /way_point geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: 5.0, y: 3.0, z: 0.0}
}" --once
```

### Cancel Goal
```bash
ros2 topic pub /cancel_goal std_msgs/msg/Bool "data: true" --once
```

### Monitor Robot State
```bash
ros2 topic echo /state_estimation
```

## Configuration Parameters

### Vehicle Parameters (`localPlanner`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vehicleLength` | 0.5 | Robot length (m) |
| `vehicleWidth` | 0.5 | Robot width (m) |
| `maxSpeed` | 0.875 | Maximum speed (m/s) |
| `autonomySpeed` | 0.875 | Autonomous mode speed (m/s) |

### Goal Tolerance Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goalReachedThreshold` | 0.3-0.5 | Distance to consider goal reached (m) |
| `goalClearRange` | 0.35-0.6 | Extra clearance around goal (m) |
| `goalBehindRange` | 0.35-0.8 | Stop pursuing if goal behind within this distance (m) |
| `omniDirGoalThre` | 1.0 | Distance for omnidirectional approach (m) |

### Obstacle Avoidance

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacleHeightThre` | 0.1-0.2 | Height threshold for obstacles (m) |
| `adjacentRange` | 3.5 | Sensor range for planning (m) |
| `minRelZ` | -0.4 | Minimum relative height to consider (m) |
| `maxRelZ` | 0.3 | Maximum relative height to consider (m) |

### Path Planning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pathScale` | 0.875 | Path resolution scale |
| `minPathScale` | 0.675 | Minimum path scale when blocked |
| `minPathRange` | 0.8 | Minimum planning range (m) |
| `dirThre` | 90.0 | Direction threshold (degrees) |

### Control Parameters (`pathFollower`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lookAheadDis` | 0.5 | Look-ahead distance (m) |
| `maxAccel` | 2.0 | Maximum acceleration (m/s²) |
| `slowDwnDisThre` | 0.875 | Slow down distance threshold (m) |

### SLAM Blind Zones (`feature_extraction_node`)

| Parameter | Mecanum | Description |
|-----------|---------|-------------|
| `blindFront` | 0.1 | Front blind zone (m) |
| `blindBack` | -0.2 | Back blind zone (m) |
| `blindLeft` | 0.1 | Left blind zone (m) |
| `blindRight` | -0.1 | Right blind zone (m) |
| `blindDiskRadius` | 0.4 | Cylindrical blind zone radius (m) |

## Operating Modes

### Mode Control
- **Joystick L2**: Hold for autonomy mode
- **Joystick R2**: Hold to disable obstacle checking

### Speed Control
The robot automatically adjusts speed based on:
1. Obstacle proximity
2. Path complexity
3. Goal distance

## Tuning Guide

### For Tighter Navigation
- Decrease `goalReachedThreshold` (e.g., 0.2)
- Decrease `goalClearRange` (e.g., 0.3)
- Decrease `vehicleLength/Width` slightly

### For Smoother Navigation
- Increase `goalReachedThreshold` (e.g., 0.5)
- Increase `lookAheadDis` (e.g., 0.7)
- Decrease `maxAccel` (e.g., 1.5)

### For Aggressive Obstacle Avoidance
- Increase `obstacleHeightThre` (e.g., 0.15)
- Increase `adjacentRange` (e.g., 4.0)
- Increase blind zone parameters

## Common Issues

### Robot Oscillates at Goal
- Increase `goalReachedThreshold`
- Increase `goalBehindRange`

### Robot Stops Too Far from Goal
- Decrease `goalReachedThreshold`
- Decrease `goalClearRange`

### Robot Hits Low Obstacles
- Decrease `obstacleHeightThre`
- Adjust `minRelZ` to include lower points

## SLAM Configuration

### Localization Mode
Set in `livox_mid360.yaml`:
```yaml
local_mode: true
init_x: 0.0
init_y: 0.0
init_yaw: 0.0
```

### Mapping Performance
```yaml
mapping_line_resolution: 0.1   # Decrease for higher quality
mapping_plane_resolution: 0.2  # Decrease for higher quality
max_iterations: 5               # Increase for better accuracy
```
