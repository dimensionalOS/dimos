# ROS Docker Integration for DimOS

This directory contains Docker configuration files to run DimOS and the ROS autonomy stack in the same container, enabling communication between the two systems.

## Prerequisites

1. **Install Docker with `docker compose` support**. Follow the [official Docker installation guide](https://docs.docker.com/engine/install/).
2. **Install NVIDIA GPU drivers**. See [NVIDIA driver installation](https://www.nvidia.com/download/index.aspx).
3. **Install NVIDIA Container Toolkit**. Follow the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

## Quick Start

1. **Build the Docker image:**
   ```bash
   ./build.sh
   ```
   This will:
   - Clone the ros-navigation-autonomy-stack repository (jazzy branch)
   - Build a Docker image with both ROS and DimOS dependencies
   - Set up the environment for both systems

   Note that the build will take over 10 minutes and build an image over 30GiB.

2. **Run the container:**
   ```bash
   ./start.sh --all
   ```

## Manual Commands

Once inside the container, you can manually run:

### ROS Autonomy Stack
```bash
cd /ros2_ws/src/ros-navigation-autonomy-stack
./system_simulation_with_route_planner.sh
```

### DimOS
```bash
# Activate virtual environment
source /opt/dimos-venv/bin/activate

# Run navigation bot
python /workspace/dimos/dimos/navigation/rosnav/nav_bot.py

# Or run other DimOS scripts
python /workspace/dimos/dimos/your_script.py
```

### ROS Commands
```bash
# List ROS topics
ros2 topic list

# Send navigation goal
ros2 topic pub /way_point geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: 5.0, y: 3.0, z: 0.0}
}" --once

# Monitor robot state
ros2 topic echo /state_estimation
```

## Custom Commands

Use the `run_command.sh` helper script to run custom commands:
```bash
./run_command.sh "ros2 topic list"
./run_command.sh "python /workspace/dimos/dimos/your_script.py"
```

## Development

The docker-compose.yml mounts the following directories for live development:
- DimOS source: `..` → `/workspace/dimos`
- Autonomy stack source: `./ros-navigation-autonomy-stack/src` → `/ros2_ws/src/ros-navigation-autonomy-stack/src`

Changes to these files will be reflected in the container without rebuilding.

**Note**: The Python virtual environment is installed at `/opt/dimos-venv` inside the container (not in the mounted `/workspace/dimos` directory). This ensures the container uses its own dependencies regardless of whether the host has a `.venv` or not.