# ROS Docker Integration for DimOS

This directory contains Docker configuration files to run DimOS and the ROS autonomy stack in the same container, enabling communication between the two systems.

## Prerequisites

- Docker with `docker compose` support
- NVIDIA GPU with drivers installed
- NVIDIA Container Toolkit (nvidia-docker2)
- X11 server for GUI applications (RVIZ, Unity simulator)

## Quick Start

1. **Build the Docker image:**
   ```bash
   ./build.sh
   ```
   This will:
   - Clone the autonomy_stack_mecanum_wheel_platform repository (jazzy branch)
   - Build a Docker image with both ROS and DimOS dependencies
   - Set up the environment for both systems

2. **Run the container:**
   ```bash
   # Interactive bash shell (default)
   ./start.sh

   # Start with ROS route planner
   ./start.sh --ros-planner

   # Start with DimOS Unitree G1 controller
   ./start.sh --dimos

   # Start both systems (basic)
   ./start.sh --all

   # Start both systems with improved shutdown handling (recommended)
   ./start_clean.sh --all
   ```

## Directory Structure

```
ros_docker_integration/
├── Dockerfile              # Combined Dockerfile for ROS + DimOS
├── docker-compose.yml      # Docker Compose configuration
├── build.sh               # Script to clone repos and build image
├── start.sh               # Script to run the container (basic)
├── start_clean.sh         # Script with improved shutdown handling
├── run_both.sh            # Bash helper to run both ROS and DimOS
├── ros_launch_wrapper.py  # Python wrapper for clean ROS shutdown
├── run_command.sh         # Helper script for running custom commands
├── shell.sh              # Quick access to interactive shell
├── test_integration.sh    # Integration test script
├── README.md              # This file
├── autonomy_stack_mecanum_wheel_platform/  # (Created by build.sh)
├── unity_models/          # (Optional) Unity environment models
├── bagfiles/             # (Optional) ROS bag files
└── config/               # (Optional) Configuration files
```

## Unity Models (Optional)

For the Unity simulator to work properly, download the Unity environment models from:
https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs

Extract them to: `ros_docker_integration/unity_models/`

## Manual Commands

Once inside the container, you can manually run:

### ROS Autonomy Stack
```bash
cd /ros2_ws/src/autonomy_stack_mecanum_wheel_platform
./system_simulation_with_route_planner.sh
```

### DimOS
```bash
# Activate virtual environment
source /home/p/pro/dimensional/dimos/.venv/bin/activate

# Run Unitree G1 controller
python /home/p/pro/dimensional/dimos/dimos/navigation/rosnav/nav_bot.py

# Or run other DimOS scripts
python /home/p/pro/dimensional/dimos/dimos/your_script.py
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
./run_command.sh "python /path/to/your/script.py"
```

## Development

The docker-compose.yml mounts the following directories for live development:
- DimOS source: `../dimos` → `/home/p/pro/dimensional/dimos/dimos`
- Autonomy stack source: `./autonomy_stack_mecanum_wheel_platform/src` → `/ros2_ws/src/autonomy_stack_mecanum_wheel_platform/src`

Changes to these files will be reflected in the container without rebuilding.

## Environment Variables

The container sets:
- `ROS_DISTRO=jazzy`
- `ROBOT_CONFIG_PATH=unitree/unitree_g1`
- `ROS_DOMAIN_ID=0`
- GPU and display variables for GUI support

## Shutdown Handling

The integration provides two methods for running both systems together:

### Basic Method (`./start.sh --all`)
Uses the bash script `run_both.sh` with signal trapping and process group management.

### Improved Method (`./start_clean.sh --all`)
Uses the Python wrapper `ros_launch_wrapper.py` which provides:
- Proper signal forwarding to ROS launch system
- Graceful shutdown with timeouts
- Automatic cleanup of orphaned ROS nodes
- Better handling of ROS2's complex process hierarchy

**Recommended**: Use `./start_clean.sh --all` for the cleanest shutdown experience.

## Troubleshooting

### ROS Nodes Not Shutting Down Cleanly
If you experience issues with ROS nodes hanging during shutdown:
1. Use `./start_clean.sh --all` instead of `./start.sh --all`
2. The improved handler will automatically clean up remaining processes
3. If issues persist, you can manually clean up with:
   ```bash
   docker compose -f ros_docker_integration/docker-compose.yml down
   ```

### X11 Display Issues
If you get display errors:
```bash
xhost +local:docker
```

### GPU Not Available
Ensure NVIDIA Container Toolkit is installed:
```bash
sudo apt-get install nvidia-container-toolkit
sudo systemctl restart docker
```

### Permission Issues
The container runs with `--privileged` and `--network=host` for hardware access.

## Notes

- The container uses `--network=host` for ROS communication
- GPU passthrough is enabled via `runtime: nvidia`
- X11 forwarding is configured for GUI applications
- The ROS workspace is built without SLAM and Mid-360 packages (simulation mode)