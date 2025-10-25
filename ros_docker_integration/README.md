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

   # Start with DimOS navigation bot
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
├── debug.sh              # Debug script to check paths and setup
├── debug_paths.sh        # Internal debug script (runs in container)
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
- Autonomy stack source: `./autonomy_stack_mecanum_wheel_platform/src` → `/ros2_ws/src/autonomy_stack_mecanum_wheel_platform/src`

Changes to these files will be reflected in the container without rebuilding.

**Note**: The Python virtual environment is installed at `/opt/dimos-venv` inside the container (not in the mounted `/workspace/dimos` directory). This ensures the container uses its own dependencies regardless of whether the host has a `.venv` or not.

## Environment Variables

The container sets:
- `ROS_DISTRO=jazzy`
- `ROBOT_CONFIG_PATH=mechanum_drive`
- `ROS_DOMAIN_ID=0`
- `DIMOS_PATH=/workspace/dimos`
- Python venv: `/opt/dimos-venv`
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

### DimOS Not Starting
If DimOS doesn't start when running `./start.sh --all`:
1. Run the debug script to check paths: `./debug.sh`
2. Rebuild the image: `./build.sh`
3. Check if nav_bot.py exists at `/workspace/dimos/dimos/navigation/rosnav/nav_bot.py`
4. Verify the Python virtual environment exists at `/opt/dimos-venv` (not in `/workspace/dimos/.venv`)
5. The container uses its own Python environment - host `.venv` is not needed

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