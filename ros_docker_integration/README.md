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
   - Clone the autonomy_stack_mecanum_wheel_platform repository (jazzy branch)
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