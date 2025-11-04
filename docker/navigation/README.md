# ROS Docker Integration for DimOS

This directory contains Docker configuration files to run DimOS and the ROS autonomy stack in the same container, enabling communication between the two systems.

## New Ubuntu Installation

**For fresh Ubuntu systems**, use the automated setup script:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dimos-rosnav-docker/docker/navigation/setup.sh | bash
```

Or download and run locally:

```bash
wget https://raw.githubusercontent.com/dimensionalOS/dimos/dimos-rosnav-docker/docker/navigation/setup.sh
chmod +x setup.sh
./setup.sh
```

**Installation time:** Approximately 20-30 minutes depending on your internet connection.

**After installation, start the demo:**
```bash
cd ~/dimos/docker/navigation
./start.sh --all
```

**Options:**
```bash
./setup.sh --help                    # Show all options
./setup.sh --install-dir /opt/dimos  # Custom installation directory
./setup.sh --skip-build              # Skip Docker image build
```

If the automated script encounters issues, follow the manual setup below.

## Prerequisites

1. **Install Docker with `docker compose` support**. Follow the [official Docker installation guide](https://docs.docker.com/engine/install/).
2. **Install NVIDIA GPU drivers**. See [NVIDIA driver installation](https://www.nvidia.com/download/index.aspx).
3. **Install NVIDIA Container Toolkit**. Follow the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

## Quick Start

**Build the Docker image:**

```bash
./build.sh
```

This will:
- Clone the ros-navigation-autonomy-stack repository (jazzy branch)
- Build a Docker image with both ROS and DimOS dependencies
- Set up the environment for both systems

Note that the build will take over 10 minutes and build an image over 30GiB.

**Run the simulator to test it's working:**

```bash
./start.sh --simulation
```

## On Real Hardware

### Copy Environment Template
```bash
cp .env.hardware .env
```

### Edit `.env` File

Key configuration parameters:

```bash
# Lidar Configuration
LIDAR_INTERFACE=eth0        # Your ethernet interface (find with: ip link show)
LIDAR_IP=192.168.1.5        # Must be 192.168.1.5 for Mid-360
LIDAR_SERIAL=42             # Last 2 digits of lidar serial number

# Motor Controller
MOTOR_SERIAL_DEVICE=/dev/ttyACM0  # Serial device (check with: ls /dev/ttyACM*)
```

### Start the Container

Start the container and leave it open.

```bash
./start.sh --hardware
```

It doesn't do anything by default. You have to run commands on it by exec-ing:

```bash
docker exec -it dimos_hardware_container bash
```

### In the container

In the container you can run any of the ROS or python code.

#### ROS

```bash
cd /ros2_ws/src/ros-navigation-autonomy-stack
./system_real_robot_with_route_planner.sh
```

### Python

Move 2 meters forward, and 2 meters left.

```bash
source /opt/dimos-venv/bin/activate
python3 dimos/navigation/demo_ros_navigation.py
```
