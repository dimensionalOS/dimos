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

## Automated Quick Start

This is an optimistic overview. Use the commands below for an in depth version.

**Build the Docker image:**

```bash
cd docker/navigation
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

## Manual build

Go to the docker dir and clone the ROS navigation stack.

```bash
cd docker/navigation
git clone -b jazzy git@github.com:dimensionalOS/ros-navigation-autonomy-stack.git
```

Download a [Unity environment model for the Mecanum wheel platform](https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs?usp=sharing) and unzip the files to `unity_models`.

Alternativelly, extract `office_building_1` from LFS:

```bash
tar -xf ../../data/.lfs/office_building_1.tar.gz
mv office_building_1 unity_models
```

Then, go back to the root and build the docker image:

```bash
cd ../..
docker compose -f docker/navigation/docker-compose.yml build
```

## On Real Hardware

### Configure the WiFi

[Read this](https://github.com/dimensionalOS/ros-navigation-autonomy-stack/tree/jazzy?tab=readme-ov-file#transmitting-data-over-wifi) to see how to configure the WiFi.

### Configure the Livox Lidar

The MID360_config.json file is automatically generated on container startup based on your environment variables (LIDAR_COMPUTER_IP and LIDAR_IP).

### Copy Environment Template
```bash
cp .env.hardware .env
```

### Edit `.env` File

Key configuration parameters:

```bash
# Lidar Configuration
LIDAR_INTERFACE=eth0              # Your ethernet interface (find with: ip link show)
LIDAR_COMPUTER_IP=192.168.1.5    # Computer IP on the lidar subnet
LIDAR_GATEWAY=192.168.1.1        # Gateway IP address for the lidar subnet
LIDAR_IP=192.168.1.116           # Full IP address of your Mid-360 lidar
ROBOT_IP=                        # IP addres of robot on local network (if using WebRTC connection) 

# Motor Controller
MOTOR_SERIAL_DEVICE=/dev/ttyACM0  # Serial device (check with: ls /dev/ttyACM*)
```

### Start the Container

Start the container and leave it open.

```bash
./start.sh --hardware
```

It doesn't do anything by default. You have to run commands on it by `exec`-ing:

```bash
docker exec -it dimos_hardware_container bash
```

### In the container

In the container to run the full navigation stack you must run both the dimensional python runfile with connection module and the navigation stack.

#### Dimensional Python + Connection Module

For the Unitree G1 
```bash
dimos run unitree-g1
ROBOT_IP=XX.X.X.XXX dimos run unitree-g1 # If ROBOT_IP env variable is not set in .env  
```

#### Navigation Stack 

```bash
cd /ros2_ws/src/ros-navigation-autonomy-stack
./system_real_robot_with_route_planner.sh
```

Now you can place goal points/poses in RVIZ by clicking the "Goalpoint" button. The robot will navigate to the point, running both local and global planners for dynamic obstacle avoidance. 

