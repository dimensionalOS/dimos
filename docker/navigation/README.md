# ROS Docker Integration for DimOS

This directory contains Docker configuration files to run DimOS and the ROS autonomy stack in the same container, enabling communication between the two systems.

## Prerequisites

1. **Install Docker with `docker compose` support**. Follow the [official Docker installation guide](https://docs.docker.com/engine/install/).
2. **Install NVIDIA GPU drivers**. See [NVIDIA driver installation](https://www.nvidia.com/download/index.aspx).
3. **Install NVIDIA Container Toolkit**. Follow the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

## Automated Quick Start

This is an optimistic overview. Use the commands below for an in depth version.

**Build the Docker image:**

```bash
cd docker/navigation
./build.sh --humble    # Build for ROS 2 Humble
./build.sh --jazzy     # Build for ROS 2 Jazzy
```

This will:
- Clone the ros-navigation-autonomy-stack repository
- Build a Docker image with both arise_slam and FASTLIO2
- Set up the environment for both ROS and DimOS

The resulting image will be named `dimos_autonomy_stack:{distro}` (e.g., `humble`, `jazzy`).
Select SLAM method at runtime via `--localization arise_slam` or `--localization fastlio`.

Note that the build will take a while and produce an image of approximately 24 GB.

**Run the simulator to test it's working:**

Use the same ROS distribution flag as your build:

```bash
./start.sh --simulation --image humble  # If built with --humble
# or
./start.sh --simulation --image jazzy   # If built with --jazzy
```

<details>
<summary><h2>Manual build</h2></summary>

Go to the docker dir and clone the ROS navigation stack (choose the branch matching your ROS distribution).

```bash
cd docker/navigation
git clone -b humble git@github.com:dimensionalOS/ros-navigation-autonomy-stack.git
# or
git clone -b jazzy git@github.com:dimensionalOS/ros-navigation-autonomy-stack.git
```

Download a [Unity environment model for the Mecanum wheel platform](https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs?usp=sharing) and unzip the files to `unity_models`.

Alternativelly, extract `office_building_1` from LFS:

```bash
tar -xf ../../data/.lfs/office_building_1.tar.gz
mv office_building_1 unity_models
```

Then, go back to the root (from docker/navigation) and build the docker image:

```bash
cd ../..  # Back to dimos root
ROS_DISTRO=humble docker compose -f docker/navigation/docker-compose.yml build
# or
ROS_DISTRO=jazzy docker compose -f docker/navigation/docker-compose.yml build
```

</details>

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
# Robot Configuration
ROBOT_CONFIG_PATH=unitree/unitree_go2  # Robot type (mechanum_drive, unitree/unitree_go2, unitree/unitree_g1)

# Lidar Configuration
LIDAR_INTERFACE=eth0              # Your ethernet interface (find with: ip link show)
LIDAR_COMPUTER_IP=192.168.1.5    # Computer IP on the lidar subnet
LIDAR_GATEWAY=192.168.1.1        # Gateway IP address for the lidar subnet
LIDAR_IP=192.168.1.1xx           # xx = last two digits from lidar QR code serial number
ROBOT_IP=                        # IP addres of robot on local network (if using WebRTC connection)

# Special Configuration for Unitree G1 EDU
# Special Configuration for Unitree G1 EDU
LIDAR_COMPUTER_IP=192.168.123.5
LIDAR_GATEWAY=192.168.123.1
LIDAR_IP=192.168.123.120
ROBOT_IP=192.168.12.1  # For WebRTC local AP mode (optional, need additional wifi dongle)
```

### Start the Navigation Stack

#### Start with Route Planner automatically

```bash
# arise_slam (default)
./start.sh --hardware --route-planner
./start.sh --hardware --route-planner --rviz

# FASTLIO2
./start.sh --hardware --localization fastlio --route-planner
./start.sh --hardware --localization fastlio --route-planner --rviz

# Jazzy image
./start.sh --hardware --image jazzy --route-planner

# Development mode (mount src for config editing)
./start.sh --hardware --dev
```

[Foxglove Studio](https://foxglove.dev/download) is the default visualization tool. It's ideal for remote operation - SSH with port forwarding to the robot's mini PC and run commands there:

```bash
ssh -L 8765:localhost:8765 user@robot-ip
```

Then on your local machine:
1. Open Foxglove and connect to `ws://localhost:8765`
2. Load the layout from `dimos/assets/foxglove_dashboards/Overwatch.json` (Layout menu → Import)
3. Click in the 3D panel to drop a target pose (similar to RViz). The "Autonomy ON" indicator should be green, and "Goal Reached" will show when the robot arrives.

<details>
<summary><h4>Start manually</h4></summary>

Start the container and leave it open. Use the same ROS distribution flag as your build:

```bash
./start.sh --hardware --image humble  # If built with --humble
# or
./start.sh --hardware --image jazzy   # If built with --jazzy
```

It doesn't do anything by default. You have to run commands on it by `exec`-ing:

To enter the container from another terminal:

```bash
docker exec -it dimos_hardware_container bash
```

##### In the container

In the container to run the full navigation stack you must run both the dimensional python runfile with connection module and the navigation stack.

###### Dimensional Python + Connection Module

For the Unitree G1
```bash
dimos run unitree-g1
ROBOT_IP=XX.X.X.XXX dimos run unitree-g1 # If ROBOT_IP env variable is not set in .env
```

###### Navigation Stack

```bash
cd /ros2_ws/src/ros-navigation-autonomy-stack
./system_real_robot_with_route_planner.sh
```

Now you can place goal points/poses in RVIZ by clicking the "Goalpoint" button. The robot will navigate to the point, running both local and global planners for dynamic obstacle avoidance.

</details>
