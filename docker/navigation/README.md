# ROS Docker Integration for DimOS

This directory contains Docker configuration files to run DimOS and the ROS autonomy stack in the same container, enabling communication between the two systems.

## Prerequisites

1. **Install Docker with `docker compose` support**. Follow the [official Docker installation guide](https://docs.docker.com/engine/install/).
2. **Install NVIDIA GPU drivers**. See [NVIDIA driver installation](https://www.nvidia.com/download/index.aspx).
3. **Install NVIDIA Container Toolkit**. Follow the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

### Hardware Requirements
- **RAM:** Minimum 8GB (16GB+ recommended). The build process is memory intensive.
- **Disk:** Minimum 40GB free space (Docker image is ~30GB).

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
git clone -b jazzy https://github.com/dimensionalOS/ros-navigation-autonomy-stack.git
cd ros-navigation-autonomy-stack
# IMPORTANT: You must initialize submodules to get all planners
git submodule update --init --recursive
cd ..
```

Download a [Unity environment model for the Mecanum wheel platform](https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs?usp=sharing) and unzip the files to `unity_models`.

Alternatively, extract `office_building_1` from LFS (ensure you have run `git lfs install && git lfs pull` in the root first):

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

## Troubleshooting

### Build Crashes (RAM issues)
If your computer freezes or the build crashes with "signal: killed", you are likely running out of RAM.
- **Fix:** The `Dockerfile` by default tries to use all cores. Edit `Dockerfile` and change `make -j$(nproc)` to `make -j1` in the heavy build sections (GTSAM, Ceres).

### RVIZ "No Image"
If RVIZ launches but camera panels show "No Image":
1.  Click the specific Image panel settings in RVIZ.
2.  Change the **Topic** dropdown. Sometimes it defaults to an inactive topic.

### Disk Full
If the build fails with "no space left on device":
- **Fix:** Run `docker system prune` to clear dangling build cache. The image requires significant space.

### Robot Not Moving
If `PCT_planner` or other packages are missing logic:
- Check if `ros-navigation-autonomy-stack` has empty directories.
- Run `git submodule update --init --recursive` in that directory.
