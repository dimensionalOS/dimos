# Base image with ROS Jazzy desktop full
FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/ros2_ws
ENV DIMOS_PATH=/workspace/dimos

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # ROS packages
    ros-jazzy-pcl-ros \
    # Development tools
    git \
    git-lfs \
    cmake \
    build-essential \
    python3-colcon-common-extensions \
    # PCL and system libraries
    libpcl-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    # X11 and GUI support for RVIZ
    x11-apps \
    xorg \
    openbox \
    # Networking tools
    iputils-ping \
    net-tools \
    iproute2 \
    ethtool \
    # USB and serial tools (for hardware support)
    usbutils \
    udev \
    # Time synchronization (for multi-computer setup)
    chrony \
    # Editor (optional but useful)
    nano \
    vim \
    # Python tools
    python3-pip \
    python3-setuptools \
    python3-venv \
    # Additional dependencies for dimos
    ffmpeg \
    portaudio19-dev \
    libsndfile1 \
    # For OpenCV
    libgl1 \
    libglib2.0-0 \
    # For Open3D
    libgomp1 \
    # For TurboJPEG
    libturbojpeg0-dev \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p ${WORKSPACE}/src

# Copy the autonomy stack repository (should be cloned by build.sh)
COPY docker/navigation/ros-navigation-autonomy-stack ${WORKSPACE}/src/ros-navigation-autonomy-stack

# Set working directory
WORKDIR ${WORKSPACE}

# Set up ROS environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Build all hardware dependencies
RUN \
    # Build Livox-SDK2 for Mid-360 lidar
    cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/Livox-SDK2 && \
    mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig && \
    # Install Sophus
    cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/Sophus && \
    mkdir -p build && cd build && \
    cmake .. -DBUILD_TESTS=OFF && make -j$(nproc) && make install && \
    # Install Ceres Solver
    cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/ceres-solver && \
    mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    # Install GTSAM
    cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/gtsam && \
    mkdir -p build && cd build && \
    cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j$(nproc) && make install && ldconfig

# Build the autonomy stack
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WORKSPACE} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Source the workspace setup
RUN echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc

# Create directory for Unity environment models
RUN mkdir -p ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/unity

# Copy the dimos repository
RUN mkdir -p ${DIMOS_PATH}
COPY . ${DIMOS_PATH}/

# Create a virtual environment in /opt (not in /workspace/dimos)
# This ensures the venv won't be overwritten when we mount the host dimos directory
# The container will always use its own dependencies, independent of the host
RUN python3 -m venv /opt/dimos-venv

# Activate Python virtual environment in interactive shells
RUN echo "source /opt/dimos-venv/bin/activate" >> ~/.bashrc

# Install Python dependencies for dimos
WORKDIR ${DIMOS_PATH}
RUN /bin/bash -c "source /opt/dimos-venv/bin/activate && \
    pip install --upgrade pip setuptools wheel && \
    pip install -e .[cpu,dev] 'mmengine>=0.10.3' 'mmcv>=2.1.0'"

# Copy helper scripts
COPY docker/navigation/run_both.sh /usr/local/bin/run_both.sh
COPY docker/navigation/ros_launch_wrapper.py /usr/local/bin/ros_launch_wrapper.py
RUN chmod +x /usr/local/bin/run_both.sh /usr/local/bin/ros_launch_wrapper.py

# Set up udev rules for USB devices (motor controller)
RUN echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout"' > /etc/udev/rules.d/99-motor-controller.rules && \
    usermod -a -G dialout root || true

# Set up entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
git config --global --add safe.directory /workspace/dimos\n\
\n\
# Source ROS setup\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${WORKSPACE}/install/setup.bash\n\
\n\
# Activate Python virtual environment for dimos\n\
source /opt/dimos-venv/bin/activate\n\
\n\
# Export ROBOT_CONFIG_PATH for autonomy stack\n\
export ROBOT_CONFIG_PATH="${ROBOT_CONFIG_PATH:-mechanum_drive}"\n\
\n\
# Hardware-specific configurations\n\
if [ "${HARDWARE_MODE}" = "true" ]; then\n\
    # Set network buffer sizes for WiFi data transmission (if needed)\n\
    if [ "${ENABLE_WIFI_BUFFER}" = "true" ]; then\n\
        sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864 2>/dev/null || true\n\
        sysctl -w net.core.wmem_max=67108864 net.core.wmem_default=67108864 2>/dev/null || true\n\
    fi\n\
    \n\
    # Configure network interface for Mid-360 lidar if specified\n\
    if [ -n "${LIDAR_INTERFACE}" ] && [ -n "${LIDAR_COMPUTER_IP}" ]; then\n\
        ip addr add ${LIDAR_COMPUTER_IP}/24 dev ${LIDAR_INTERFACE} 2>/dev/null || true\n\
        ip link set ${LIDAR_INTERFACE} up 2>/dev/null || true\n\
        if [ -n "${LIDAR_GATEWAY}" ]; then\n\
            ip route add default via ${LIDAR_GATEWAY} dev ${LIDAR_INTERFACE} 2>/dev/null || true\n\
        fi\n\
    fi\n\
    \n\
    # Generate MID360_config.json if LIDAR_COMPUTER_IP and LIDAR_IP are set\n\
    if [ -n "${LIDAR_COMPUTER_IP}" ] && [ -n "${LIDAR_IP}" ]; then\n\
        cat > ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/config/MID360_config.json <<EOF\n\
{\n\
  "lidar_summary_info": {\n\
    "lidar_type": 8\n\
  },\n\
  "MID360": {\n\
    "lidar_net_info": {\n\
      "cmd_data_port": 56100,\n\
      "push_msg_port": 56200,\n\
      "point_data_port": 56300,\n\
      "imu_data_port": 56400,\n\
      "log_data_port": 56500\n\
    },\n\
    "host_net_info": {\n\
      "cmd_data_ip": "${LIDAR_COMPUTER_IP}",\n\
      "cmd_data_port": 56101,\n\
      "push_msg_ip": "${LIDAR_COMPUTER_IP}",\n\
      "push_msg_port": 56201,\n\
      "point_data_ip": "${LIDAR_COMPUTER_IP}",\n\
      "point_data_port": 56301,\n\
      "imu_data_ip": "${LIDAR_COMPUTER_IP}",\n\
      "imu_data_port": 56401,\n\
      "log_data_ip": "${LIDAR_COMPUTER_IP}",\n\
      "log_data_port": 56501\n\
    }\n\
  },\n\
  "lidar_configs": [\n\
    {\n\
      "ip": "${LIDAR_IP}",\n\
      "pcl_data_type": 1,\n\
      "pattern_mode": 0,\n\
      "extrinsic_parameter": {\n\
        "roll": 0.0,\n\
        "pitch": 0.0,\n\
        "yaw": 0.0,\n\
        "x": 0,\n\
        "y": 0,\n\
        "z": 0\n\
      }\n\
    }\n\
  ]\n\
}\n\
EOF\n\
        echo "Generated MID360_config.json with LIDAR_COMPUTER_IP=${LIDAR_COMPUTER_IP} and LIDAR_IP=${LIDAR_IP}"\n\
    fi\n\
    \n\
fi\n\
\n\
# Execute the command\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command
CMD ["bash"]
