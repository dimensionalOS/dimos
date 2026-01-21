# =============================================================================
# OPTIMIZED DOCKERFILE - Multi-stage build for reduced image size
# =============================================================================
#
# Key optimizations:
# 1. Multi-stage build with ros:desktop-full base image
# 2. Multi-stage build to discard build artifacts
# 3. No Python ML dependencies (~14 GB saved)
# 4. COPY dimos source for editable pip install (volume-mounted at runtime overlays it)
# 5. Clean up build directories after compile (~800 MB saved)
# 6. Minimal apt packages with --no-install-recommends
# 7. DDS configuration for optimized ROS 2 communication
#
# Supported ROS distributions: jazzy, humble
# Build with: docker build --build-arg ROS_DISTRO=humble ...
#
# =============================================================================

# Build argument for ROS distribution (default: humble)
ARG ROS_DISTRO=humble

# -----------------------------------------------------------------------------
# STAGE 1: Build Stage - compile all C++ dependencies
# -----------------------------------------------------------------------------
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS builder

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=/ros2_ws

# Install build dependencies only
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    git \
    cmake \
    build-essential \
    python3-colcon-common-extensions \
    # Libraries needed for building
    libpcl-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    # ROS packages needed for build
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p ${WORKSPACE}/src

# Copy autonomy stack source
COPY docker/navigation/ros-navigation-autonomy-stack ${WORKSPACE}/src/ros-navigation-autonomy-stack

# Compatibility fix: In Humble, cv_bridge uses .h extension, but Jazzy uses .hpp
# Create a symlink so code written for Jazzy works on Humble
RUN if [ "${ROS_DISTRO}" = "humble" ]; then \
        CV_BRIDGE_DIR=$(find /opt/ros/humble/include -name "cv_bridge.h" -printf "%h\n" 2>/dev/null | head -1) && \
        if [ -n "$CV_BRIDGE_DIR" ]; then \
            ln -sf "$CV_BRIDGE_DIR/cv_bridge.h" "$CV_BRIDGE_DIR/cv_bridge.hpp"; \
            echo "Created cv_bridge.hpp symlink in $CV_BRIDGE_DIR"; \
        else \
            echo "Warning: cv_bridge.h not found, skipping symlink creation"; \
        fi; \
    fi

# Build Livox-SDK2
RUN cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/Livox-SDK2 && \
    mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig && \
    rm -rf ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/Livox-SDK2/build

# Build Sophus
RUN cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/Sophus && \
    mkdir -p build && cd build && \
    cmake .. -DBUILD_TESTS=OFF && make -j$(nproc) && make install && \
    rm -rf ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/Sophus/build

# Build Ceres Solver
RUN cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/ceres-solver && \
    mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/ceres-solver/build

# Build GTSAM
RUN cd ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/gtsam && \
    mkdir -p build && cd build && \
    cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/dependency/gtsam/build

# Build ROS workspace (no --symlink-install for multi-stage build compatibility)
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WORKSPACE} && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# -----------------------------------------------------------------------------
# STAGE 2: Runtime Stage - minimal image for running
# -----------------------------------------------------------------------------
ARG ROS_DISTRO
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS runtime

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=/ros2_ws
ENV DIMOS_PATH=/workspace/dimos

# DDS Configuration - Use FastDDS (default ROS 2 middleware)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml

# Install runtime dependencies only (no build tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS packages
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt* \
    # DDS middleware (FastDDS is default, just ensure it's installed)
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    # Runtime libraries
    libpcl-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    # X11 for GUI (minimal)
    libx11-6 \
    libxext6 \
    libxrender1 \
    libgl1 \
    libglib2.0-0 \
    # Networking tools
    iputils-ping \
    net-tools \
    iproute2 \
    # Serial/USB for hardware
    usbutils \
    # Python (minimal)
    python3-pip \
    python3-venv \
    # Joystick support
    joystick \
    # Time sync for multi-computer setups
    chrony \
    && rm -rf /var/lib/apt/lists/*

# Copy installed libraries from builder
COPY --from=builder /usr/local/lib /usr/local/lib
COPY --from=builder /usr/local/include /usr/local/include

RUN ldconfig

# Copy built ROS workspace from builder
COPY --from=builder ${WORKSPACE}/install ${WORKSPACE}/install

# Copy only config/rviz files from src (not the large dependency folders)
# These are needed if running without volume mount
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/rviz ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/rviz
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/route_planner/far_planner/rviz ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/route_planner/far_planner/rviz
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/exploration_planner/tare_planner/rviz ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/exploration_planner/tare_planner/rviz
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/arise_slam_mid360/config ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/arise_slam_mid360/config
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/config ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/config

# Copy simulation shell scripts (real robot mode uses volume mount)
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/system_simulation*.sh ${WORKSPACE}/src/ros-navigation-autonomy-stack/

# Create directories
RUN mkdir -p ${DIMOS_PATH} \
    ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/unity \
    ${WORKSPACE}/bagfiles \
    ${WORKSPACE}/logs \
    ${WORKSPACE}/config

# Create FastDDS configuration file
RUN cat > ${WORKSPACE}/config/fastdds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <!-- Default participant profile applied to all ROS 2 nodes -->
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <name>ros2_navigation_participant</name>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <leaseDuration>
                        <sec>10</sec>
                        <nanosec>0</nanosec>
                    </leaseDuration>
                    <leaseAnnouncement>
                        <sec>3</sec>
                        <nanosec>0</nanosec>
                    </leaseAnnouncement>
                </discovery_config>
            </builtin>
            <!-- Socket buffer sizes for high-bandwidth data (point clouds) -->
            <sendSocketBufferSize>10485760</sendSocketBufferSize>
            <listenSocketBufferSize>10485760</listenSocketBufferSize>
            <useBuiltinTransports>true</useBuiltinTransports>
        </rtps>
    </participant>

    <!-- Transport descriptors -->
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>10485760</sendBufferSize>
            <receiveBufferSize>10485760</receiveBufferSize>
            <maxMessageSize>65500</maxMessageSize>
        </transport_descriptor>
        <!-- Shared memory for same-host communication -->
        <transport_descriptor>
            <transport_id>shm_transport</transport_id>
            <type>SHM</type>
            <segment_size>10485760</segment_size>
            <max_message_size>1048576</max_message_size>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
EOF

# Install portaudio for unitree-webrtc-connect (pyaudio dependency)
RUN apt-get update && apt-get install -y --no-install-recommends \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# Create Python venv and install dependencies
RUN python3 -m venv /opt/dimos-venv && \
    /opt/dimos-venv/bin/pip install --no-cache-dir \
    pyyaml

# Copy dimos source and install as editable package
# The volume mount at runtime will overlay /workspace/dimos, but the editable
# install creates a link that will use the volume-mounted files
COPY pyproject.toml setup.py /workspace/dimos/
COPY dimos /workspace/dimos/dimos
RUN /opt/dimos-venv/bin/pip install --no-cache-dir -e "/workspace/dimos[unitree]"

# Set up shell environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/dimos-venv/bin/activate" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml" >> ~/.bashrc

# Copy helper scripts
COPY docker/navigation/run_both.sh /usr/local/bin/run_both.sh
COPY docker/navigation/ros_launch_wrapper.py /usr/local/bin/ros_launch_wrapper.py
COPY docker/navigation/foxglove_utility/twist_relay.py /usr/local/bin/twist_relay.py
COPY docker/navigation/foxglove_utility/goal_autonomy_relay.py /usr/local/bin/goal_autonomy_relay.py
RUN chmod +x /usr/local/bin/run_both.sh /usr/local/bin/ros_launch_wrapper.py /usr/local/bin/twist_relay.py /usr/local/bin/goal_autonomy_relay.py

# Set up udev rules for motor controller
RUN mkdir -p /etc/udev/rules.d && \
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout"' \
    > /etc/udev/rules.d/99-motor-controller.rules

# Set up entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Mark git directories as safe\n\
git config --global --add safe.directory /workspace/dimos 2>/dev/null || true\n\
git config --global --add safe.directory /ros2_ws/src/ros-navigation-autonomy-stack 2>/dev/null || true\n\
\n\
# Source ROS setup\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${WORKSPACE}/install/setup.bash\n\
\n\
# Activate Python virtual environment\n\
source /opt/dimos-venv/bin/activate\n\
\n\
# DDS Configuration (FastDDS)\n\
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n\
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml\n\
\n\
# Use custom DDS config if provided via mount\n\
if [ -f "/ros2_ws/config/custom_fastdds.xml" ]; then\n\
    export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/custom_fastdds.xml\n\
    echo "Using custom FastDDS configuration"\n\
fi\n\
\n\
# Export ROBOT_CONFIG_PATH for autonomy stack\n\
export ROBOT_CONFIG_PATH="${ROBOT_CONFIG_PATH:-mechanum_drive}"\n\
\n\
# Hardware-specific configurations\n\
if [ "${HARDWARE_MODE}" = "true" ]; then\n\
    # Set network buffer sizes for WiFi data transmission\n\
    if [ "${ENABLE_WIFI_BUFFER}" = "true" ]; then\n\
        sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864 2>/dev/null || true\n\
        sysctl -w net.core.wmem_max=67108864 net.core.wmem_default=67108864 2>/dev/null || true\n\
    fi\n\
    \n\
    # Configure network interface for Mid-360 lidar if specified\n\
    if [ -n "${LIDAR_INTERFACE}" ] && [ -n "${LIDAR_COMPUTER_IP}" ]; then\n\
        ip addr add ${LIDAR_COMPUTER_IP}/24 dev ${LIDAR_INTERFACE} 2>/dev/null || true\n\
        ip link set ${LIDAR_INTERFACE} up 2>/dev/null || true\n\
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
        # Also copy to installed location where the driver actually reads from\n\
        cp ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/config/MID360_config.json \\\n\
           ${WORKSPACE}/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json 2>/dev/null || true\n\
        echo "Generated MID360_config.json with LIDAR_COMPUTER_IP=${LIDAR_COMPUTER_IP} and LIDAR_IP=${LIDAR_IP}"\n\
    fi\n\
    \n\
    # Display Robot IP configuration if set\n\
    if [ -n "${ROBOT_IP}" ]; then\n\
        echo "Robot IP configured on local network: ${ROBOT_IP}"\n\
    fi\n\
fi\n\
\n\
# Execute the command\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Working directory
WORKDIR ${DIMOS_PATH}

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command
CMD ["bash"]
