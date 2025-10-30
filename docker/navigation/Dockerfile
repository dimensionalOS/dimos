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

# Build the autonomy stack workspace (simulation mode - skipping SLAM and Mid-360 driver)
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WORKSPACE} && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-skip arise_slam_mid360 arise_slam_mid360_msgs livox_ros_driver2"

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

# Install Python dependencies for dimos
WORKDIR ${DIMOS_PATH}
RUN /bin/bash -c "source /opt/dimos-venv/bin/activate && \
    pip install --upgrade pip setuptools wheel && \
    pip install -e .[cpu,dev] 'mmengine>=0.10.3' 'mmcv>=2.1.0'"

# Copy helper scripts
COPY docker/navigation/run_both.sh /usr/local/bin/run_both.sh
COPY docker/navigation/ros_launch_wrapper.py /usr/local/bin/ros_launch_wrapper.py
RUN chmod +x /usr/local/bin/run_both.sh /usr/local/bin/ros_launch_wrapper.py

# Set up entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS setup\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${WORKSPACE}/install/setup.bash\n\
\n\
# Activate Python virtual environment for dimos\n\
source /opt/dimos-venv/bin/activate\n\
\n\
# Export ROBOT_CONFIG_PATH for autonomy stack\n\
export ROBOT_CONFIG_PATH="mechanum_drive"\n\
\n\
# Execute the command\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command
CMD ["bash"]