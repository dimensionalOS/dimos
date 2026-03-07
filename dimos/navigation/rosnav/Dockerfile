# syntax=docker/dockerfile:1
# =============================================================================
# DimOS Navigation Docker Image
# =============================================================================
#
# Multi-stage build for ROS 2 navigation with SLAM support.
# Includes both arise_slam and FASTLIO2 - select at runtime via LOCALIZATION_METHOD.
#
# The ros-navigation-autonomy-stack repo is cloned at build time via SSH.
# Build with:  docker build --ssh default ...
#
# =============================================================================

# Build argument for ROS distribution (default: humble)
ARG ROS_DISTRO=humble
ARG TARGETARCH
# Pinned git ref for ros-navigation-autonomy-stack (branch, tag, or commit SHA)
ARG NAV_STACK_REF=fastlio2

# -----------------------------------------------------------------------------
# Platform-specific base images
# - amd64: Use osrf/ros desktop-full (includes Gazebo, full GUI)
# - arm64: Use ros-base (desktop-full not available for ARM)
# -----------------------------------------------------------------------------
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base-amd64
FROM ros:${ROS_DISTRO}-ros-base AS base-arm64

# -----------------------------------------------------------------------------
# STAGE 1: Build Stage - compile all C++ dependencies
# -----------------------------------------------------------------------------
FROM base-${TARGETARCH} AS builder

ARG ROS_DISTRO
ARG NAV_STACK_REF
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
    # SSH client for private repo clone
    openssh-client \
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

# On arm64, ros-base doesn't include rviz2 (unlike desktop-full on amd64)
# Install it separately for building rviz plugins
# Note: ARG must be re-declared after FROM; placed here to maximize layer caching above
ARG TARGETARCH
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rviz2 \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# On arm64, build open3d from source (no Linux aarch64 wheels on PyPI)
# Cached as a separate layer; the wheel is copied to the runtime stage
# mkdir runs unconditionally so COPY --from=builder works on all architectures
RUN mkdir -p /opt/open3d-wheel && \
    PYTHON_MINOR=$(python3 -c "import sys; print(sys.version_info.minor)") && \
    if [ "${TARGETARCH}" = "arm64" ] && [ "$PYTHON_MINOR" -ge 12 ]; then \
        echo "Building open3d from source for arm64 + Python 3.${PYTHON_MINOR} (no PyPI wheel)..." && \
        apt-get update && apt-get install -y --no-install-recommends \
            python3-dev \
            python3-pip \
            python3-setuptools \
            python3-wheel \
            libblas-dev \
            liblapack-dev \
            libgl1-mesa-dev \
            libglib2.0-dev \
            libxinerama-dev \
            libxcursor-dev \
            libxrandr-dev \
            libxi-dev \
            gfortran \
        && rm -rf /var/lib/apt/lists/* && \
        cd /tmp && \
        git clone --depth 1 --branch v0.19.0 https://github.com/isl-org/Open3D.git && \
        cd Open3D && \
        util/install_deps_ubuntu.sh assume-yes && \
        mkdir build && cd build && \
        cmake .. \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_CUDA_MODULE=OFF \
            -DBUILD_GUI=OFF \
            -DBUILD_TENSORFLOW_OPS=OFF \
            -DBUILD_PYTORCH_OPS=OFF \
            -DBUILD_UNIT_TESTS=OFF \
            -DBUILD_BENCHMARKS=OFF \
            -DBUILD_EXAMPLES=OFF \
            -DBUILD_WEBRTC=OFF && \
        make -j$(($(nproc) > 4 ? 4 : $(nproc))) && \
        make pip-package -j$(($(nproc) > 4 ? 4 : $(nproc))) && \
        mkdir -p /opt/open3d-wheel && \
        cp lib/python_package/pip_package/open3d*.whl /opt/open3d-wheel/ && \
        cd / && rm -rf /tmp/Open3D; \
    fi

# On arm64, build or-tools from source (pre-built binaries are x86_64 only)
# This is cached as a separate layer since it takes significant time to build
ENV OR_TOOLS_VERSION=9.8
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        echo "Building or-tools v${OR_TOOLS_VERSION} from source for arm64..." && \
        apt-get update && apt-get install -y --no-install-recommends \
            lsb-release \
            wget \
        && rm -rf /var/lib/apt/lists/* && \
        cd /tmp && \
        wget -q https://github.com/google/or-tools/archive/refs/tags/v${OR_TOOLS_VERSION}.tar.gz && \
        tar xzf v${OR_TOOLS_VERSION}.tar.gz && \
        cd or-tools-${OR_TOOLS_VERSION} && \
        cmake -S . -B build \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_DEPS=ON \
            -DBUILD_SAMPLES=OFF \
            -DBUILD_EXAMPLES=OFF \
            -DBUILD_FLATZINC=OFF \
            -DUSE_SCIP=OFF \
            -DUSE_COINOR=OFF && \
        cmake --build build --config Release -j$(($(nproc) > 4 ? 4 : $(nproc))) && \
        cmake --install build --prefix /opt/or-tools && \
        rm -rf /tmp/or-tools-${OR_TOOLS_VERSION} /tmp/v${OR_TOOLS_VERSION}.tar.gz; \
    fi

# Create workspace
RUN mkdir -p ${WORKSPACE}/src

# Clone autonomy stack source via SSH (requires --ssh default at build time)
RUN --mount=type=ssh \
    mkdir -p ~/.ssh && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts && \
    git clone -b ${NAV_STACK_REF} --depth 1 \
        git@github.com:dimensionalOS/ros-navigation-autonomy-stack.git \
        ${WORKSPACE}/src/ros-navigation-autonomy-stack

# On arm64, replace pre-built x86_64 or-tools with arm64 built version
RUN if [ "${TARGETARCH}" = "arm64" ] && [ -d "/opt/or-tools" ]; then \
        echo "Replacing x86_64 or-tools with arm64 build..." && \
        OR_TOOLS_DIR=${WORKSPACE}/src/ros-navigation-autonomy-stack/src/exploration_planner/tare_planner/or-tools && \
        rm -rf ${OR_TOOLS_DIR}/lib/*.so* ${OR_TOOLS_DIR}/lib/*.a && \
        cp -r /opt/or-tools/lib/* ${OR_TOOLS_DIR}/lib/ && \
        rm -rf ${OR_TOOLS_DIR}/include && \
        cp -r /opt/or-tools/include ${OR_TOOLS_DIR}/ && \
        ldconfig; \
    fi

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

# Build ROS workspace with both SLAM systems (no --symlink-install for multi-stage build compatibility)
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WORKSPACE} && \
    echo 'Building with both arise_slam and FASTLIO2' && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# -----------------------------------------------------------------------------
# STAGE 2: Runtime Stage - minimal image for running
# -----------------------------------------------------------------------------
ARG ROS_DISTRO
ARG TARGETARCH
FROM base-${TARGETARCH} AS runtime

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=/ros2_ws
ENV DIMOS_PATH=/workspace/dimos
# LOCALIZATION_METHOD: arise_slam (default) or fastlio
ENV LOCALIZATION_METHOD=arise_slam

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
    ros-${ROS_DISTRO}-joy \
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
# Copy SLAM config files based on SLAM_TYPE
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/config ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/config

# Copy config files for both SLAM systems
RUN --mount=from=builder,source=${WORKSPACE}/src/ros-navigation-autonomy-stack/src,target=/tmp/src \
    echo "Copying arise_slam configs" && \
    mkdir -p ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/arise_slam_mid360 && \
    cp -r /tmp/src/slam/arise_slam_mid360/config ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/arise_slam_mid360/ 2>/dev/null || true && \
    echo "Copying FASTLIO2 configs" && \
    mkdir -p ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/FASTLIO2_ROS2 && \
    for pkg in fastlio2 localizer pgo hba; do \
        if [ -d "/tmp/src/slam/FASTLIO2_ROS2/$pkg/config" ]; then \
            mkdir -p ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/FASTLIO2_ROS2/$pkg && \
            cp -r /tmp/src/slam/FASTLIO2_ROS2/$pkg/config ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/FASTLIO2_ROS2/$pkg/; \
        fi; \
        if [ -d "/tmp/src/slam/FASTLIO2_ROS2/$pkg/rviz" ]; then \
            cp -r /tmp/src/slam/FASTLIO2_ROS2/$pkg/rviz ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/slam/FASTLIO2_ROS2/$pkg/; \
        fi; \
    done

# Copy simulation shell scripts (real robot mode uses volume mount)
COPY --from=builder ${WORKSPACE}/src/ros-navigation-autonomy-stack/system_simulation*.sh ${WORKSPACE}/src/ros-navigation-autonomy-stack/

# Create directories
RUN mkdir -p ${DIMOS_PATH} \
    ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/unity \
    ${WORKSPACE}/bagfiles \
    ${WORKSPACE}/logs \
    ${WORKSPACE}/config

# Copy FastDDS configuration (single source: dimos/navigation/rosnav/fastdds.xml)
# At runtime the volume mount may overlay this, but the baked-in copy ensures
# the image works standalone.
COPY dimos/navigation/rosnav/fastdds.xml ${WORKSPACE}/config/fastdds.xml

# Install portaudio for unitree-webrtc-connect (pyaudio dependency)
RUN apt-get update && apt-get install -y --no-install-recommends \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# Create Python venv and install dependencies
RUN python3 -m venv /opt/dimos-venv && \
    /opt/dimos-venv/bin/pip install --no-cache-dir \
    pyyaml

# On arm64, install open3d wheel built from source in the builder stage
COPY --from=builder /opt/open3d-wheel /opt/open3d-wheel
ARG TARGETARCH
RUN if [ "${TARGETARCH}" = "arm64" ] && ls /opt/open3d-wheel/open3d*.whl 1>/dev/null 2>&1; then \
        echo "Installing open3d from pre-built arm64 wheel..." && \
        /opt/dimos-venv/bin/pip install --no-cache-dir /opt/open3d-wheel/open3d*.whl && \
        rm -rf /opt/open3d-wheel; \
    fi

# Copy dimos source and install as editable package
# The volume mount at runtime will overlay /workspace/dimos, but the editable
# install creates a link that will use the volume-mounted files
COPY pyproject.toml /workspace/dimos/
COPY dimos /workspace/dimos/dimos
RUN /opt/dimos-venv/bin/pip install --no-cache-dir -e "/workspace/dimos[unitree]"

# Set up shell environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/dimos-venv/bin/activate" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml" >> ~/.bashrc

# Copy helper scripts (paths relative to repo root build context)
COPY docker/navigation/foxglove_utility/twist_relay.py /usr/local/bin/twist_relay.py
COPY docker/navigation/foxglove_utility/goal_autonomy_relay.py /usr/local/bin/goal_autonomy_relay.py
COPY dimos/navigation/rosnav/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/twist_relay.py /usr/local/bin/goal_autonomy_relay.py /usr/local/bin/entrypoint.sh

# Set up udev rules for motor controller
RUN mkdir -p /etc/udev/rules.d && \
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout"' \
    > /etc/udev/rules.d/99-motor-controller.rules

# Working directory
WORKDIR ${DIMOS_PATH}

# Default entrypoint (overridden at docker run time by DockerModule)
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command
CMD ["bash"]

# DIMOS-MODULE-CONVERSION-427593ae-c6e8-4cf1-9b2d-ee81a420a5dc
# (sentinel prevents docker_build.py from appending redundant DimOS footer;
#  dimos is already installed above via pip install -e)
