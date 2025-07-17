FROM nvcr.io/nvidia/isaac-sim:4.2.0

# Set up locales
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    git \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Set timezone non-interactively
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Setup ROS 2
RUN add-apt-repository universe -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    && apt-get upgrade -y \
    && apt-get install -y \
        ros-humble-desktop \
        ros-humble-ros-base \
        ros-dev-tools \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-pip \
        python3.10-venv \
        ament-cmake \
        ros-humble-ament-cmake \
        build-essential \
        cmake \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        nano \
        wget \
        curl \
        vim \
        git \
        x11-apps \
        tmux \
        ros-humble-foxglove-bridge \
        ros-humble-moveit \
        ros-humble-moveit-visual-tools \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-servo \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rosbridge-suite \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-teleop-twist-keyboard \
        ros-humble-teleop-twist-joy \
        ros-humble-joy \
        ros-humble-controller-manager \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-trajectory-controller \
        ros-humble-joint-state-broadcaster \
        ros-humble-vision-msgs \
        ros-humble-ackermann-msgs \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-nav2-msgs \
        ros-humble-nav2-common \
        ros-humble-nav2-behavior-tree \
        ros-humble-nav2-costmap-2d \
        ros-humble-nav2-core \
        ros-humble-nav2-bt-navigator \
        ros-humble-pointcloud-to-laserscan \
        iputils-ping \
        net-tools \
        htop \
        python3-pip \
        ros-humble-tf* \
        ros-humble-gazebo-ros-pkgs \
        dos2unix \
        python3-genmsg \
        gpg \
        pass \
        ros-humble-depthai-ros \
        zstd \
        && rm -rf /var/lib/apt/lists/*

RUN apt-get upgrade -y


# Initialize rosdep
RUN rosdep init && rosdep update

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Python packages directly
RUN pip install --no-cache-dir \
    rospkg \
    numpy==1.24.4 \
    jsonpickle \
    scipy \
    easydict \
    matplotlib==3.9.1 \
    opencv-python \
    pyyaml \
    pyquaternion \
    pybullet \
    requests \
    pillow \
    open3d \
    av==10.0.0 \
    transforms3d \
    torch \
    torchvision \
    torchaudio \
    transformers


ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create ros home directory
RUN mkdir -p /home/$USERNAME

RUN cd /home/$USERNAME && git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
RUN rosdep update
RUN /bin/bash -c "cd /home/$USERNAME/IsaacSim-ros_workspaces/humble_ws && rosdep install -i --from-path src --rosdistro humble -y"

RUN mkdir -p /home/$USERNAME/dev_ws/src
RUN cd /home/$USERNAME/dev_ws/src && git clone https://github.com/yashas-salankimatt/thesis_ros_ws.git

# Install ZED SDK
RUN wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Ubuntu22_cuda12.1_v4.2.1.zstd.run && chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.2.1.zstd.run
RUN /bin/bash -c "./ZED_SDK_Ubuntu22_cuda12.1_v4.2.1.zstd.run -- silent skip_cuda"

ENV ZED_SDK_ROOT_DIR=/usr/local/zed
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${ZED_SDK_ROOT_DIR}


RUN mkdir -p /home/$USERNAME/deps
RUN cd /home/$USERNAME/deps && git clone https://github.com/facebookresearch/segment-anything-2.git
RUN cd /home/$USERNAME/deps/segment-anything-2 && pip install -e .
RUN cd /home/$USERNAME/dev_ws
RUN chown -R $USER_UID:$USER_GID /home/$USERNAME/

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/$USERNAME/IsaacSim-ros_workspaces/humble_ws && colcon build"
RUN rm -rf /var/lib/apt/lists/*

ENV CUDA_HOME=/usr/local/lib/python3.10/dist-packages/nvidia/cuda_runtime
ENV CUDA_TOOLKIT_ROOT_DIR=${CUDA_HOME}
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# Change working directory to /app for proper relative pathing
WORKDIR /app

# Copy application code
COPY ./dimos ./dimos
COPY ./tests ./tests
COPY ./assets ./assets
COPY ./dimos/__init__.py ./
COPY ./docker/simulation/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD [ "/isaac-sim/python.sh", "/app/tests/isaacsim/stream_camera.py" ] 
# For testing
#CMD ["tail", "-f", "/dev/null"]