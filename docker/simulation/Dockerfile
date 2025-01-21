FROM nvcr.io/nvidia/isaac-sim:4.2.0

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
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Python packages directly
RUN pip install --no-cache-dir \
    av==10.0.0 \
    numpy>=1.21.0 \
    pillow>=9.0.0 \
    opencv-python>=4.5.0 \
    transforms3d \
    scipy

# Create and setup ROS 2 workspace
RUN mkdir -p /home/ros/dev_ws/src && \
    cd /home/ros/dev_ws && \
    git clone https://github.com/yashas-salankimatt/thesis_ros_ws.git src/thesis_ros_ws && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build && \
    echo "source /home/ros/dev_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /app

# Copy application code
COPY ./dimos ./dimos
COPY ./tests ./tests
COPY ./assets ./assets
COPY ./dimos/__init__.py ./
COPY ./docker/simulation/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
#CMD [ "/isaac-sim/python.sh", "/app/tests/isaacsim/stream_camera.py" ] 
# For testing
CMD ["tail", "-f", "/dev/null"]