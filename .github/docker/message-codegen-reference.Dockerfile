FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-venv python3-numpy \
    ros-jazzy-common-interfaces ros-jazzy-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/cdr-python \
    && /opt/cdr-python/bin/pip install rosbags==0.11.0
ENV PATH="/opt/cdr-python/bin:$PATH"

# This image is an independent conformance oracle. DimOS builds and users do not
# depend on it or install ROS to generate or use messages.
ENTRYPOINT ["/bin/bash", "-c"]
