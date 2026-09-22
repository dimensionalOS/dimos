FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-pytest python3-typing-extensions \
    ros-jazzy-common-interfaces ros-jazzy-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

# This image is an independent conformance oracle. DimOS builds and users do not
# depend on it or install ROS to generate or use messages.
ENTRYPOINT ["/bin/bash", "-c"]
