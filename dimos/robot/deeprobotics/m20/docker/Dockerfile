# dimos M20 NOS Docker Image
#
# Extends pre-built deps image (dimos/m20-deps:latest) with dimos source.
# The deps image was created by interactively installing all Python
# dependencies into the DeepRobotics Humble base container, then
# committing it (to avoid OOM during docker build on 4-core NOS).
#
# To rebuild deps image from scratch:
#   docker run -d --name deps-builder --network host dimos/m20-humble-base:latest sleep 3600
#   docker exec deps-builder bash -c '<install commands>'
#   docker commit deps-builder dimos/m20-deps:latest
#
# Build context must be the dimos repo root.
#
# Usage:
#   docker build -t dimos-m20-nos -f dimos/robot/deeprobotics/m20/docker/Dockerfile .

ARG BASE_IMAGE=dimos/m20-deps:latest
FROM ${BASE_IMAGE}

# Copy dimos source and install as editable package
COPY . /opt/dimos/src
RUN /opt/dimos-venv/bin/pip install --no-cache-dir -e /opt/dimos/src

# Rebuild drdds ROS2 package from vendor .msg definitions.
# The base image has drdds pre-compiled for Python 3.8 with potentially
# ABI-incompatible rosidl typesupport.  This colcon build regenerates
# everything against the container's ROS2 Humble (Python 3.10, matching
# rosidl/FastRTPS versions).
COPY dimos/robot/deeprobotics/m20/docker/build_drdds_bindings.sh /opt/dimos/docker/build_drdds_bindings.sh
RUN chmod +x /opt/dimos/docker/build_drdds_bindings.sh \
    && bash -c "source /opt/ros/humble/setup.bash && /opt/dimos/docker/build_drdds_bindings.sh"

# Copy Docker support files
COPY dimos/robot/deeprobotics/m20/docker/entrypoint.sh /opt/dimos/entrypoint.sh
COPY dimos/robot/deeprobotics/m20/docker/fastdds.xml /opt/dimos/docker/fastdds.xml
COPY dimos/robot/deeprobotics/m20/docker/launch_nos.py /opt/dimos/launch_nos.py
RUN chmod +x /opt/dimos/entrypoint.sh

WORKDIR /opt/dimos
ENTRYPOINT ["/opt/dimos/entrypoint.sh"]
CMD ["/opt/dimos-venv/bin/python", "/opt/dimos/launch_nos.py"]
