# dimos M20 NOS Docker Image
#
# Extends pre-built deps image with dimos source. The deps image contains
# the DeepRobotics Humble base + all Python dependencies (pre-built to
# avoid OOM on the 4-core NOS).
#
# Build on Mac (ARM64), push to GHCR, pull on robot:
#   ./deploy.sh push    # Build + push from Mac
#   ./deploy.sh pull    # Pull on NOS via SSH
#   ./deploy.sh start   # Start container
#
# Build context must be the dimos repo root.

ARG BASE_IMAGE=ghcr.io/aphexcx/m20-deps:latest
FROM ${BASE_IMAGE}

# Copy dimos source and install as editable package
COPY . /opt/dimos/src
# Create .git marker so get_data() uses local source instead of cloning from GitHub
RUN mkdir -p /opt/dimos/src/.git
RUN /opt/dimos-venv/bin/pip install --no-cache-dir -e "/opt/dimos/src[agents]" && \
    /opt/dimos-venv/bin/pip install --no-cache-dir --ignore-installed matplotlib

# Rebuild drdds ROS2 package from vendor .msg definitions.
# The base image has drdds pre-compiled for Python 3.8 with potentially
# ABI-incompatible rosidl typesupport.  This colcon build regenerates
# everything against the container's ROS2 Humble (Python 3.10, matching
# rosidl/FastRTPS versions).
# Bundled .msg files (fallback when vendor .msg not in base image)
COPY dimos/robot/deeprobotics/m20/docker/drdds_msgs/msg /opt/dimos/docker/drdds_msgs/msg
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
