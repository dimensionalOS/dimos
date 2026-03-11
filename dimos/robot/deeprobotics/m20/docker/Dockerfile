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
# Requires: DOCKER_BUILDKIT=1 (enabled by deploy.sh)

ARG BASE_IMAGE=ghcr.io/aphexcx/m20-deps:latest
FROM ${BASE_IMAGE}

# --- Layer 0: System packages ---
RUN apt-get update && apt-get install -y --no-install-recommends openssh-client sshpass \
    && rm -rf /var/lib/apt/lists/*

# --- Layer 1: Install Python deps (cached until pyproject.toml/setup.py change) ---
# Copy only packaging metadata so this layer survives pure source changes
COPY pyproject.toml setup.py /opt/dimos/src/
# Create minimal stub so pip can resolve deps and build a trivial C++ extension
RUN mkdir -p /opt/dimos/src/dimos/navigation/replanning_a_star \
    && touch /opt/dimos/src/dimos/__init__.py \
    && echo "// stub" > /opt/dimos/src/dimos/navigation/replanning_a_star/min_cost_astar_cpp.cpp
# Install all dependencies (BuildKit cache mount speeds up rebuilds)
# dimos-viewer is automatically skipped on Linux via sys_platform marker in pyproject.toml
RUN --mount=type=cache,target=/root/.cache/pip \
    /opt/dimos-venv/bin/pip install -e "/opt/dimos/src[agents]" && \
    /opt/dimos-venv/bin/pip install --ignore-installed matplotlib

# --- Layer 2: Build drdds ROS2 bindings (cached until .msg files change) ---
# Rebuild drdds ROS2 package from vendor .msg definitions.
# The base image has drdds pre-compiled for Python 3.8 with potentially
# ABI-incompatible rosidl typesupport.  This colcon build regenerates
# everything against the container's ROS2 Humble (Python 3.10, matching
# rosidl/FastRTPS versions).
# Placed before source COPY so it's cached independently of Python source changes.
# Bundled .msg files (fallback when vendor .msg not in base image)
COPY dimos/robot/deeprobotics/m20/docker/drdds_msgs/msg /opt/dimos/docker/drdds_msgs/msg
COPY dimos/robot/deeprobotics/m20/docker/build_drdds_bindings.sh /opt/dimos/docker/build_drdds_bindings.sh
RUN chmod +x /opt/dimos/docker/build_drdds_bindings.sh \
    && bash -c "source /opt/ros/humble/setup.bash && /opt/dimos/docker/build_drdds_bindings.sh"

# --- Layer 3: Copy source + reinstall editable (rebuilds C++ ext, no dep work) ---
COPY . /opt/dimos/src
# Create .git marker so get_data() uses local source instead of cloning from GitHub
RUN mkdir -p /opt/dimos/src/.git
# Decompress command_center.html from LFS archive (LFS pull won't work without real .git)
RUN if [ -f /opt/dimos/src/data/.lfs/command_center.html.tar.gz ]; then \
        tar -xzf /opt/dimos/src/data/.lfs/command_center.html.tar.gz -C /opt/dimos/src/data/ ; \
    fi
# Reinstall editable — rebuilds pybind11 C++ extension with real source, skips dep resolution
RUN --mount=type=cache,target=/root/.cache/pip \
    /opt/dimos-venv/bin/pip install --no-deps -e "/opt/dimos/src"

# --- Layer 4: Docker support files ---
COPY dimos/robot/deeprobotics/m20/docker/entrypoint.sh /opt/dimos/entrypoint.sh
COPY dimos/robot/deeprobotics/m20/docker/fastdds.xml /opt/dimos/docker/fastdds.xml
COPY dimos/robot/deeprobotics/m20/docker/launch_nos.py /opt/dimos/launch_nos.py
RUN chmod +x /opt/dimos/entrypoint.sh

WORKDIR /opt/dimos
ENTRYPOINT ["/opt/dimos/entrypoint.sh"]
CMD ["/opt/dimos-venv/bin/python", "/opt/dimos/launch_nos.py"]
