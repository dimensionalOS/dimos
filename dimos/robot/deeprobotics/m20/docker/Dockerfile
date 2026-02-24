# dimos M20 NOS Docker Image
#
# Extends DeepRobotics ROS2 Humble base with dimos and dependencies.
# Build context must be the dimos repo root.
#
# Usage:
#   docker build -t dimos-m20-nos -f dimos/robot/deeprobotics/m20/docker/Dockerfile .

ARG BASE_IMAGE=dimos/m20-humble-base:latest
FROM ${BASE_IMAGE}

# System deps for dimos
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-dev \
    libturbojpeg0 libgl1-mesa-glx libglib2.0-0 \
    git curl \
    && rm -rf /var/lib/apt/lists/*

# Create dimos venv with system-site-packages (for rclpy + drdds access)
RUN python3 -m venv --system-site-packages /opt/dimos-venv

# Install dimos Python dependencies
# Split into layers for better caching: heavy deps first, then dimos
RUN /opt/dimos-venv/bin/pip install --no-cache-dir \
    "numpy>=1.26.4" "scipy>=1.15.1" reactivex \
    "dask[distributed]==2025.5.1" "plum-dispatch==2.5.7" \
    "structlog>=25.5.0,<26" pydantic "pydantic-settings>=2.11.0,<3" \
    "typer>=0.19.2,<1" opencv-python-headless lcm sortedcontainers \
    PyTurboJPEG rerun-sdk dimos-lcm \
    uvicorn "starlette>=0.41.0" python-socketio aiohttp \
    av PyYAML

# ARM64 Open3D (unofficial build — no PyPI wheels for aarch64)
RUN /opt/dimos-venv/bin/pip install --no-cache-dir \
    "open3d-unofficial-arm" \
    || echo "WARNING: open3d-unofficial-arm failed — mapping may be limited"

# Copy dimos source and install as editable package
COPY . /opt/dimos/src
RUN /opt/dimos-venv/bin/pip install --no-cache-dir -e /opt/dimos/src

# Copy Docker support files
COPY dimos/robot/deeprobotics/m20/docker/entrypoint.sh /opt/dimos/entrypoint.sh
COPY dimos/robot/deeprobotics/m20/docker/fastdds.xml /opt/dimos/docker/fastdds.xml
COPY dimos/robot/deeprobotics/m20/docker/launch_nos.py /opt/dimos/launch_nos.py
RUN chmod +x /opt/dimos/entrypoint.sh

WORKDIR /opt/dimos
ENTRYPOINT ["/opt/dimos/entrypoint.sh"]
CMD ["/opt/dimos-venv/bin/python", "/opt/dimos/launch_nos.py"]
