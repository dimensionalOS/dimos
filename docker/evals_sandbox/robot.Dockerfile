# The sandbox's robot side: dimOS, for the robot container (any blueprint behind a zenoh
# router, recording itself) and the model trace proxy. See docs/usage/evals-sandbox.md.
# Build from the repo root:
#   docker build -f docker/evals_sandbox/robot.Dockerfile -t dimensional/evals-robot .
FROM ubuntu:24.04

ARG PYTHON_VERSION=3.12

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git build-essential pkg-config \
        libgl1 libegl1 libglib2.0-0 libvulkan1 mesa-vulkan-drivers libgfortran5 libgomp1 libturbojpeg portaudio19-dev iproute2 \
    && rm -rf /var/lib/apt/lists/*

# NVIDIA's Vulkan and EGL drivers, for a robot container given the GPU (--robot-gpu): the NVIDIA
# Container Toolkit mounts their libraries but not these manifests. Without the GPU they name a
# library that isn't there, and the loaders skip them.
RUN mkdir -p /usr/share/vulkan/icd.d /usr/share/glvnd/egl_vendor.d \
    && echo '{"file_format_version": "1.0.1", "ICD": {"library_path": "libGLX_nvidia.so.0", "api_version": "1.3"}}' \
        > /usr/share/vulkan/icd.d/nvidia_icd.json \
    && echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' \
        > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

ENV XDG_CACHE_HOME=/cache \
    XDG_STATE_HOME=/state \
    XDG_CONFIG_HOME=/config

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

WORKDIR /app

# Dependencies first so code edits don't invalidate the deps layer.
COPY pyproject.toml uv.lock README.md setup.py ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv python install "${PYTHON_VERSION}" \
    && uv sync --frozen --no-default-groups --extra sim --no-install-project --python "${PYTHON_VERSION}"

COPY . /app
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --no-default-groups --extra sim --python "${PYTHON_VERSION}"

ENV PATH="/app/.venv/bin:${PATH}" \
    VIEWER=none \
    DIMOS_TRANSPORT=zenoh \
    PYTHONUNBUFFERED=1

# dimOS reads data from its checkout's data/ when it finds a .git there, and otherwise clones
# a second copy. This image is a checkout without history whose data/ is mounted in
# (DimosRobot.data), so mark it as the checkout.
RUN mkdir -p /state /config /app/.git /app/data
CMD ["bash"]
