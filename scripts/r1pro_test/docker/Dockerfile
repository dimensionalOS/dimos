# On-robot dimos image for the Galaxea R1 Pro.
#
# Runs the r1pro-coordinator blueprint on the robot itself so no bulk sensor
# data ever crosses a physical link (the failure class documented in
# ../STREAM_RELIABILITY_DIAGNOSTIC.md). The laptop only connects a rerun
# viewer to the bridge's gRPC server over WiFi.
#
# Build context is the REPO ROOT (see build.sh):
#   docker build -f scripts/r1pro_test/docker/Dockerfile -t dimos-r1pro .
#
# ros:humble-ros-base is multi-arch; on the Jetson (JetPack 6 = Ubuntu 22.04
# userspace) the arm64 image runs natively. No CUDA needed — the pipeline is
# CPU turbojpeg/OpenCV.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        build-essential \
        # Native half of PyTurboJPEG — without it JpegLcmTransport fails at
        # runtime as "decode error" on every color stream (2026-05-22 incident;
        # package name on 22.04 is libturbojpeg, not libturbojpeg0).
        libturbojpeg \
        # opencv-python runtime libs
        libgl1 \
        libglib2.0-0 \
        # entrypoint uses `ip` to route LCM multicast via loopback
        iproute2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Dependency layer first (cached across code edits), source after.
COPY pyproject.toml /app/pyproject.toml
RUN pip3 install --no-cache-dir uv \
    && uv pip install --system --no-cache -r /app/pyproject.toml

COPY MANIFEST.in README.md /app/
COPY dimos /app/dimos
RUN uv pip install --system --no-cache --no-deps -e /app

COPY scripts /app/scripts

# Robot-side defaults. GlobalConfig is pydantic-settings, so these are plain
# env vars (overridable at `docker run -e ...`):
#  - RERUN_OPEN=none  → headless; the bridge's gRPC server is the viewing
#    endpoint, no viewer binary on the robot.
#  - RERUN_HOST=0.0.0.0 → bind the gRPC server on all interfaces so the
#    untethered laptop can connect (default is 127.0.0.1 = unreachable).
#  - N_WORKERS=3 → connection / coordinator / rerun bridge in separate
#    processes (blueprint also sets this; env keeps it explicit).
ENV ROS_DOMAIN_ID=41 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    RERUN_OPEN=none \
    RERUN_HOST=0.0.0.0 \
    N_WORKERS=3

ENTRYPOINT ["/app/scripts/r1pro_test/docker/entrypoint.sh"]
CMD ["dimos", "run", "r1pro-coordinator"]
