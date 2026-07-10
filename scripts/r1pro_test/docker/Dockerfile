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
    && uv pip install --system --no-cache -r /app/pyproject.toml \
    # matplotlib is imported by RerunBridgeModule (depth/pointcloud colormaps)
    # but in pyproject it lives only in the `manipulation` extra, so the
    # base-deps install above misses it. The bridge deploys even headless
    # (RERUN_OPEN=none), so without this its start() dies with
    # "No module named 'matplotlib'". Install it explicitly.
    && uv pip install --system --no-cache matplotlib

# `docker exec -it <container> bash` bypasses the ENTRYPOINT, so an interactive
# shell never sources ROS and rclpy imports fail ("rclpy is not installed. ROS
# pubsub requires ROS 2") when you run a blueprint by hand. Source it from
# .bashrc so manual `dimos run` in an exec shell — the default workflow, since
# PID 1 idles — has rclpy on the path.
RUN echo "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash" >> /root/.bashrc

COPY MANIFEST.in README.md /app/
COPY dimos /app/dimos
RUN uv pip install --system --no-cache --no-deps -e /app

COPY scripts /app/scripts

# Robot-side defaults. GlobalConfig is pydantic-settings, so these are plain
# env vars (overridable at `docker run -e ...`):
#  - ROS_DOMAIN_ID=1  → new-gen R1 Pro (V2.3.0). R1ProConnection reads this
#    from the env — nothing hardcodes it in dimos — so it MUST match the
#    robot or the container discovers zero topics. Old-gen robot was 41.
#  - RERUN_OPEN=none  → headless; the bridge's gRPC server is the viewing
#    endpoint, no viewer binary on the robot.
#  - RERUN_HOST=0.0.0.0 → bind the gRPC server on all interfaces so the
#    untethered laptop can connect (default is 127.0.0.1 = unreachable).
#  - N_WORKERS=3 → connection / coordinator / rerun bridge in separate
#    processes (blueprint also sets this; env keeps it explicit).
#  - LCM_DEFAULT_URL port 7300 → LCM's default port is 7667, which COLLIDES
#    with FastDDS on ROS_DOMAIN_ID=1: DDS computes ports as 7400 + 250*domain
#    + offsets, so domain 1 (base 7650) hands out 7661/7663/7665/7667/... to
#    participants, and under --network=host a robot DDS node (e.g.
#    signal_camera_node) grabs 7667 first → LCM can't bind → every worker dies
#    with "bind: Address already in use". 7300 sits below the DDS base (7400)
#    where no domain reaches. Keep the multicast GROUP (239.255.76.67) matching
#    the entrypoint's loopback route. Old-gen domain 41 put DDS at ~17650, so
#    this never bit until the new-gen domain-1 move.
ENV ROS_DOMAIN_ID=1 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    RERUN_OPEN=none \
    RERUN_HOST=0.0.0.0 \
    N_WORKERS=3 \
    LCM_DEFAULT_URL="udpm://239.255.76.67:7300?ttl=0"

ENTRYPOINT ["/app/scripts/r1pro_test/docker/entrypoint.sh"]
# Idle by default so PID 1 doesn't auto-launch the coordinator: `dimos run`
# prompts interactively for system-config approval, which a detached PID 1
# can't answer, and a second manual instance would collide on the LCM/rerun
# ports. Exec in and run a blueprint by hand (see run.sh), or override this
# CMD to auto-run (`run.sh dimos run r1pro-coordinator`).
CMD ["sleep", "infinity"]
