#!/bin/bash
# One-command R1 Lite blueprint launcher — run on the LAPTOP (not in the
# container, not on the robot).
#
#   ./scripts/r1lite_test/run_r1lite.sh                    # r1lite-coordinator
#   ./scripts/r1lite_test/run_r1lite.sh r1lite-keyboard-teleop
#
# Does three things:
#   1. Starts the rerun viewer on the laptop (port 9877) if not already up.
#      Viewer must run on the HOST: launching GUIs inside the container
#      fails (X11 auth + software-GL crashes — see BRINGUP_LOG Day 3).
#      Install once with: uv tool install rerun-sdk==0.29.2
#   2. Ensures the dev container is running.
#   3. Runs the blueprint inside the container with VIEWER=rerun-connect,
#      so the bridge streams to the laptop viewer (host networking makes
#      127.0.0.1:9877 the same place for both).
#
# Robot-side prerequisites are NOT handled here (stack up, RC mode 5 for
# chassis) — see RUNBOOK.md.

set -e

# --web: serve the viewer as a headless in-container sidecar and view in
# the BROWSER at http://127.0.0.1:9090?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy
# (dimos' built-in rerun-web mode is known-broken: rr.serve_grpc()
# GIL-deadlocks in forkserver workers — BRINGUP_LOG Day 3. The sidecar
# keeps the rust server in its own process, which is why it works.)
WEB=0
if [ "$1" = "--web" ]; then
    WEB=1
    shift
fi

BLUEPRINT="${1:-r1lite-coordinator}"
CONTAINER=dimos-dev-r1lite
IMAGE=ghcr.io/dimensionalos/ros-dev:dev
REPO_ROOT="$(git rev-parse --show-toplevel)"

# One-time provisioning: create the dev container if this machine doesn't
# have it yet (fresh clone). Mounts THIS checkout at /app, host networking
# (required for DDS multicast to the robot).
if ! docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "[run_r1lite] container $CONTAINER not found — creating it"
    echo "[run_r1lite] (needs ghcr access: gh auth token | docker login ghcr.io -u <user> --password-stdin)"
    docker run -d --name "$CONTAINER" --network host \
        -v "$REPO_ROOT":/app \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e PYTHONUNBUFFERED=1 -e PYTHONPATH=/app \
        -it "$IMAGE" /bin/bash >/dev/null
fi
docker start "$CONTAINER" >/dev/null 2>&1 || true

# The venv must be the container-built py3.10 one (Humble rclpy). Host
# syncs (py3.12) silently break it — rebuild takes a few minutes, so we
# refuse loudly instead of doing it behind your back.
if ! docker exec "$CONTAINER" bash -c 'test -x /app/.venv/bin/python && /app/.venv/bin/python -c "import sys; sys.exit(sys.version_info[:2] != (3,10))"' 2>/dev/null; then
    echo "[run_r1lite] ERROR: /app/.venv is missing or not the container py3.10 build."
    echo "[run_r1lite] Fix (one-time, ~3 min):"
    echo "    docker exec -it $CONTAINER bash -c 'cd /app && rm -rf .venv && UV_PYTHON=3.10 uv sync --all-extras --no-extra dds --no-extra unitree-dds'"
    exit 1
fi

if [ "$WEB" = "1" ]; then
    if ! ss -tln | grep -q ':9877 '; then
        echo "[run_r1lite] starting headless web sidecar in container (:9877 grpc, :9090 browser)"
        docker exec -d "$CONTAINER" rerun --serve-web --port 9877 --memory-limit 2GB
        sleep 2
    fi
    echo "[run_r1lite] BROWSER VIEWER:"
    echo "    http://127.0.0.1:9090?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy"
elif ! ss -tln | grep -q ':9877 '; then
    RERUN_BIN="$(command -v rerun || echo "$HOME/.local/bin/rerun")"
    if [ -x "$RERUN_BIN" ]; then
        echo "[run_r1lite] starting rerun viewer on :9877"
        ("$RERUN_BIN" --port 9877 >/dev/null 2>&1 &)
        sleep 2
    else
        echo "[run_r1lite] WARNING: rerun not found on host — install with:"
        echo "    uv tool install rerun-sdk==0.29.2"
        echo "continuing headless (viewer panes will not appear)"
    fi
else
    echo "[run_r1lite] rerun viewer already up on :9877"
fi

echo "[run_r1lite] launching $BLUEPRINT in container (Ctrl-C stops it)"
exec docker exec -it "$CONTAINER" bash -c "
    cd /app &&
    source .venv/bin/activate &&
    source /opt/ros/humble/setup.bash &&
    export ROS_DOMAIN_ID=2 &&
    export VIEWER=rerun-connect &&
    dimos run $BLUEPRINT
"
