#!/bin/bash
# One-time dimos provisioning for an R1 Lite ONBOARD PC — single run.
# RUN THIS ON THE ROBOT (ssh r1lite). Standard flow for any fresh R1 Lite
# (repo is public — no credentials needed on the robot):
#     git clone -b krishna/task/r1lite-integration \
#         https://github.com/dimensionalOS/dimos.git ~/dimos
#     cd ~/dimos && bash scripts/r1lite_test/r1lite_dimos_install.sh
#
# Idempotent; prompts before every host change. Host changes (with consent):
#   docker.io (apt) · container "dimos-dev-r1lite" · py3.10 venv in it ·
#   /etc/sysctl.d/60-dimos.conf. Does NOT touch the Galaxea stack.
# One pause mid-run: the ghcr IMAGE is private (repo isn't), so it's
# transferred from the laptop over the cable when the script asks.

set -e

BRANCH=krishna/task/r1lite-integration
REPO_URL=https://github.com/dimensionalOS/dimos.git
IMAGE=ghcr.io/dimensionalos/ros-dev:dev
CONTAINER=dimos-dev-r1lite
SCRIPT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && git rev-parse --show-toplevel 2>/dev/null || true)"
DIMOS_DIR="${SCRIPT_REPO:-$HOME/dimos}"

step()    { echo; echo "=== [$1] $2"; }
confirm() { read -r -p "    Proceed? [y/N] " a; [ "$a" = "y" ]; }

step 1 "Preflight"
[ "$(uname -m)" = "x86_64" ] || { echo "unexpected arch"; exit 1; }
avail_gb=$(df --output=avail -BG "$HOME" | tail -1 | tr -dc '0-9')
[ "$avail_gb" -gt 40 ] || { echo "need >40GB free, have ${avail_gb}G"; exit 1; }
timeout 5 curl -sI https://github.com >/dev/null || { echo "no internet"; exit 1; }
echo "    arch/disk/internet OK (${avail_gb}G free)"

step 2 "Docker"
if ! command -v docker >/dev/null; then
    echo "    docker missing. Will: sudo apt-get install -y docker.io && usermod -aG docker $USER"
    confirm || exit 1
    sudo apt-get update -qq && sudo apt-get install -y docker.io
    sudo usermod -aG docker "$USER"
    echo "    installed (group applies on next login; this run continues via sudo)"
fi
# Group membership may not be active in this session yet — fall back to sudo.
DOCKER="docker"
docker info >/dev/null 2>&1 || DOCKER="sudo docker"
$DOCKER info >/dev/null || { echo "docker not usable"; exit 1; }
echo "    docker usable (as: $DOCKER)"

step 3 "dimos checkout at $DIMOS_DIR"
if [ -d "$DIMOS_DIR/.git" ]; then
    echo "    using existing checkout: $(git -C "$DIMOS_DIR" rev-parse --abbrev-ref HEAD) @ $(git -C "$DIMOS_DIR" rev-parse --short HEAD)"
else
    git clone --branch "$BRANCH" "$REPO_URL" "$DIMOS_DIR"
fi

step 4 "Container image"
while ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; do
    if $DOCKER pull "$IMAGE" 2>/dev/null; then break; fi
    echo "    Pull failed (image is private). From the LAPTOP, run:"
    echo "        docker save $IMAGE | ssh r1lite \"echo 1 | sudo -S docker load\""
    echo "    (~15GB over the cable, several minutes)"
    read -r -p "    Press Enter when the transfer is done (or Ctrl-C to abort)... " _
done
echo "    image present"

step 5 "Container $CONTAINER"
if ! $DOCKER ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    # --network host: DDS to the Galaxea stack.
    # -v /dev/shm: CRITICAL same-host FastDDS uses shared memory; a private
    #   container /dev/shm means "topics visible, zero messages".
    # X11 mounts: allow ssh -X forwarded pygame teleop.
    $DOCKER run -d --name "$CONTAINER" --network host \
        -v "$DIMOS_DIR":/app \
        -v /dev/shm:/dev/shm \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$HOME/.Xauthority":/root/.Xauthority \
        -e PYTHONUNBUFFERED=1 -e PYTHONPATH=/app \
        -it "$IMAGE" /bin/bash >/dev/null
fi
$DOCKER start "$CONTAINER" >/dev/null 2>&1 || true
echo "    container running"

step 6 "py3.10 venv in container (~10 min first run)"
if $DOCKER exec "$CONTAINER" bash -c 'test -x /app/.venv/bin/python && /app/.venv/bin/python -c "import sys; sys.exit(sys.version_info[:2] != (3,10))"' 2>/dev/null; then
    echo "    venv OK"
else
    $DOCKER exec "$CONTAINER" bash -c 'cd /app && rm -rf .venv && UV_PYTHON=3.10 uv sync --all-extras --no-extra dds --no-extra unitree-dds'
fi

step 7 "Host sysctls (/etc/sysctl.d/60-dimos.conf: UDP buffers, lo multicast)"
if [ -f /etc/sysctl.d/60-dimos.conf ]; then
    echo "    already applied"
elif confirm; then
    sudo tee /etc/sysctl.d/60-dimos.conf >/dev/null <<'EOF'
net.core.rmem_max=67108864
net.core.rmem_default=67108864
EOF
    sudo sysctl --system >/dev/null
    sudo ip link set lo multicast on
fi

step 8 "Verification"
$DOCKER exec "$CONTAINER" bash -c 'cd /app && source .venv/bin/activate && python -c "
import rclpy, dimos
from dimos.robot.galaxea.r1lite.connection import R1LiteConnection
R1LiteConnection.blueprint()
print(\"    imports + blueprint: OK\")"'
echo "    DDS cross-boundary check (needs the Galaxea stack running):"
$DOCKER exec "$CONTAINER" bash -c 'cd /app && source .venv/bin/activate && source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=2 && timeout 15 python - <<PYEOF
import time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("install_verify")
c = [0]
n.create_subscription(JointState, "/hdas/feedback_arm_left", lambda m: c.__setitem__(0, c[0]+1),
                      QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
end = time.time() + 8
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
print(f"    feedback_arm_left msgs in 8s: {c[0]}", "-- DDS OK" if c[0] > 100 else "-- FAIL (stack up? /dev/shm shared?)")
PYEOF'

echo
echo "=== install complete. Launch blueprints with:"
echo "    cd $DIMOS_DIR && ./scripts/r1lite_test/run_r1lite.sh"
echo "    (log out/in once so plain 'docker' works without sudo)"
