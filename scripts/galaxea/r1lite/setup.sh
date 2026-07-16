#!/bin/bash
# One-command dimos deployment for a Galaxea R1 Lite onboard PC.
#
#   git clone https://github.com/dimensionalOS/dimos.git ~/dimos   # public
#   cd ~/dimos
#   bash scripts/galaxea/r1lite/setup.sh [--tar /path/to/dimos-r1lite.tar.gz]
#
# Idempotent; prompts before host changes. Installs: docker + compose,
# sysctls, /opt/dimos/{compose.yaml,.env}, /usr/local/bin/dimos wrapper,
# and the dimos-r1lite runtime image (pull -> --tar file -> build-on-robot
# ladder). Ends with a live DDS verification against the vendor stack.
# Does NOT touch the Galaxea stack. Update later = edit DIMOS_IMAGE in
# /opt/dimos/.env, `docker compose up -d`. Remove = `docker compose down`.
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
REGISTRY=ghcr.io/dimensionalos
DEPLOY_DIR=/opt/dimos

TARBALL=""
[ "$1" = "--tar" ] && TARBALL="$2"

DIMOS_VERSION="$(grep -m1 '^version' "$REPO_ROOT/pyproject.toml" | sed 's/.*"\(.*\)".*/\1/')"
TAG="dimos-r1lite:${DIMOS_VERSION}-r1lite.1"

step()    { echo; echo "=== [$1] $2"; }
confirm() { read -r -p "    Proceed? [y/N] " a; [ "$a" = "y" ]; }

step 1 "Preflight"
[ "$(uname -m)" = "x86_64" ] || { echo "unexpected arch: $(uname -m)"; exit 1; }
avail_gb=$(df --output=avail -BG "$HOME" | tail -1 | tr -dc '0-9')
[ "$avail_gb" -gt 20 ] || { echo "need >20GB free, have ${avail_gb}G"; exit 1; }
echo "    arch/disk OK (${avail_gb}G free); image tag: $TAG"

step 2 "Docker + compose"
if ! command -v docker >/dev/null; then
    echo "    docker missing. Will: sudo apt-get install -y docker.io docker-compose-v2"
    confirm || exit 1
    sudo apt-get update -qq && sudo apt-get install -y docker.io docker-compose-v2
    sudo usermod -aG docker "$USER"
    echo "    installed (group applies on next login; this run continues via sudo)"
elif ! docker compose version >/dev/null 2>&1 && ! sudo docker compose version >/dev/null 2>&1; then
    echo "    compose plugin missing. Will: sudo apt-get install -y docker-compose-v2"
    confirm || exit 1
    sudo apt-get update -qq && sudo apt-get install -y docker-compose-v2
fi
DOCKER="docker"
docker info >/dev/null 2>&1 || DOCKER="sudo docker"
$DOCKER info >/dev/null || { echo "docker not usable"; exit 1; }
echo "    docker usable (as: $DOCKER)"

step 3 "Runtime image ($TAG)"
if $DOCKER image inspect "$TAG" >/dev/null 2>&1; then
    echo "    already present"
elif $DOCKER pull "$REGISTRY/$TAG" 2>/dev/null; then
    $DOCKER tag "$REGISTRY/$TAG" "$TAG"
    echo "    pulled from registry"
elif [ -n "$TARBALL" ] && [ -f "$TARBALL" ]; then
    echo "    loading from $TARBALL (a few minutes)..."
    case "$TARBALL" in
        *.gz) gunzip -c "$TARBALL" | $DOCKER load ;;
        *)    $DOCKER load -i "$TARBALL" ;;
    esac
else
    echo "    Registry unavailable and no --tar given. Options:"
    echo "      a) copy a tarball to this robot, rerun with: setup.sh --tar <file>"
    echo "      b) build on this robot now (~30-60 min, needs internet)"
    read -r -p "    Build now? [y/N] " a
    [ "$a" = "y" ] || exit 1
    bash "$REPO_ROOT/scripts/galaxea/docker/build.sh"
fi
$DOCKER image inspect "$TAG" >/dev/null || { echo "image still missing"; exit 1; }

step 4 "Host sysctls (UDP buffers for DDS/LCM)"
if [ -f /etc/sysctl.d/60-dimos.conf ]; then
    echo "    already applied"
elif confirm; then
    sudo tee /etc/sysctl.d/60-dimos.conf >/dev/null <<'EOF'
net.core.rmem_max=67108864
net.core.rmem_default=67108864
EOF
    sudo sysctl --system >/dev/null
fi

step 5 "Deploy files -> $DEPLOY_DIR"
sudo mkdir -p "$DEPLOY_DIR"
sudo cp "$HERE/compose.yaml" "$DEPLOY_DIR/compose.yaml"
if [ ! -f "$DEPLOY_DIR/.env" ]; then
    sudo tee "$DEPLOY_DIR/.env" >/dev/null <<EOF
DIMOS_IMAGE=$TAG
ROS_DOMAIN_ID=2
VIEWER=rerun-connect
EOF
    echo "    wrote .env (image tag, domain 2, rerun-connect viewer)"
else
    echo "    .env exists — leaving it (edit DIMOS_IMAGE there to update/rollback)"
fi
sudo install -m 0755 "$HERE/dimos-wrapper.sh" /usr/local/bin/dimos
touch "$HOME/.Xauthority"   # pre-empt docker dir-creation trap for ssh -X teleop
echo "    installed /usr/local/bin/dimos wrapper"

step 6 "Start services"
(cd "$DEPLOY_DIR" && $DOCKER compose up -d)
$DOCKER compose -f "$DEPLOY_DIR/compose.yaml" ps

step 7 "DDS verification (needs the Galaxea stack running)"
$DOCKER compose -f "$DEPLOY_DIR/compose.yaml" exec -T dimos bash -c '
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID:-2}"' && timeout 15 python3 - <<PYEOF
import time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("setup_verify")
c = [0]
n.create_subscription(JointState, "/hdas/feedback_arm_left", lambda m: c.__setitem__(0, c[0]+1),
                      QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
end = time.time() + 8
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
print(f"    feedback_arm_left msgs in 8s: {c[0]}", "-- DDS OK" if c[0] > 100 else "-- FAIL (vendor stack up? ipc:host?)")
PYEOF'

echo
echo "=== dimos deployed. From this terminal:"
echo "    dimos list"
echo "    dimos run r1lite-keyboard-teleop      # via ssh -X"
echo "    Browser viewer: http://<robot-ip>:9090?url=rerun%2Bhttp%3A%2F%2F<robot-ip>%3A9877%2Fproxy"
echo "    Update: edit DIMOS_IMAGE in $DEPLOY_DIR/.env, then: docker compose -f $DEPLOY_DIR/compose.yaml up -d"
