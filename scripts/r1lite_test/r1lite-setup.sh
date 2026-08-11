#!/bin/bash
# R1 Lite first-time setup wizard: provisions a fresh robot to run the
# pinned dimos image with a 3-command session workflow.
#
#     bash <(curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/feat/krishna/r1lite-v4-docker/scripts/r1lite_test/r1lite-setup.sh)
#
# Prompted, idempotent, consent before every host change, verification at
# the end. Design: FDEIssues/galaxea-integration/R1LITE_SETUP_WIZARD_PLAN.md
# (laptop copy) — dim-installer interface philosophy, image-based install.
#
# Test overrides (laptop dry-runs): R1LITE_SETUP_ALLOW_NON_ROBOT=1 skips
# the robot preflight and host persistence; R1LITE_SETUP_YES=1 answers
# every consent prompt yes; R1LITE_IMAGE_TAG pins a different tag.

set -o pipefail

BRANCH="feat/krishna/r1lite-v4-docker"
RAW_BASE="https://raw.githubusercontent.com/dimensionalOS/dimos/$BRANCH/scripts/r1lite_test"
REPO_REF="ghcr.io/dimensionalos/dimos-r1lite"
DEFAULT_TAG="0.0.14b1-r1lite-dev.14"
TAG="${R1LITE_IMAGE_TAG:-$DEFAULT_TAG}"
STATE_DIR="$HOME/.config/dimos-r1lite"
STATE="$STATE_DIR/image.env"
BIN_DIR="$HOME/.local/bin"
LOG="/tmp/r1lite_setup_$(date +%Y%m%d_%H%M%S).log"

say()  { echo "[setup] $*" | tee -a "$LOG"; }
fail() { say "FAIL: $*"; say "log: $LOG"; exit 1; }
confirm() {
    [ "${R1LITE_SETUP_YES:-}" = "1" ] && return 0
    read -r -p "[setup] $1 [y/N] " a; [ "$a" = "y" ]
}

step() { echo; say "=== $* ==="; }

step "1/8 preflight"
[ "$(uname -m)" = "x86_64" ] || fail "expected x86_64, got $(uname -m)"
if [ "${R1LITE_SETUP_ALLOW_NON_ROBOT:-}" != "1" ]; then
    [ -d "$HOME/galaxea/install" ] \
        || fail "no Galaxea install in \$HOME — run this ON the R1 Lite (or set R1LITE_SETUP_ALLOW_NON_ROBOT=1 for a dry run)"
fi
avail_gb=$(df --output=avail -BG "$HOME" | tail -1 | tr -dc '0-9')
[ "${avail_gb:-0}" -gt 20 ] || fail "need >20 GB free in \$HOME, have ${avail_gb}G"
timeout 8 curl -sI https://ghcr.io >/dev/null || fail "no route to ghcr.io"
say "arch/disk/network OK (${avail_gb}G free)"

step "2/8 docker"
if command -v docker >/dev/null && docker info >/dev/null 2>&1; then
    say "docker present and usable"
else
    say "docker missing or this user is not in the docker group."
    confirm "install docker.io and add $USER to the docker group (sudo)?" || fail "docker required"
    sudo apt-get update -qq && sudo apt-get install -y docker.io || fail "docker install failed"
    sudo usermod -aG docker "$USER"
    say "IMPORTANT: log out, ssh back in (group change), and re-run this wizard."
    exit 0
fi

step "3/8 image pull -> pinned digest"
if ! docker pull "$REPO_REF:$TAG" 2>&1 | tee -a "$LOG"; then
    say "pull failed — the package may be private."
    confirm "docker login ghcr.io now (needs a read:packages token)?" || fail "cannot pull image"
    read -r -p "[setup] GitHub username: " ghuser
    docker login ghcr.io -u "$ghuser" || fail "login failed"
    docker pull "$REPO_REF:$TAG" || fail "pull still failing"
fi
DIGEST=$(docker inspect "$REPO_REF:$TAG" --format '{{index .RepoDigests 0}}' | cut -d@ -f2)
REVISION=$(docker image inspect "$REPO_REF:$TAG" \
    --format '{{index .Config.Labels "org.opencontainers.image.revision"}}')
[ -n "$DIGEST" ] || fail "could not resolve the image digest"
say "pinned: $TAG"
say "  digest:   $DIGEST"
say "  revision: $REVISION"

step "4/8 host persistence (loopback DDS across reboots)"
if [ "${R1LITE_SETUP_ALLOW_NON_ROBOT:-}" = "1" ]; then
    say "skipped (non-robot dry run)"
elif [ -f /etc/systemd/system/dimos-r1lite-net.service ]; then
    say "systemd unit already installed"
else
    say "will install a systemd oneshot: lo multicast on + 224.0.0.0/4 via lo"
    say "(sysctl cannot set link flags; without this every reboot goes deaf)"
    if confirm "install /etc/systemd/system/dimos-r1lite-net.service (sudo)?"; then
        sudo tee /etc/systemd/system/dimos-r1lite-net.service >/dev/null <<'UNIT'
[Unit]
Description=dimos R1 Lite loopback DDS prerequisites
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set lo multicast on
ExecStart=-/usr/sbin/ip route add 224.0.0.0/4 dev lo
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT
        sudo systemctl daemon-reload
        sudo systemctl enable --now dimos-r1lite-net.service || fail "unit failed to start"
        say "unit installed and active"
    else
        say "SKIPPED — you will need the two ip commands by hand after every reboot"
    fi
fi

step "5/8 install session tools (vendor_up.sh, r1)"
mkdir -p "$BIN_DIR"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-/nonexistent}")" 2>/dev/null && pwd)"
for tool in vendor_up.sh r1; do
    if [ -n "$script_dir" ] && [ -f "$script_dir/$tool" ]; then
        cp "$script_dir/$tool" "$BIN_DIR/$tool"
    else
        curl -fsSL "$RAW_BASE/$tool" -o "$BIN_DIR/$tool" || fail "could not fetch $tool"
    fi
    chmod +x "$BIN_DIR/$tool"
    say "installed $BIN_DIR/$tool"
done
case ":$PATH:" in *":$BIN_DIR:"*) ;; *) say "NOTE: add $BIN_DIR to PATH (usually a re-login suffices)";; esac

step "6/8 write state (pin + robot passport)"
mkdir -p "$STATE_DIR"
{
    echo "IMAGE_TAG=$TAG"
    echo "IMAGE_DIGEST=$DIGEST"
    echo "IMAGE_REVISION=$REVISION"
    echo "INSTALL_DATE=$(date -Is)"
    echo "ROBOT_HOSTNAME=$(hostname)"
    echo "ROBOT_IPS=\"$(hostname -I 2>/dev/null | tr -d '\n')\""
} > "$STATE"
say "state written: $STATE"

step "7/8 DDS self-test (the image must not be deaf)"
docker run --rm --network host --entrypoint dds-selftest "$REPO_REF@$DIGEST" 2>&1 | tee -a "$LOG" \
    || fail "in-image DDS self-test failed — do NOT proceed; this image cannot hear anything"
docker run -d --rm --name r1setup-pub --network host --ipc host --entrypoint bash \
    "$REPO_REF@$DIGEST" -lc 'source /opt/ros/humble/setup.bash && timeout 60 ros2 topic pub /r1setup_check std_msgs/msg/String "{data: x}" -r 5' >/dev/null
sleep 8
if docker run --rm --network host --ipc host --entrypoint bash "$REPO_REF@$DIGEST" \
    -lc 'source /opt/ros/humble/setup.bash && timeout 15 ros2 topic echo /r1setup_check std_msgs/msg/String --once >/dev/null'; then
    say "cross-container discovery: PASS"
else
    docker rm -f r1setup-pub >/dev/null 2>&1
    fail "cross-container DDS discovery failed on this host"
fi
docker rm -f r1setup-pub >/dev/null 2>&1

step "8/8 done"
say "installation verified. Every session from now on:"
say ""
say "    r1 up        # vendor stack up + health-gated"
say "    r1 shell     # container terminal, env baked -> dimos run <blueprint>"
say "    r1 status    # when anything looks off, run this FIRST"
say ""
say "arming stays deliberate: preflight phase2 + arm from a second r1 shell."
say "evidence log: $LOG"
