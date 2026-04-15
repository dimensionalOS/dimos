#!/bin/bash
#
# M20 NOS deployment helper
#
# Sets up NOS prerequisites and manages the drdds SHM bridge.
# The nav stack runs natively via nix — no Docker needed.
#
# Prerequisites:
#   SSH keys copied to NOS and AOS (ssh-copy-id)
#   NOS user has passwordless sudo (or set SUDO_PASS env var)
#
# Usage:
#   ./deploy.sh setup [--host <hostname>]        # NOS prerequisites
#   ./deploy.sh bridge-start [--host <hostname>]  # Start drdds SHM bridge
#   ./deploy.sh bridge-stop [--host <hostname>]   # Stop bridge
#   ./deploy.sh status [--host <hostname>]        # Show status
#
# Options:
#   --host <hostname>   Access robot via Tailscale (e.g., m20-770-gogo)
#                       Without --host: uses AOS WiFi (10.21.41.1)

set -e

CMD="${1:-help}"
shift 2>/dev/null || true

REMOTE_HOST=""
positional=()
while [ $# -gt 0 ]; do
    case "$1" in
        --host) REMOTE_HOST="$2"; shift 2 ;;
        *) positional+=("$1"); shift ;;
    esac
done
set -- "${positional[@]}"

NOS_HOST="${1:-10.21.31.106}"
NOS_USER="${2:-user}"
AOS_INTERNAL="10.21.31.103"

if [ -n "${REMOTE_HOST}" ]; then
    JUMP_HOST="${NOS_USER}@${REMOTE_HOST}"
else
    JUMP_HOST="${JUMP_HOST:-user@10.21.41.1}"
fi

SSH_OPTS="-o ProxyJump=${JUMP_HOST}"

remote_ssh() {
    ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "$@"
}

remote_sudo() {
    if [ -n "${SUDO_PASS}" ]; then
        remote_ssh "echo '${SUDO_PASS}' | sudo -S $*" 2>&1 | grep -v '^\[sudo\]' || true
    else
        remote_ssh "sudo $*"
    fi
}

case "${CMD}" in
    setup)
        echo "=== Setting up NOS prerequisites ==="
        remote_sudo ip link set lo multicast on
        remote_sudo ip route add 224.0.0.0/4 dev lo 2>/dev/null || true
        remote_sudo ip route add 10.21.33.103/32 via 10.21.31.103 2>/dev/null || true
        remote_sudo mount --bind /var/opt/robot/data/nix /nix 2>/dev/null || true
        echo "=== Setup Complete ==="
        ;;

    bridge-start)
        echo "=== Starting drdds bridge ==="

        echo "  Stopping rsdriver for discovery ordering..."
        remote_sudo systemctl stop rsdriver 2>/dev/null || true
        sleep 2

        echo "  Cleaning stale SHM segments..."
        remote_sudo rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* /dev/shm/sem.fastrtps_* /dev/shm/drdds_bridge_* /dev/shm/sem.drdds_bridge_*

        echo "  Starting drdds_recv..."
        remote_sudo "nohup /opt/drdds_bridge/lib/drdds_bridge/drdds_recv > /var/log/drdds_recv.log 2>&1 &"
        sleep 3

        echo "  Restarting rsdriver (30s init delay)..."
        remote_sudo systemctl start rsdriver
        sleep 35

        echo "  Verifying bridge data flow..."
        remote_ssh "tail -3 /var/log/drdds_recv.log 2>/dev/null" || true

        echo "=== Bridge running ==="
        ;;

    bridge-stop)
        echo "Stopping drdds bridge..."
        remote_ssh "pkill -f drdds_recv 2>/dev/null" || true
        echo "Bridge stopped."
        ;;

    status)
        echo "=== drdds bridge ==="
        remote_ssh "pgrep -a drdds_recv 2>/dev/null" || echo "  NOT RUNNING"
        echo ""
        echo "=== dimos processes ==="
        remote_ssh "pgrep -a -f m20_smartnav 2>/dev/null" || echo "  smartnav not running"
        remote_ssh "pgrep -a nav_cmd_pub 2>/dev/null" || echo "  nav_cmd_pub not running"
        remote_ssh "pgrep -a drdds_lidar 2>/dev/null" || echo "  lidar bridge not running"
        remote_ssh "pgrep -a arise_slam 2>/dev/null" || echo "  arise_slam not running"
        echo ""
        echo "=== Ports ==="
        remote_ssh "ss -tlnp 2>/dev/null | grep -E '9877|7779|3030'" || echo "  no ports listening"
        echo ""
        echo "=== AOS services ==="
        remote_ssh "ssh user@${AOS_INTERNAL} 'for s in basic_server ctrlmcu rl_deploy rsdriver; do printf \"  %-15s %s\n\" \$s \$(systemctl is-active \$s); done'" 2>/dev/null || echo "  (could not reach AOS)"
        ;;

    *)
        echo "Usage: $0 <command> [--host <hostname>]"
        echo ""
        echo "Commands:"
        echo "  setup        - NOS prerequisites (multicast, nix mount, route)"
        echo "  bridge-start - Start drdds SHM bridge (with rsdriver ordering)"
        echo "  bridge-stop  - Stop drdds bridge"
        echo "  status       - Show bridge + service status"
        echo ""
        echo "Options:"
        echo "  --host <hostname>  Access robot via Tailscale (e.g., m20-770-gogo)"
        exit 1
        ;;
esac
