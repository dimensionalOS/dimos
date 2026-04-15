#!/bin/bash
#
# M20 NOS deployment helper
#
# Sets up NOS prerequisites and manages the drdds SHM bridge.
# The nav stack runs natively via nix — no Docker needed.
#
# Usage:
#   ./deploy.sh setup [--host <hostname>]    # NOS prerequisites (multicast, nix, route)
#   ./deploy.sh bridge-start [--host <hostname>]  # Start drdds SHM bridge (lidar + IMU)
#   ./deploy.sh bridge-stop [--host <hostname>]   # Stop bridge
#   ./deploy.sh status [--host <hostname>]   # Show bridge + service status
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

SUDO_PASS="${SUDO_PASS:-"'"}"

case "${CMD}" in
    setup)
        echo "=== Setting up NOS prerequisites ==="
        remote_ssh "printf '%s\n' '${SUDO_PASS}' | sudo -S bash -c '
            ip link set lo multicast on
            ip route add 224.0.0.0/4 dev lo 2>/dev/null || true
            ip route add 10.21.33.103/32 via 10.21.31.103 2>/dev/null || true
            mount --bind /var/opt/robot/data/nix /nix 2>/dev/null || true
            echo done
        '"
        echo "=== Setup Complete ==="
        ;;

    bridge-start)
        echo "=== Starting drdds bridge ==="
        echo "Stopping rsdriver for discovery ordering..."
        remote_ssh "printf '%s\n' '${SUDO_PASS}' | sudo -S systemctl stop rsdriver 2>/dev/null"
        sleep 2

        echo "Cleaning stale SHM segments..."
        remote_ssh "printf '%s\n' '${SUDO_PASS}' | sudo -S rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* /dev/shm/sem.fastrtps_* /dev/shm/drdds_bridge_* /dev/shm/sem.drdds_bridge_*"

        echo "Starting drdds_recv..."
        remote_ssh "printf '%s\n' '${SUDO_PASS}' | sudo -S nohup /opt/drdds_bridge/lib/drdds_bridge/drdds_recv > /var/log/drdds_recv.log 2>&1 &"
        sleep 3

        echo "Restarting rsdriver (30s init delay)..."
        remote_ssh "printf '%s\n' '${SUDO_PASS}' | sudo -S systemctl start rsdriver"
        sleep 35

        echo "Verifying bridge data flow..."
        remote_ssh "tail -3 /var/log/drdds_recv.log 2>/dev/null || true"
        echo "=== Bridge running ==="
        ;;

    bridge-stop)
        echo "Stopping drdds bridge..."
        remote_ssh "pkill -f drdds_recv 2>/dev/null"
        echo "Bridge stopped."
        ;;

    status)
        echo "=== drdds bridge ==="
        remote_ssh "pgrep -a drdds_recv 2>/dev/null || echo 'NOT RUNNING'"
        echo ""
        echo "=== NOS processes ==="
        remote_ssh "pgrep -a -f m20_smartnav 2>/dev/null || echo 'smartnav not running'"
        remote_ssh "pgrep -a nav_cmd_pub 2>/dev/null || echo 'nav_cmd_pub not running'"
        echo ""
        echo "=== NOS ports ==="
        remote_ssh "ss -tlnp 2>/dev/null | grep -E '9877|7779|3030' || echo 'no ports listening'"
        echo ""
        echo "=== AOS services ==="
        remote_ssh "ssh user@${AOS_INTERNAL} 'systemctl is-active basic_server ctrlmcu rl_deploy rsdriver 2>/dev/null'" 2>/dev/null || true
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
        echo "  --host <hostname>  Access robot via Tailscale"
        exit 1
        ;;
esac
