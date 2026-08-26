#!/usr/bin/env bash
# Bake the Go2 motion host if it is stale, then install it onto a robot.
#
#   dimos/navigation/motion/deploy/deploy.sh go2
#
# `go2` is any ssh target (an alias from ~/.ssh/config, or root@<ip>). Runs from
# anywhere: paths resolve off this script, not the working directory.
#
# The binary carries its whole config, baked in from GO2_MOTION_HOST
# (dimos/robot/unitree/go2/zenoh/motion_host.py) -- so a config change means a
# rebuild, which is what the staleness check below is for.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

DEPLOYMENT="dimos.robot.unitree.go2.zenoh.motion_host:GO2_MOTION_HOST"
TARGET="aarch64-unknown-linux-gnu.2.31"
BINARY="$REPO_ROOT/motion-host"
SERVICE="$SCRIPT_DIR/dimos-motion-host.service"

REMOTE_DIR="/root/motion-host"
UNIT="dimos-motion-host"

no_build=""
host=""

usage() {
    cat >&2 <<'EOF'
Usage: deploy.sh [options] <ssh-host>

      --no-build   Install the binary that is already there; skip the bake.
  -h, --help       This.

Installs to /root/motion-host/ and enables dimos-motion-host on the robot.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h | --help) usage; exit 0 ;;
        --no-build) no_build=1; shift ;;
        -*) echo "deploy: unknown option $1" >&2; usage; exit 2 ;;
        *)
            [[ -n "$host" ]] && { echo "deploy: one host at a time (got '$host' and '$1')" >&2; exit 2; }
            host="$1"; shift ;;
    esac
done

[[ -n "$host" ]] || { echo "deploy: need an ssh host" >&2; usage; exit 2; }

# --- is the robot there? ------------------------------------------------------
#
# Before the bake, not after: a bake is minutes cold, and finding out the robot
# is off at the END of it helps nobody. ssh rather than ping, because ssh is what
# the install actually needs -- it catches a down host, a stale alias and a key
# problem in one probe.

echo "deploy: checking $host ..."
if ! ssh -o ConnectTimeout=8 -o BatchMode=yes "$host" true 2>/dev/null; then
    echo "deploy: $host is down or unreachable over ssh." >&2
    echo "        Powered on? On this LAN? The Go2's IP moves -- rediscover with:" >&2
    echo "        uv run python -m dimos.robot.unitree.go2.cli.landiscovery" >&2
    exit 1
fi

# --- bake ---------------------------------------------------------------------
#
# Unconditionally: `dimos bake` shells out to cargo, which does its own
# incremental staleness check, and re-emits the embedded config every run. A
# warm bake is ~25s. Any mtime heuristic here would just be a worse copy of what
# cargo already does correctly, and would go stale the first time someone adds an
# import this script does not know about.

if [[ -n "$no_build" ]]; then
    [[ -f "$BINARY" ]] || { echo "deploy: --no-build but no binary at $BINARY" >&2; exit 1; }
    echo "deploy: --no-build, installing the existing binary"
else
    echo "deploy: baking motion-host ..."
    (cd "$REPO_ROOT" && uv run dimos bake --deployment "$DEPLOYMENT" \
        -o "$BINARY" --builder zigbuild --target "$TARGET")
fi

# --- install ------------------------------------------------------------------

# Stop first: Linux refuses to write a running executable (ETXTBSY).
ssh "$host" "systemctl stop $UNIT 2>/dev/null || true; mkdir -p $REMOTE_DIR"

scp -q "$BINARY" "$host:$REMOTE_DIR/motion-host"
scp -q "$SERVICE" "$host:/etc/systemd/system/$UNIT.service"

ssh "$host" "
    set -e
    chmod +x $REMOTE_DIR/motion-host
    systemctl daemon-reload
    systemctl is-enabled --quiet $UNIT || systemctl enable $UNIT
    systemctl start $UNIT
"

# --- verify -------------------------------------------------------------------

sleep 5
if ! ssh "$host" "systemctl is-active --quiet $UNIT"; then
    echo "deploy: $UNIT did not come up. Last lines:" >&2
    ssh "$host" "journalctl -u $UNIT -n 30 --no-pager" >&2
    exit 1
fi

# Scoped to THIS start, not a time window: a restart minutes ago would otherwise
# have its four lines counted again.
loaded="$(ssh "$host" "
    inv=\$(systemctl show -p InvocationID --value $UNIT)
    journalctl _SYSTEMD_INVOCATION_ID=\"\$inv\" --no-pager | grep -c 'config loaded'
" 2>/dev/null || echo 0)"

echo "deploy: $UNIT active, $loaded/4 modules loaded config"
if [[ "$loaded" -ne 4 ]]; then
    echo "deploy: expected 4 -- check journalctl -u $UNIT" >&2
    exit 1
fi
