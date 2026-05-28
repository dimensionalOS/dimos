#!/usr/bin/env bash
# Run a dimos blueprint and ALWAYS save the full terminal output to a
# timestamped log under /tmp/dimos-logs/. A `latest.log` symlink always
# points at the most recent run so you don't have to remember the
# timestamp when something goes wrong.
#
# Usage:
#   scripts/run-blueprint.sh                          # defaults to go2-marauders-map
#   scripts/run-blueprint.sh go2-marauders-map        # explicit blueprint
#   scripts/run-blueprint.sh yoloe-target-lock-distance-follow  # any other one
#
# Extra args after the blueprint name are forwarded to `dimos`, e.g.:
#   scripts/run-blueprint.sh go2-marauders-map --replay
#
# Defaults assume the real Go2 at 192.168.12.1 on dimair10. Override with:
#   SIM=mujoco scripts/run-blueprint.sh go2-marauders-map         # MuJoCo physics
#   SIM=dimsim scripts/run-blueprint.sh go2-marauders-map         # DimSim
#   REPLAY=1   scripts/run-blueprint.sh go2-marauders-map         # dataset replay (no physics)
#   ROBOT_IP=192.168.123.18 scripts/run-blueprint.sh go2-marauders-map
#
# Note: MuJoCo on macOS needs `mjpython` (not plain python) for the GUI to
# render. The script auto-uses .venv/bin/mjpython when SIM=mujoco AND that
# binary exists. If you see a black/frozen MuJoCo window, that's why.

set -uo pipefail

# Repo root = parent of this script's directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

BLUEPRINT="${1:-go2-marauders-map}"
shift || true  # consume the blueprint arg if it was passed
EXTRA_ARGS=("$@")

# Mode resolution priority: SIM > REPLAY > ROBOT_IP > default real-dog IP.
# We surface this as separate env vars rather than overloading ROBOT_IP so the
# log header is unambiguous about WHY this run isn't talking to real hardware.
SIM="${SIM:-}"
REPLAY="${REPLAY:-}"
ROBOT_IP="${ROBOT_IP:-192.168.12.1}"

MODE_ARGS=()
MODE_LABEL=""
if [ -n "$SIM" ]; then
    MODE_ARGS=(--simulation "$SIM")
    MODE_LABEL="simulation=$SIM"
elif [ -n "$REPLAY" ]; then
    MODE_ARGS=(--replay)
    MODE_LABEL="replay"
else
    MODE_ARGS=(--robot-ip "$ROBOT_IP")
    MODE_LABEL="robot_ip=$ROBOT_IP"
fi

LOG_DIR="/tmp/dimos-logs"
mkdir -p "$LOG_DIR"

# Timestamp like 20260528-013742 — sortable, no spaces, no colon.
TS="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="$LOG_DIR/${BLUEPRINT}-${TS}.log"

# Auto-rotate: keep newest 20, drop the rest. Quiet on first run.
ls -1t "$LOG_DIR"/*.log 2>/dev/null | tail -n +21 | xargs -I{} rm -f -- {} 2>/dev/null || true

# Pick interpreter: MuJoCo on macOS requires the special `mjpython` launcher
# (it owns the main thread for the Cocoa GL context). If we're in SIM=mujoco
# AND mjpython exists in the venv, dispatch via it; otherwise fall back to
# the normal `dimos` CLI script.
DIMOS_BIN=".venv/bin/dimos"
if [ "$SIM" = "mujoco" ] && [ -x ".venv/bin/mjpython" ]; then
    # Invoke the CLI's entry function under mjpython so the MuJoCo GL window
    # runs on the macOS main thread. `dimos` console-script targets
    # dimos.robot.cli.dimos:cli_main.
    DIMOS_CMD=(.venv/bin/mjpython -c
        "from dimos.robot.cli.dimos import cli_main; import sys; sys.exit(cli_main())"
        "${MODE_ARGS[@]}" --rerun-open native run "$BLUEPRINT")
else
    DIMOS_CMD=("$DIMOS_BIN" "${MODE_ARGS[@]}" --rerun-open native run "$BLUEPRINT")
fi
if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
    DIMOS_CMD+=("${EXTRA_ARGS[@]}")
fi

# Header in the log so post-hoc reading is easier.
{
    echo "=========================================="
    echo "  dimos blueprint runner"
    echo "  blueprint  : $BLUEPRINT"
    echo "  mode       : $MODE_LABEL"
    echo "  started    : $(date)"
    echo "  cwd        : $REPO_ROOT"
    echo "  git HEAD   : $(git rev-parse --short HEAD 2>/dev/null || echo n/a)"
    echo "  git status : $(git status --short 2>/dev/null | wc -l | tr -d ' ') uncommitted files"
    echo "  cmd        : ${DIMOS_CMD[*]}"
    echo "=========================================="
    echo ""
} | tee "$LOG_FILE"

echo "[runner] full log -> $LOG_FILE"
echo "[runner] follow live: tail -f $LOG_DIR/latest.log"
echo ""

# Use PATH so any subprocess that calls `dimos` / `python` finds the venv first.
export PATH="$REPO_ROOT/.venv/bin:$PATH"

# Run and tee. `script` would give us a PTY (preserves ANSI colors, progress
# bars), but tee is enough for plain-text postmortem.
"${DIMOS_CMD[@]}" 2>&1 | tee -a "$LOG_FILE"
RC=${PIPESTATUS[0]}

# Refresh the latest.log symlink only AFTER a successful tee so a crashed run
# still leaves its file on disk but `latest.log` doesn't point at empty data.
ln -sfn "$LOG_FILE" "$LOG_DIR/latest.log"

echo ""
echo "[runner] exit $RC — log: $LOG_FILE"
exit $RC
