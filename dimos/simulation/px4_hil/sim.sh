#!/usr/bin/env bash
# DimOS mixed-fleet simulator: MuJoCo physics + PX4 SITL + the DimOS stack.
#
#   ./sim.sh start [DRONES] [DOGS] [--viewer]
#                                    bring everything up   (default 3 1)
#                                    --viewer opens the MuJoCo window
#   ./sim.sh stop                    tear everything down
#   ./sim.sh status                  what is running
#   ./sim.sh log [bridge|daemon|px4N]
#
# Order matters and is handled here: the MuJoCo bridge must be listening before
# PX4 dials in on 4560+i, PX4 must be up before its simulated battery can be
# tuned, and the DimOS daemon must come last so every vehicle is already there.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
VENV="$REPO/.venv/bin"
LOGS="${SIM_LOG_DIR:-/tmp/dimos-sim}"
# Match the bridge by its module basename only. A pattern like
# 'dimos.simulation.px4_hil' looks safe but `.` is a regex wildcard, so it also
# matches this script's own path (dimos/simulation/px4_hil/sim.sh) and the
# teardown kills itself before it can start anything.
PATTERN='fleet_bridge'

mkdir -p "$LOGS"

_stop() {
  "$VENV/dimos" stop >/dev/null 2>&1 || true
  for pid in $(pgrep -f "$PATTERN" 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    kill -9 "$pid" 2>/dev/null || true
  done
  for pid in $(pgrep -x px4 2>/dev/null); do kill -9 "$pid" 2>/dev/null || true; done
  rm -f /tmp/px4-sock-* 2>/dev/null || true
  sleep 2
  echo "stopped."
}

_status() {
  echo "px4 instances : $(pgrep -xc px4 2>/dev/null || echo 0)"
  echo "mujoco bridge : $(pgrep -fc "$PATTERN" 2>/dev/null || echo 0)"
  "$VENV/dimos" status 2>&1 | head -4
  echo "governor      : $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo '?')"
  last=$(grep -ao 'sim=[ 0-9.]*s rtf=[ 0-9.]*x' "$LOGS/bridge.log" 2>/dev/null | tail -1)
  echo "physics       : ${last:-no rtf line yet (starting, or bridge down)}"
}

_start() {
  local drones="${1:-3}" dogs="${2:-1}" viewer="${3:-}"
  case "$drones$dogs" in
    *[!0-9]*) echo "usage: start [DRONES] [DOGS] [--viewer] -- counts must be integers, got '$drones' '$dogs'" >&2; exit 1 ;;
  esac
  if [ "$drones" -eq 0 ] && [ "$dogs" -eq 0 ]; then
    echo "nothing to simulate: pass at least one drone or dog" >&2; exit 1
  fi
  if [ "$drones" -gt 5 ] || [ "$dogs" -gt 5 ]; then
    echo "refusing $drones drones / $dogs dogs: the operating envelope is 5 drones + 5 dogs." >&2
    echo "Everything past that is unvalidated. Edit sim.sh if the envelope changes." >&2; exit 1
  fi
  gov=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo '?')
  if [ "$gov" = "powersave" ]; then
    echo "WARNING: CPU governor is 'powersave' (~1.5 GHz of a 4.7 GHz machine)." >&2
    echo "         This costs ~2.4x realtime. Fix: sudo cpupower frequency-set -g performance" >&2
  fi
  [ -x "$PX4_BUILD/bin/px4" ] || { echo "PX4 not built at $PX4_BUILD" >&2; exit 1; }

  _stop >/dev/null
  if [ "$drones" -gt 0 ]; then
    for i in $(seq 0 $((drones - 1))); do
      port=$((4560 + i))
      if ss -ltn 2>/dev/null | grep -q ":$port "; then
        echo "port $port is still in use after teardown:" >&2
        ss -ltnp 2>/dev/null | grep ":$port " >&2 || true
        echo "kill that process, then retry." >&2
        exit 1
      fi
    done
  fi
  rm -f "$LOGS"/*.log

  if [ -n "$viewer" ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "      (--viewer requested but DISPLAY and WAYLAND_DISPLAY are both unset;" >&2
    echo "       no window can open. Run from a desktop session.)" >&2
  fi
  echo "[1/4] MuJoCo world: $drones drone(s), $dogs dog(s)${viewer:+ (viewer)}"
  ( cd "$REPO" && setsid "$VENV/python" -m dimos.simulation.px4_hil.fleet_bridge \
      --drones "$drones" --dogs "$dogs" $viewer >"$LOGS/bridge.log" 2>&1 </dev/null & )
  sleep 4
  # The bridge dying here used to be invisible: PX4 and the daemon would still
  # start, every module would register, and this script would print "ready" over
  # a world with no physics in it. Catch it at the source.
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "" >&2
    echo "MuJoCo bridge failed to start. Last lines of $LOGS/bridge.log:" >&2
    tail -20 "$LOGS/bridge.log" >&2
    _stop >/dev/null
    return 1
  fi

  if [ "$drones" -gt 0 ]; then
    echo "[2/4] PX4 SITL x$drones"
    for i in $(seq 0 $((drones - 1))); do
      # Start from a clean parameter store. A stale or corrupt parameters.bson
      # produces bizarre estimator behaviour that looks like a swarm bug.
      rm -f "$PX4_BUILD/rootfs/$i"/parameters*.bson 2>/dev/null || true
      ( cd "$PX4_BUILD" && PX4_SIM_MODEL=none_iris setsid ./bin/px4 -i "$i" -d \
          >"$LOGS/px4_$i.log" 2>&1 </dev/null & )
      sleep 2
    done
    sleep 12
    echo "[3/4] tuning simulated battery (default drains full->empty in 60 s)"
    if ! ( cd "$REPO" && "$VENV/python" \
        dimos/simulation/px4_hil/sim_params.py --count "$drones" ) \
        >"$LOGS/params.log" 2>&1; then
      echo "      first attempt failed, retrying once..."
      if ! ( cd "$REPO" && "$VENV/python" \
          dimos/simulation/px4_hil/sim_params.py --count "$drones" ) \
          >>"$LOGS/params.log" 2>&1; then
        echo "sim parameter tuning FAILED twice -- without it batteries drain in 60 s" >&2
        echo "and the geofence is not armed. See $LOGS/params.log" >&2
        _stop >/dev/null   # do not leave a half-started stack behind
        exit 1
      fi
    fi
  else
    echo "[2/4] no drones requested, skipping PX4"
    echo "[3/4] no battery to tune"
  fi

  # SIM_BLUEPRINT=mixed-fleet-agentic adds the natural-language agent on top
  # of the same fleet (needs OPENAI_API_KEY exported in this shell; drive it
  # with `dimos humancli`). Default is the plain MCP blueprint.
  blueprint="${SIM_BLUEPRINT:-mixed-fleet-mcp}"
  echo "[4/4] DimOS daemon ($blueprint, SIM_DRONES=$drones SIM_DOGS=$dogs)"
  if [ "$blueprint" != "mixed-fleet-mcp" ] && [ -z "${OPENAI_API_KEY:-}" ]; then
    echo "WARNING: OPENAI_API_KEY is not set -- the agent in $blueprint will not work." >&2
  fi
  ( cd "$REPO" && CI=1 DIMOS_TRANSPORT=zenoh SIM_DRONES="$drones" SIM_DOGS="$dogs" \
      setsid "$VENV/dimos" run "$blueprint" >"$LOGS/daemon.log" 2>&1 </dev/null & )

  # Wait until every robot has actually reported to the coordinator, rather
  # than printing "ready" and letting the first command fail. Each probe is
  # bounded: while the daemon is still coming up an mcp call can block for a
  # long time, which would otherwise eat the whole budget in two attempts.
  local want=$((drones + dogs)) tries=0
  printf '      waiting for %d robot(s) ' "$want"
  while [ $tries -lt 30 ]; do
    tries=$((tries + 1))
    # A module registering is not the same as a robot existing. Require
    # connected=True: a crashed bridge still leaves every module registered but
    # reporting connected=False, which is exactly how a dead simulator used to
    # sail through this check.
    got=$(timeout 10 "$VENV/dimos" mcp call list_drones 2>/dev/null \
            | grep -cE "^  (drone|dog)[0-9]+ .*connected=True" || true)
    if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
      printf '\n'
      echo "MuJoCo bridge died during startup. Last lines of $LOGS/bridge.log:" >&2
      tail -20 "$LOGS/bridge.log" >&2
      _stop >/dev/null
      return 1
    fi
    if [ "${got:-0}" -ge "$want" ]; then
      # Final gate: the physics clock must be moving. Everything above can pass
      # while the world is frozen.
      # Sample either side of a full status-log period. The bridge prints
      # "sim=..." every 5 s, so a 2 s window reads the SAME line twice and
      # declares a perfectly healthy world frozen.
      local t0 t1
      t0=$(grep -ao 'sim=[ 0-9.]*' "$LOGS/bridge.log" | tail -1 | tr -dc '0-9.')
      sleep 7
      t1=$(grep -ao 'sim=[ 0-9.]*' "$LOGS/bridge.log" | tail -1 | tr -dc '0-9.')
      if [ -n "$t0" ] && [ -n "$t1" ] && [ "$t0" = "$t1" ]; then
        printf '\n'
        echo "robots are connected but the sim clock is not advancing" >&2
        echo "(stuck at sim=${t0}s). Check $LOGS/bridge.log" >&2
        _stop >/dev/null
        return 1
      fi
      printf ' ok\n\n'
      timeout 20 "$VENV/dimos" mcp call list_drones 2>/dev/null
      printf '\nready. logs in %s\n' "$LOGS"
      return 0
    fi
    printf '.'
    sleep 3
  done
  printf '\n'
  echo "timed out waiting for $want robot(s); check $LOGS/daemon.log" >&2
  echo "(stack left RUNNING for diagnosis -- sim.sh stop when done)" >&2
  return 1
}

case "${1:-}" in
  start)
    shift
    _viewer=""; _drones=""; _dogs=""
    for _a in "$@"; do
      if   [ "$_a" = "--viewer" ]; then _viewer="--viewer"
      elif [ -z "$_drones" ];      then _drones="$_a"
      elif [ -z "$_dogs" ];        then _dogs="$_a"
      fi
    done
    _start "${_drones:-3}" "${_dogs:-1}" "$_viewer" ;;
  stop)   _stop ;;
  status) _status ;;
  log)    tail -f "$LOGS/${2:-bridge}.log" ;;
  *) echo "usage: $0 {start [DRONES] [DOGS] [--viewer]|stop|status|log [bridge|daemon|px4N]}" >&2; exit 1 ;;
esac
