#!/usr/bin/env bash
# QoS under stress — the A/B that proves the data-path prioritization is good tech. A sim robot publishes
# pose (light, high-priority) + lidar/img (heavy, low-priority) onto the bus; the single service (the gateway)
# fans it to a browser SDK client over a BANDWIDTH-CAPPED netsim link the heavy stream saturates. We run
# the SAME load with the gateway scheduler OFF (FIFO — today's baseline) and ON (the per-client priority
# outbox). ON, the light topic stays crisp while lidar conflates; OFF, the light topic starves.
#   SIM=synthetic|replay|mujoco|dimsim  → deno task demo:qos
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

SIM="${SIM:-synthetic}"
LIGHT="${LIGHT_TOPIC:-/bench/p0}"
HEAVY="${HEAVY_TOPIC:-/bench/img}"
DUR="${DUR_MS:-4000}"
BW="${NETSIM_BW_KBPS:-4000}" # link cap (kbps) — below the heavy stream so it saturates the pipe
# Heavy stream in MANY SMALL frames (default 4KB @ 500Hz ≈ 2 MB/s): a single big frame would head-of-line-
# block the writer regardless of priority, so the heavy lane is fragmented (as a real bulk stream would be).
HBYTES="${HEAVY_BYTES:-4000}"
HHZ="${HEAVY_HZ:-500}"
LOG="${TMPDIR:-/tmp}/dimoscope_qos_demo"; mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done
  for port in 8080 8443 8099; do lsof -ti :$port 2>/dev/null | xargs -r kill -9 2>/dev/null; done; }
trap cleanup EXIT
for port in 8080 8443 8099; do lsof -ti :$port 2>/dev/null | xargs -r kill -9 2>/dev/null; done

echo "[qos-demo] load: SIM=$SIM  light=$LIGHT  heavy=$HEAVY  link=${BW}kbps"
case "$SIM" in
  synthetic) DIMOS_TRANSPORT=lcm BENCH_HZ=100 BENCH_GRID_HZ=0 BENCH_IMG_HZ="$HHZ" BENCH_IMG_BYTES="$HBYTES" \
      PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 & ;;
  replay) DIMOS_TRANSPORT=zenoh uv run dimos --replay run unitree-go2 >"$LOG/pub.log" 2>&1 & ;;
  mujoco) DIMOS_TRANSPORT=zenoh uv run dimos --simulation mujoco run unitree-go2 >"$LOG/pub.log" 2>&1 & ;;
  dimsim) DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run unitree-go2 >"$LOG/pub.log" 2>&1 & ;;
  *) echo "unknown SIM=$SIM"; exit 2 ;;
esac
pids+=($!)
echo "[qos-demo] warming up load (cold dimos import)…"; sleep 16

run_mode() { # $1 label  [$2 LIGHT_LANE  $3 HEAVY_LANE] (client-declared overrides)
  for port in 8080 8443; do lsof -ti :$port 2>/dev/null | xargs -r kill -9 2>/dev/null; done
  # EGRESS_KBPS paces the gateway's writer to the client's link budget → the backlog (and so the priority
  # decision) stays in the gateway outbox instead of bloating kernel/proxy buffers (which would FIFO it).
  EGRESS_KBPS="$BW" PORT=8080 "$PY" -m gateway >"$LOG/serve_$1.log" 2>&1 &
  local sp=$!; pids+=("$sp")
  until curl -s --max-time 1 http://localhost:8080/health >/dev/null 2>&1; do sleep 0.5; done
  sleep 2 # let it re-tap the bus + discover topics
  GW_URL="ws://localhost:8080/ws" LIGHT_TOPIC="$LIGHT" HEAVY_TOPIC="$HEAVY" DUR_MS="$DUR" MODE="$1" \
    LIGHT_LANE="${2:-}" HEAVY_LANE="${3:-}" \
    "$DENO" run -A bench/qos_demo_run.ts || true
  kill "$sp" 2>/dev/null || true; sleep 1
}

echo
echo "=== QoS A/B — sim → the gateway (egress-paced ${BW}kbps) → SDK · light vs heavy under contention ==="
# The priority outbox is now unconditional (no QOS_SCHED toggle); the OFF/FIFO baseline is preserved in
# the committed demo GIF (ba36735e7). This live run shows server-default vs client-override.
run_mode ON-default
run_mode ON-invert bulk command # CLIENT declares pose=bulk(low) + lidar=command(critical) over the wire
echo
echo "ON-default = priority outbox, server default (pose/teleop wins, lidar conflates)"
echo "ON-invert = the CLIENT overrides per topic over the wire (pose→bulk/low, lidar→command/critical), so"
echo "the heavy stream now wins and pose starves — the OPPOSITE of the default. Proves client override works."
