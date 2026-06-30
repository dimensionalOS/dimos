#!/usr/bin/env bash
# bench:qos — the client QoS knobs over a mixed load (pose 100Hz + grid + lidar 2MB/s):
#   phase 1 (direct gateway): rate-limit + on-demand — clean gateway-side QoS numbers.
#   phase 2 (through a 4G netsim link): prioritization — rate-limit lidar so pose survives saturation.
# → deno task bench:qos
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

GW_PORT="${GW_PORT:-8090}"
PROXY_PORT="${PROXY_PORT:-8099}"
DUR="${DUR:-3000}"
LOG="${TMPDIR:-/tmp}/dimoscope_qos"
mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

echo "[qos] dimoscope service + bench_source (mixed: pose 100Hz + grid + lidar 200KB@10Hz)…"
PORT="$GW_PORT" "$PY" serve.py >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ=100 BENCH_GRID_HZ=20 BENCH_IMG_HZ=10 BENCH_IMG_BYTES=200000 \
  PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 &
pids+=($!)
sleep 16

echo
GATEWAY_URL="ws://localhost:$GW_PORT/ws" DUR="$DUR" MODE=basic "$DENO" run -A bench/qos_run.ts

echo "[qos] phase 2 — through a 4G netsim link…"
NETSIM_PROFILE=4g NETSIM_LISTEN="$PROXY_PORT" NETSIM_TARGET="localhost:$GW_PORT" \
  "$DENO" run -A bench/netsim.ts >"$LOG/proxy.log" 2>&1 &
pids+=($!)
sleep 0.6
echo
GATEWAY_URL="ws://localhost:$PROXY_PORT/ws" DUR="$DUR" MODE=prio "$DENO" run -A bench/qos_run.ts
