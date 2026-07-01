#!/usr/bin/env bash
# One-command decode-LOCATION benchmark: client-side binary decode (gateway = thin self-describing
# byte-relay, dimos today) vs server-side decode→JSON (the rosbridge model). Starts the gateway + load
# generator over LCM, runs bench/decode_bench.ts, tears everything down. → `deno task bench:decode`.
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)" # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"     # dimos repo root
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

GW_PORT="${GW_PORT:-8090}"
DUR="${DUR:-3000}"
LOG="${TMPDIR:-/tmp}/dimoscope_decode"
mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

echo "[decode] starting dimoscope service (:$GW_PORT) + load generator (LCM)…"
PORT="$GW_PORT" "$PY" -m gateway >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ="${BENCH_HZ:-100}" BENCH_GRID_HZ="${BENCH_GRID_HZ:-20}" \
  PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 &
pids+=($!)

sleep 2 # let the gateway bind; decode_bench.ts then warms up on its own (cold dimos import)
GATEWAY_URL="ws://localhost:$GW_PORT/ws" DUR="$DUR" "$DENO" run -A --unstable-sloppy-imports bench/decode_bench.ts
