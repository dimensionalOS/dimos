#!/usr/bin/env bash
# Data-path matrix: measure every delivery mechanism (WebSocket · SSE · HTTP long-poll)
# through the netsim impairment proxy across a set of network profiles, over the LCM
# gateway + bench_source load generator. One command → bench/RESULTS-mechanisms.md.
#
#   bench/matrix.sh                          # default profiles
#   PROFILES="lan 3g 2g" DUR=4000 bench/matrix.sh
#   BENCH_IMG_HZ=10 BENCH_IMG_BYTES=1000000 bench/matrix.sh   # add a 1MB image stream
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)" # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"     # dimos repo root
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

PROFILES="${PROFILES:-lan wifi 4g 3g 2g}"
GW_PORT="${GW_PORT:-8090}"
PROXY_PORT="${PROXY_PORT:-8099}"
DUR="${DUR:-3000}"
LOG="${TMPDIR:-/tmp}/dimoscope_matrix"
mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

echo "[matrix] starting gateway (:$GW_PORT) + bench_source (LCM)…"
GATEWAY_PORT="$GW_PORT" "$DENO" run -A servers/gateway.ts >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ="${BENCH_HZ:-100}" BENCH_GRID_HZ="${BENCH_GRID_HZ:-20}" \
  BENCH_IMG_HZ="${BENCH_IMG_HZ:-0}" BENCH_IMG_BYTES="${BENCH_IMG_BYTES:-1000000}" \
  PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 &
pids+=($!)

echo "[matrix] warming up publisher (cold dimos import)…"
sleep 18

rows=""
for prof in $PROFILES; do
  echo "[matrix] profile=$prof"
  NETSIM_PROFILE="$prof" NETSIM_LISTEN="$PROXY_PORT" NETSIM_TARGET="localhost:$GW_PORT" \
    "$DENO" run -A bench/netsim.ts >"$LOG/proxy.log" 2>&1 &
  proxy=$!
  sleep 0.6
  md="$(GATEWAY_URL="ws://localhost:$PROXY_PORT" PROFILE="$prof" DUR="$DUR" WARMUP_MS=8000 \
        "$DENO" run -A bench/matrix_run.ts 2>>"$LOG/run.log")"
  kill "$proxy" 2>/dev/null || true
  rows+="$md"$'\n'
  sleep 0.5
done

stamp="$(date '+%Y-%m-%d %H:%M')"
out="bench/RESULTS-mechanisms.md"
{
  echo "# dimos data-path benchmark — delivery mechanisms × network conditions"
  echo
  echo "_Generated $stamp · LCM gateway byte-relay · 4×PoseStamped @ ${BENCH_HZ:-100}Hz · ${DUR}ms/run · end-to-end (publish→recv) · netsim TCP proxy._"
  echo
  echo "Latency is one-way ms (publisher and client share a clock). \`loss%\` is wire drop from seq gaps."
  echo
  echo "| profile | mechanism | hz | kB/s | p50 | p95 | p99 | max | std | loss% |"
  echo "|---|---|--:|--:|--:|--:|--:|--:|--:|--:|"
  printf "%s" "$rows"
  echo
  echo "Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps."
} >"$out"
echo "[matrix] → wrote $out"
cat "$out"
