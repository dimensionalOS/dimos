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
STREAM="${STREAM:-pose}"
# Stream profile → bench_source rates/sizes. lidar≈2MB/s · dense≈20MB/s · camera≈11MB/s · mixed=all.
case "$STREAM" in
  lidar) S_HZ=0; S_GRID=0; S_IMGHZ=10; S_IMGB=200000 ;;
  dense) S_HZ=0; S_GRID=0; S_IMGHZ=20; S_IMGB=1000000 ;;
  camera) S_HZ=0; S_GRID=0; S_IMGHZ=20; S_IMGB=550000 ;;
  mixed) S_HZ=100; S_GRID=20; S_IMGHZ=10; S_IMGB=200000 ;;
  *) S_HZ=100; S_GRID=20; S_IMGHZ=0; S_IMGB=0 ;;
esac
LOG="${TMPDIR:-/tmp}/dimoscope_matrix"
mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

echo "[matrix] starting dimoscope service (:$GW_PORT) + bench_source (LCM)…"
PORT="$GW_PORT" "$PY" -m gateway >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ="$S_HZ" BENCH_GRID_HZ="$S_GRID" \
  BENCH_IMG_HZ="$S_IMGHZ" BENCH_IMG_BYTES="$S_IMGB" \
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
  md="$(GATEWAY_URL="ws://localhost:$PROXY_PORT" PROFILE="$prof" STREAM="$STREAM" DUR="$DUR" WARMUP_MS=8000 \
        "$DENO" run -A bench/matrix_run.ts 2>>"$LOG/run.log")"
  kill "$proxy" 2>/dev/null || true
  rows+="$md"$'\n'
  sleep 0.5
done

stamp="$(date '+%Y-%m-%d %H:%M')"
out="bench/RESULTS-mechanisms.md"
[ "$STREAM" = "pose" ] || out="bench/RESULTS-mechanisms-$STREAM.md"
{
  echo "# dimos data-path benchmark — delivery mechanisms × network conditions"
  echo
  echo "_Generated $stamp · stream=$STREAM · LCM gateway byte-relay · ${DUR}ms/run · end-to-end (publish→recv) · netsim TCP proxy._"
  echo
  echo "Latency is one-way ms (publisher and client share a clock). \`loss%\` is wire drop from seq gaps."
  echo
  echo "| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |"
  echo "|---|---|---|--:|--:|--:|--:|--:|--:|"
  printf "%s" "$rows"
  echo
  echo "Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps."
} >"$out"
echo "[matrix] → wrote $out"
cat "$out"
