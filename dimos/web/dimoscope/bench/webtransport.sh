#!/usr/bin/env bash
# One-command WebTransport (HTTP/3 / QUIC) e2e: gateway + load + servers/webtransport.py + a headless
# aioquic probe. The load is pose (small → datagrams) + lidar (large → streams), so the probe exercises
# BOTH size-routed paths. → deno task bench:webtransport
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

GW_PORT="${GW_PORT:-8090}"
WT_PORT="${WT_PORT:-8093}"
DUR="${DUR:-3}"
LOG="${TMPDIR:-/tmp}/dimoscope_webtransport"
mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

echo "[webtransport] gateway + load (pose 100Hz → datagrams · lidar 200KB@10Hz → streams)…"
GATEWAY_PORT="$GW_PORT" "$DENO" run -A servers/gateway.ts >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ=100 BENCH_IMG_HZ=10 BENCH_IMG_BYTES=200000 \
  PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 &
pids+=($!)
echo "[webtransport] warming up load (cold dimos import)…"
sleep 15
WEBTRANSPORT_PORT="$WT_PORT" GATEWAY_URL="ws://localhost:$GW_PORT" "$PY" servers/webtransport.py >"$LOG/wt.log" 2>&1 &
pids+=($!)
sleep 3
WT_PORT="$WT_PORT" DUR="$DUR" "$PY" bench/webtransport_client_probe.py
rc=$?
echo "--- webtransport.py log ---"; tail -3 "$LOG/wt.log"
exit $rc
