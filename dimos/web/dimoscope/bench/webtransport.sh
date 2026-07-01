#!/usr/bin/env bash
# One-command WebTransport (HTTP/3 / QUIC) e2e: the dimoscope service (QUIC) + load + a headless
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

echo "[webtransport] dimoscope service (QUIC :$WT_PORT) + load (pose 100Hz → datagrams · lidar 200KB@10Hz → streams)…"
PORT="$GW_PORT" WT_PORT="$WT_PORT" "$PY" -m gateway >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ=100 BENCH_IMG_HZ=10 BENCH_IMG_BYTES=200000 \
  PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 &
pids+=($!)
echo "[webtransport] warming up service + load (cold dimos import)…"
sleep 15
WT_PORT="$WT_PORT" DUR="$DUR" "$PY" bench/webtransport_client_probe.py
rc=$?
echo "--- service log ---"; tail -3 "$LOG/gw.log"
exit $rc
