#!/usr/bin/env bash
# One-command WebRTC DataChannel e2e benchmark: gateway + load generator + servers/webrtc_data.py +
# a headless aiortc probe (browser stand-in) counting frames over a real unordered/lossy DataChannel.
# → `deno task bench:webrtc`.  (DUR is in seconds here — the probe measures a fixed window.)
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)" # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"     # dimos repo root
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

GW_PORT="${GW_PORT:-8090}"
WRTC_PORT="${WRTC_PORT:-8093}"
DUR="${DUR:-3}"
LOG="${TMPDIR:-/tmp}/dimoscope_webrtc"
mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

echo "[webrtc] starting gateway (:$GW_PORT) + load generator (LCM)…"
GATEWAY_PORT="$GW_PORT" "$DENO" run -A --unstable-sloppy-imports servers/gateway.ts >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ="${BENCH_HZ:-100}" PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 &
pids+=($!)

echo "[webrtc] warming up load generator (cold dimos import)…"
sleep 15
WEBRTC_PORT="$WRTC_PORT" GATEWAY_URL="ws://localhost:$GW_PORT" "$PY" servers/webrtc_data.py >"$LOG/webrtc.log" 2>&1 &
pids+=($!)
sleep 3 # webrtc_data subscribes to the gateway
WEBRTC_URL="ws://localhost:$WRTC_PORT" DUR="$DUR" "$PY" bench/webrtc_client_probe.py
