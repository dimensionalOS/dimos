#!/usr/bin/env bash
# Reproducible transport benchmark: start a gateway + bench publisher, run the
# headless bench, tear everything down.
#   bench/run.sh            # Bun↔LCM gateway (:8090)
#   bench/run.sh zenoh      # Python↔Zenoh gateway (:8091)
set -uo pipefail
MODE="${1:-lcm}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"        # dimos repo root
PY="$REPO/.venv/bin/python"
PORT="${GATEWAY_PORT:-$([ "$MODE" = zenoh ] && echo 8091 || echo 8090)}"
pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT
cd "$HERE"

if [ "$MODE" = zenoh ]; then
  DIMOS_TRANSPORT=zenoh GATEWAY_PORT="$PORT" "$PY" servers/gateway_zenoh.py & pids+=($!)
  LABEL="Python↔Zenoh gateway"
  DIMOS_TRANSPORT=zenoh BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
else
  GATEWAY_PORT="$PORT" bun run servers/gateway.ts & pids+=($!)
  LABEL="Bun↔LCM gateway"
  DIMOS_TRANSPORT=lcm BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
fi

sleep 2.5
GATEWAY_URL="ws://localhost:$PORT" BENCH_LABEL="$LABEL" \
  BENCH_DUR_MS="${BENCH_DUR_MS:-4000}" BENCH_STAMP="$(date '+%Y-%m-%d %H:%M')" \
  bun run bench/bench.ts
