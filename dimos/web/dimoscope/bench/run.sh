#!/usr/bin/env bash
# Reproducible transport benchmark: start the relevant server(s) + bench publisher, run
# the headless bench, tear everything down. Modes:
#   bench/run.sh            # dimoscope service, LCM source (:8090)
#   bench/run.sh zenoh      # dimoscope service, Zenoh source (:8090)
#   bench/run.sh all        # both, sequentially → combined bench/RESULTS.md
# Everything runs headless: the dimoscope service (the gateway) + bench_publisher on the chosen
# bus; the client rows go through bench/bench.ts. `all` measures end-to-end latency (BENCH_E2E).
set -uo pipefail
MODE="${1:-lcm}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"        # dimos repo root
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

# run_one <lcm|zenoh|ts> — start its server(s) + publisher, run the bench, tear down.
run_one() {
  local mode="$1" port label start=${#pids[@]} rc
  local stamp; stamp="$(date '+%Y-%m-%d %H:%M')"
  local bus
  port="${GATEWAY_PORT:-8090}"
  if [ "$mode" = zenoh ]; then bus=zenoh; label="dimoscope (Zenoh source)"; else bus=lcm; label="dimoscope (LCM source)"; fi
  PORT="$port" "$PY" -m gateway & pids+=($!)
  DIMOS_TRANSPORT="$bus" BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
  sleep 4
  GATEWAY_URL="ws://localhost:$port/ws" BENCH_LABEL="$label" BENCH_E2E="${BENCH_E2E:-}" \
    BENCH_DUR_MS="${BENCH_DUR_MS:-4000}" BENCH_STAMP="$stamp" "$DENO" run -A bench/bench.ts
  rc=$?
  # tear down just this run's procs so the next mode's ports/bus are free
  for ((i = start; i < ${#pids[@]}; i++)); do kill "${pids[$i]}" 2>/dev/null || true; done
  sleep 1
  return $rc
}

if [ "$MODE" = all ]; then
  stamp="$(date '+%Y-%m-%d %H:%M')"
  BENCH_E2E=1 run_one lcm;   lcm_md="$(cat bench/last_run.md)"
  BENCH_E2E=1 run_one zenoh; zenoh_md="$(cat bench/last_run.md)"
  {
    echo "# dimoscope transport benchmark — LCM vs Zenoh source (one service)"
    echo
    echo "_Generated $stamp · headless · **end-to-end latency** (publish→client) · synthetic \`bench_publisher.py\` source (no sim) · single dimoscope service ingesting both buses._"
    echo
    echo "$lcm_md"; echo; echo "---"; echo
    echo "$zenoh_md"; echo
    echo "> Both measured end-to-end (publish→client) against the one dimoscope service, which ingests LCM and Zenoh at once."
  } > bench/RESULTS.md
  echo "→ wrote combined bench/RESULTS.md (LCM + Zenoh source)"
else
  run_one "$MODE"
fi
