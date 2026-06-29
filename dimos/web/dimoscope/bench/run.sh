#!/usr/bin/env bash
# Reproducible transport benchmark: start a gateway + bench publisher, run the
# headless bench, tear everything down. Modes:
#   bench/run.sh            # Bun↔LCM gateway (:8090)
#   bench/run.sh zenoh      # Python↔Zenoh gateway (:8091)
#   bench/run.sh all        # both, sequentially → combined bench/RESULTS.md
# (zenoh-ts direct is browser-only — no Bun/Node client — so it's benched in-app.)
set -uo pipefail
MODE="${1:-lcm}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"        # dimos repo root
PY="$REPO/.venv/bin/python"
cd "$HERE"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

# run_one <lcm|zenoh> — start its gateway + publisher, run bench.ts, tear down.
run_one() {
  local mode="$1" port label start=${#pids[@]} rc
  if [ "$mode" = zenoh ]; then
    port="${GATEWAY_PORT:-8091}"; label="Python↔Zenoh gateway"
    DIMOS_TRANSPORT=zenoh GATEWAY_PORT="$port" "$PY" servers/gateway_zenoh.py & pids+=($!)
    DIMOS_TRANSPORT=zenoh BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
  else
    port="${GATEWAY_PORT:-8090}"; label="Bun↔LCM gateway"
    GATEWAY_PORT="$port" bun run servers/gateway.ts & pids+=($!)
    DIMOS_TRANSPORT=lcm BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
  fi
  sleep 2.5
  GATEWAY_URL="ws://localhost:$port" BENCH_LABEL="$label" \
    BENCH_DUR_MS="${BENCH_DUR_MS:-4000}" BENCH_STAMP="$(date '+%Y-%m-%d %H:%M')" \
    bun run bench/bench.ts
  rc=$?
  # tear down just this run's procs so the next mode's ports/bus are free
  for ((i = start; i < ${#pids[@]}; i++)); do kill "${pids[$i]}" 2>/dev/null || true; done
  sleep 1
  return $rc
}

if [ "$MODE" = all ]; then
  stamp="$(date '+%Y-%m-%d %H:%M')"
  run_one lcm;   lcm_md="$(cat bench/last_run.md)"
  run_one zenoh; zenoh_md="$(cat bench/last_run.md)"
  {
    echo "# dimoscope transport benchmark — all transports"
    echo
    echo "_Generated $stamp · headless \`@dimos/topics\` SDK vs each gateway · synthetic \`bench_publisher.py\` source (no sim)._"
    echo
    echo "$lcm_md"
    echo
    echo "---"
    echo
    echo "$zenoh_md"
    echo
    echo "> **zenoh-ts (direct)** is browser-only (its client has no Bun/Node target), so it can't run in this headless harness — bench it via the in-app **Stats** latency on the \`zenoh-ts (direct)\` dropdown option."
  } > bench/RESULTS.md
  echo "→ wrote combined bench/RESULTS.md"
else
  run_one "$MODE"
fi
