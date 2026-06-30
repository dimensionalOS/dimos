#!/usr/bin/env bash
# Reproducible transport benchmark: start the relevant server(s) + bench publisher, run
# the headless bench, tear everything down. Modes:
#   bench/run.sh            # Deno↔LCM gateway (:8090)
#   bench/run.sh zenoh      # Python↔Zenoh gateway (:8091)
#   bench/run.sh ts         # zenoh-ts direct via the :10000 bridge
#   bench/run.sh all        # all three, sequentially → combined bench/RESULTS.md
# Everything runs headless under Deno. The gateway rows go through bench/bench.ts; the
# zenoh-ts direct row has its own runner (bench/bench_deno.ts). `all` measures end-to-end
# latency (BENCH_E2E) so every transport — including the gateway-less zenoh-ts — is
# compared the same way.
set -uo pipefail
MODE="${1:-lcm}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"        # dimos repo root
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
BRIDGE="$(command -v zenoh-bridge-remote-api || echo "$HOME/.cargo/bin/zenoh-bridge-remote-api")"
cd "$HERE"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

# run_one <lcm|zenoh|ts> — start its server(s) + publisher, run the bench, tear down.
run_one() {
  local mode="$1" port label start=${#pids[@]} rc
  local stamp; stamp="$(date '+%Y-%m-%d %H:%M')"
  if [ "$mode" = ts ]; then
    "$BRIDGE" --ws-port 10000 >/dev/null 2>&1 & pids+=($!)
    DIMOS_TRANSPORT=zenoh BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
    sleep 3
    BENCH_DUR_MS="${BENCH_DUR_MS:-4000}" BENCH_STAMP="$stamp" \
      "$DENO" run -A --node-modules-dir bench/bench_deno.ts
    rc=$?
  else
    if [ "$mode" = zenoh ]; then
      port="${GATEWAY_PORT:-8091}"; label="Python↔Zenoh gateway"
      DIMOS_TRANSPORT=zenoh GATEWAY_PORT="$port" "$PY" servers/gateway_zenoh.py & pids+=($!)
      DIMOS_TRANSPORT=zenoh BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
    else
      port="${GATEWAY_PORT:-8090}"; label="Deno↔LCM gateway"
      GATEWAY_PORT="$port" "$DENO" run -A servers/gateway.ts & pids+=($!)
      DIMOS_TRANSPORT=lcm BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
    fi
    sleep 2.5
    GATEWAY_URL="ws://localhost:$port" BENCH_LABEL="$label" BENCH_E2E="${BENCH_E2E:-}" \
      BENCH_DUR_MS="${BENCH_DUR_MS:-4000}" BENCH_STAMP="$stamp" "$DENO" run -A bench/bench.ts
    rc=$?
  fi
  # tear down just this run's procs so the next mode's ports/bus are free
  for ((i = start; i < ${#pids[@]}; i++)); do kill "${pids[$i]}" 2>/dev/null || true; done
  sleep 1
  return $rc
}

if [ "$MODE" = all ]; then
  stamp="$(date '+%Y-%m-%d %H:%M')"
  BENCH_E2E=1 run_one lcm;   lcm_md="$(cat bench/last_run.md)"
  BENCH_E2E=1 run_one zenoh; zenoh_md="$(cat bench/last_run.md)"
  run_one ts;                ts_md="$(cat bench/last_run.md)"   # bench_deno.ts is always end-to-end
  {
    echo "# dimoscope transport benchmark — all transports"
    echo
    echo "_Generated $stamp · headless · **end-to-end latency** (publish→client) · synthetic \`bench_publisher.py\` source (no sim)._"
    echo
    echo "$lcm_md"; echo; echo "---"; echo
    echo "$zenoh_md"; echo; echo "---"; echo
    echo "$ts_md"; echo
    echo "> All three measured end-to-end (publish→client) for a fair compare, all headless under **Deno**. zenoh-ts has no gateway in the read path → true per-client on-demand."
  } > bench/RESULTS.md
  echo "→ wrote combined bench/RESULTS.md (all 3 transports)"
else
  run_one "$MODE"
fi
