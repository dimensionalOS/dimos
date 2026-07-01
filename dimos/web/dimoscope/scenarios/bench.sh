#!/usr/bin/env bash
# Benchmark all 3 scenarios over the WS transport: for each, start its publisher → measure its
# topics for DUR_MS → stop. Emits scenarios/RESULTS.md. Only starts/stops ITS OWN publisher PIDs.
#
#   bash scenarios/bench.sh          # → scenarios/RESULTS.md
# Env: DIMOS_TRANSPORT=zenoh|lcm · GW_URL=ws://localhost:8080/ws · DUR_MS=6000 · WARM_S=18
# Prereq: a gateway is running (`deno task serve`). For the cleanest numbers, run it against a
# freshly-started serve (a quiet bus); other live publishers add background load.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # → dimos/web/dimoscope
cd "$HERE"
export DIMOS_TRANSPORT="${DIMOS_TRANSPORT:-zenoh}"
URL="${GW_URL:-ws://localhost:8080/ws}"
DUR="${DUR_MS:-6000}"
WARM="${WARM_S:-18}"  # cold import (open3d for nav/cam) before topics flow
OUT="scenarios/RESULTS.md"

{
  echo "# Scenario data-path benchmark"
  echo
  echo "_${DUR}ms/scenario · ${URL} · DIMOS_TRANSPORT=${DIMOS_TRANSPORT} · WS transport_"
  echo
  echo "| scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | loss% |"
  echo "|---|--:|--:|--:|--:|--:|--:|--:|--:|"
} >"$OUT"

for S in nav arm cam; do
  echo "▶ $S — starting publisher…"
  uv run python "scenarios/$S.py" >/dev/null 2>&1 &
  PID=$!
  sleep "$WARM"
  echo "▶ $S — measuring ${DUR}ms…"
  deno run -A scenarios/bench_scope.ts "$S" "$DUR" "$URL" >>"$OUT" ||
    echo "| $S | — | measure failed | | | | | | |" >>"$OUT"
  kill "$PID" 2>/dev/null
  wait "$PID" 2>/dev/null
  sleep 2
done

echo "✔ wrote $OUT"
cat "$OUT"
