#!/usr/bin/env bash
# Benchmark harness: the one dimoscope service (the gateway — all transports on one port) + the
# synthetic bench_publisher on BOTH buses (LCM + Zenoh) so /bench/p0..3 + /bench/grid flow.
# No DimSim needed (controlled source, low noise). Then either:
#   • open  http://localhost:8080/bench.html  → Run   (all transports, in-browser; needs `deno task build`)
#   • run   bench/run.sh all                          (headless: lcm + zenoh source)
# Ctrl-C tears everything down.
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"        # dimos repo root
PY="$REPO/.venv/bin/python"
pids=()
cleanup() { echo; echo "[serve-bench] stopping…"; for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
cd "$HERE"

echo "[serve-bench] dimoscope service (:8080 — all transports on one port)"
"$PY" -m gateway & pids+=($!)

sleep 5   # let the service come up (cold dimos import for egress + zenoh)
echo "[serve-bench] bench_publisher → LCM bus  (/bench/* @ 100Hz)"
DIMOS_TRANSPORT=lcm BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
echo "[serve-bench] bench_publisher → Zenoh bus (/bench/* @ 100Hz)"
DIMOS_TRANSPORT=zenoh BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)

echo "[serve-bench] up → open http://localhost:8080/bench.html and click Run (Ctrl-C to stop)"
wait
