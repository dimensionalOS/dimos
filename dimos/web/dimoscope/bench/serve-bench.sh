#!/usr/bin/env bash
# Benchmark harness: the 3 transports' servers (start-all.sh) + the synthetic
# bench_publisher on BOTH buses (LCM + Zenoh) so /bench/p0..3 + /bench/grid flow on every
# transport. No DimSim needed (controlled source, low noise). Then either:
#   • open  http://localhost:5173/bench.html  → Run   (all 3 transports, in-browser)
#   • run   bench/run.sh all                          (headless Bun: lcm + zenoh [+ ts])
# Ctrl-C tears everything down.
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"        # dimos repo root
PY="$REPO/.venv/bin/python"
pids=()
cleanup() { echo; echo "[serve-bench] stopping…"; for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
cd "$HERE"

echo "[serve-bench] 3 transport servers (zenoh :8088 · lcm :8089 · bridge :10000)"
bash servers/start-all.sh & pids+=($!)

sleep 4   # let the gateways + bridge come up
echo "[serve-bench] bench_publisher → LCM bus  (/bench/* @ 100Hz)"
DIMOS_TRANSPORT=lcm BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)
echo "[serve-bench] bench_publisher → Zenoh bus (/bench/* @ 100Hz)"
DIMOS_TRANSPORT=zenoh BENCH_HZ=100 "$PY" bench/bench_publisher.py & pids+=($!)

echo "[serve-bench] up → open http://localhost:5173/bench.html and click Run (Ctrl-C to stop)"
wait
