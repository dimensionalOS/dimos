#!/usr/bin/env bash
# Start every dimoscope transport at once, each on its own port, for the in-app
# server dropdown:
#   Python↔Zenoh gateway   ws://localhost:8088   (servers/gateway_zenoh.py)
#   Bun↔LCM gateway        ws://localhost:8089   (servers/gateway.ts; :8090 is DimSim's)
#   zenoh-ts direct bridge ws://localhost:10000  (zenoh-bridge-remote-api, peer mode)
#
# Provide a data source + the app SEPARATELY (heavy / interactive):
#   DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run unitree-go2
#   cd dimos/web/dimoscope/app && bun run dev          # http://localhost:5173
#
# Ctrl-C tears everything down.
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"     # dimoscope/
REPO="$(cd "$HERE/../../.." && pwd)"          # dimos repo root
PY="$REPO/.venv/bin/python"
BRIDGE="$(command -v zenoh-bridge-remote-api || echo "$HOME/.cargo/bin/zenoh-bridge-remote-api")"
pids=()
cleanup() { echo; echo "[start-all] stopping…"; for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
cd "$HERE"

echo "[start-all] Python↔Zenoh gateway  → ws://localhost:8088"
DIMOS_TRANSPORT=zenoh GATEWAY_PORT=8088 ZENOH_KEY='**' "$PY" servers/gateway_zenoh.py & pids+=($!)

echo "[start-all] Bun↔LCM gateway       → ws://localhost:8089"
GATEWAY_PORT=8089 bun run servers/gateway.ts & pids+=($!)

if [ -x "$BRIDGE" ]; then
  echo "[start-all] zenoh-ts bridge       → ws://localhost:10000  (remote-api, peer)"
  "$BRIDGE" --ws-port 10000 & pids+=($!)
else
  echo "[start-all] zenoh-ts bridge SKIPPED — install: cargo install zenoh-bridge-remote-api"
fi

echo "[start-all] all up. Ctrl-C to stop. (start a sim + 'cd app && bun run dev' separately)"
wait
