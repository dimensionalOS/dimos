#!/usr/bin/env bash
set -euo pipefail

echo "=== DimSim + dimos Integration Test ==="

# --- Multicast setup ---
echo "Setting up multicast routing..."
ip link set lo multicast on 2>/dev/null || true
ip route add 224.0.0.0/4 dev lo 2>/dev/null || true

# --- Start bridge server (DimSim) ---
echo "Starting DimSim bridge server..."
cd /app/DimSim
deno run --allow-all --unstable-net \
  dimos-cli/cli.ts dev --scene apt --port 8090 --headless --render cpu \
  &> /tmp/bridge.log &
BRIDGE_PID=$!
cd /app

# Wait for HTTP to be ready
echo "Waiting for bridge..."
for i in $(seq 1 30); do
  if curl -s http://localhost:8090 > /dev/null 2>&1; then
    echo "Bridge ready (${i}s)"
    break
  fi
  if [ "$i" -eq 30 ]; then
    echo "FAIL: Bridge never started"
    cat /tmp/bridge.log
    exit 1
  fi
  sleep 1
done

# Wait for headless browser to load scene + start publishing
# SwiftShader CPU rendering is very slow — needs more time
echo "Waiting for scene load (30s)..."
sleep 30

# --- Run dimos TS controller ---
echo "Starting dimos TS controller (main_custom_multicast.ts)..."
echo "Will timeout after 60s if no data received."

# Create sensor output dir so the controller doesn't crash
mkdir -p /app/sensor_output

cd /app/dimos/examples/language-interop/ts
timeout 60 deno run --allow-net --allow-write --unstable-net \
  main_custom_multicast.ts &> /tmp/controller.log &
CTRL_PID=$!
cd /app

# Monitor: wait for 3+ odom messages (poll controller output)
PASS=false
for i in $(seq 1 60); do
  ODOM_COUNT="$(grep -c '\[recv\] odom' /tmp/controller.log 2>/dev/null)" || ODOM_COUNT=0
  if [ "$ODOM_COUNT" -ge 3 ]; then
    PASS=true
    break
  fi
  sleep 1
done

# Cleanup
kill $CTRL_PID 2>/dev/null || true
kill $BRIDGE_PID 2>/dev/null || true

echo ""
echo "--- Controller output ---"
cat /tmp/controller.log

if $PASS; then
  echo ""
  echo "=== TEST PASSED — dimos ↔ DimSim multicast integration works ==="
  exit 0
else
  echo ""
  echo "=== TEST FAILED — no odom data received ==="
  echo "--- Bridge logs ---"
  tail -50 /tmp/bridge.log
  exit 1
fi
