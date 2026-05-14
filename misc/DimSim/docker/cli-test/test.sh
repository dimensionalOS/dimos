#!/usr/bin/env bash
set -euo pipefail

echo "=== DimSim CLI Install Test ==="
echo ""

# 1. Verify CLI is installed
echo "--- Step 1: Verify CLI installed ---"
which dimsim
dimsim help 2>&1 | head -5
echo ""

# 2. Setup core assets
echo "--- Step 2: dimsim setup ---"
dimsim setup --local /app/dimsim-core-v0.1.0.tar.gz
echo ""

# 3. Verify core installed
echo "--- Step 3: Verify core ---"
ls -la ~/.dimsim/dist/
echo ""

# 4. Install apt scene
echo "--- Step 4: dimsim scene install apt ---"
dimsim scene install apt --local /app/scene-apt-v0.1.0.tar.gz
echo ""

# 5. List scenes
echo "--- Step 5: dimsim scene list ---"
dimsim scene list
echo ""

# 6. Verify scene installed
echo "--- Step 6: Verify scene files ---"
ls -lh ~/.dimsim/dist/sims/
echo ""

# 7. Start dev server briefly and verify it responds
echo "--- Step 7: dimsim dev (quick test) ---"
dimsim dev --scene apt --port 8090 &
DEV_PID=$!

# Wait for server to start
for i in $(seq 1 15); do
  if curl -s http://localhost:8090 > /dev/null 2>&1; then
    echo "Server responding after ${i}s"
    break
  fi
  if [ "$i" -eq 15 ]; then
    echo "FAIL: Server never started"
    kill $DEV_PID 2>/dev/null || true
    exit 1
  fi
  sleep 1
done

# Verify HTML response contains DimSim
RESP=$(curl -s http://localhost:8090)
if echo "$RESP" | grep -q "dimosMode"; then
  echo "HTML contains dimosMode injection — correct!"
else
  echo "FAIL: HTML missing dimosMode"
  kill $DEV_PID 2>/dev/null || true
  exit 1
fi

# Verify scene asset is served
STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8090/sims/apt.json)
if [ "$STATUS" = "200" ]; then
  echo "Scene apt.json served — HTTP 200"
else
  echo "FAIL: Scene not served (HTTP $STATUS)"
  kill $DEV_PID 2>/dev/null || true
  exit 1
fi

kill $DEV_PID 2>/dev/null || true

echo ""
echo "=== ALL TESTS PASSED ==="
