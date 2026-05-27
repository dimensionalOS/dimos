#!/usr/bin/env bash
# PACK MIND — live Go2 venue bring-up (Go2 EDU onboard-Jetson topology).
#
# Run this from the operator laptop AFTER joining the dog's WiFi (dimair14).
# It probes the dog, guides the on-dog stack launch, then validates G0 and
# launches the conductor. Read-only probing; the on-dog launch is explicit.
#
# Override defaults via env:  DOG=192.168.12.1 SSH_USER=unitree MCP_PORT=9990 ./venue_go2.sh
set -uo pipefail

DOG="${DOG:-192.168.12.1}"
SSH_USER="${SSH_USER:-unitree}"
PORT="${MCP_PORT:-9990}"
SELF="$(cd "$(dirname "$0")" && pwd)"

stage() { printf '\n=== %s ===\n' "$1"; }
ok()    { printf '  PASS: %s\n' "$1"; }
bad()   { printf '  FAIL: %s\n' "$1"; }

stage "0  network — am I on dimair14 and can I see the dog?"
if ping -c 2 -t 3 "$DOG" >/dev/null 2>&1; then
  ok "dog reachable at $DOG"
else
  bad "cannot reach $DOG — join WiFi dimair14 (pw 88888888) first, then re-run."
  exit 1
fi

stage "1  probe the dog (prompts for the dog's ssh password)"
echo "  ssh ${SSH_USER}@${DOG} ..."
ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=no "${SSH_USER}@${DOG}" '
  echo "  host: $(hostname)";
  printf "  cuda: "; (nvidia-smi -L 2>/dev/null | head -1 || echo "NO nvidia-smi — wrong host or no GPU");
  printf "  dimos: "; (command -v dimos 2>/dev/null || ls -d ~/dimos 2>/dev/null || echo "NOT FOUND — DimOS not installed here");
  printf "  port '"$PORT"' busy?: "; (ss -ltn 2>/dev/null | grep ":'"$PORT"'" || echo "free");
' || { bad "ssh failed — check user/host/password. Default Unitree user is 'unitree'."; exit 1; }

cat <<EOF

=== 2  launch the stack ON THE DOG (do this in a separate ssh session) ===
  ssh ${SSH_USER}@${DOG}
    cd ~/dimos && source .venv/bin/activate          # adjust path if different
    dimos run unitree-go2-agentic \\
        --listen-host 0.0.0.0 \\
        --mcp-port ${PORT} \\
        --daemon
    dimos mcp list-tools     # MUST list: speak, navigate_with_text, look_out_for
  # If the dog doesn't connect, add: --robot-ip 192.168.123.161
  # CRITICAL: --listen-host 0.0.0.0 or your laptop gets connection-refused.

Press ENTER here once 'dimos mcp list-tools' looks good on the dog...
EOF
read -r _

stage "3  G0 — MCP reachability from THIS laptop over the LAN"
RESP="$(curl -s --max-time 10 -X POST "http://${DOG}:${PORT}/mcp" \
  -H 'content-type: application/json' \
  -d '{"jsonrpc":"2.0","id":"1","method":"tools/list"}')"
if echo "$RESP" | grep -q '"speak"'; then
  ok "G0 — MCP reachable, speak tool present"
  echo "$RESP" | (jq -r '.result.tools[].name' 2>/dev/null | sed 's/^/    tool: /' || true)
else
  bad "no tool list back. Got: ${RESP:0:200}"
  echo "  -> re-check --listen-host 0.0.0.0 on the dog, then dog firewall."
  exit 1
fi

stage "4  G1 — make the REAL dog speak (direct MCP, bypasses conductor)"
curl -s --max-time 10 -X POST "http://${DOG}:${PORT}/mcp" \
  -H 'content-type: application/json' \
  -d '{"jsonrpc":"2.0","id":"2","method":"tools/call","params":{"name":"speak","arguments":{"text":"Pack mind online.","blocking":false}}}' \
  | (jq -r '.result.content[].text' 2>/dev/null || cat)
echo "  (listen — the dog should have spoken)"

stage "5  launch the conductor + dashboard"
cat <<EOF
  python ${SELF}/conductor.py --dog alpha=${DOG}:${PORT}
  # open http://localhost:8080  -> drive start_act1 / inject_found / ask_where / send_dog / verify
  # second dog: add  --dog bravo=<DOG2_IP>:9990
EOF
echo "Done. Single-dog path is live once the dashboard speaks through the dog."
