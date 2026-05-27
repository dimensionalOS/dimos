#!/usr/bin/env bash
# go2-start.sh — hackathon quickstart for the dimairos05 Go2
# Usage:
#   ./go2-start.sh                       # default blueprint: unitree-go2-basic
#   ./go2-start.sh unitree-go2-agentic-gemini
#   ROBOT_IP=10.0.0.42 ./go2-start.sh    # override IP (robot on shared wifi)

set -euo pipefail

# ---- config ----------------------------------------------------------------
ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
EXPECTED_SSID="${EXPECTED_SSID:-dimairos05}"
BLUEPRINT="${1:-unitree-go2-basic}"
VENV_DIR="${VENV_DIR:-.venv}"
# ----------------------------------------------------------------------------

c_red()   { printf '\033[31m%s\033[0m\n' "$*"; }
c_green() { printf '\033[32m%s\033[0m\n' "$*"; }
c_yellow(){ printf '\033[33m%s\033[0m\n' "$*"; }
c_bold()  { printf '\033[1m%s\033[0m\n'  "$*"; }

c_bold "▶ Go2 hackathon quickstart"
echo "  blueprint : $BLUEPRINT"
echo "  robot ip  : $ROBOT_IP"
echo

# 1. Wifi check (macOS) ------------------------------------------------------
if [[ "$(uname)" == "Darwin" ]]; then
  SSID="$(ipconfig getsummary en0 2>/dev/null | awk -F' SSID : ' '/ SSID : /{print $2; exit}')"
  if [[ -z "$SSID" ]]; then
    SSID="$(ipconfig getsummary en1 2>/dev/null | awk -F' SSID : ' '/ SSID : /{print $2; exit}')"
  fi
  if [[ "$SSID" == "$EXPECTED_SSID" ]]; then
    c_green "✓ on wifi $SSID"
  else
    c_yellow "⚠ current wifi is '${SSID:-unknown}', expected '$EXPECTED_SSID'"
    c_yellow "  if the robot is on a different network, set ROBOT_IP and ignore this"
  fi
else
  c_yellow "⚠ non-macOS host, skipping wifi check"
fi

# 2. Reachability ------------------------------------------------------------
echo
echo "→ pinging $ROBOT_IP …"
if ping -c 3 -W 1000 "$ROBOT_IP" >/dev/null 2>&1; then
  c_green "✓ robot reachable"
else
  c_red "✗ cannot reach $ROBOT_IP"
  c_red "  check: joined dimairos05? robot powered on? sport mode on in the Unitree app?"
  exit 1
fi

# 3. Clock sync (Unitree video desyncs without this) -------------------------
echo
echo "→ syncing clock (sudo, may prompt for password) …"
if command -v sntp >/dev/null 2>&1; then
  sudo sntp -sS pool.ntp.org >/dev/null 2>&1 \
    && c_green "✓ clock synced" \
    || c_yellow "⚠ clock sync failed (non-fatal, but video may lag lidar)"
elif command -v ntpdate >/dev/null 2>&1; then
  sudo ntpdate pool.ntp.org >/dev/null 2>&1 \
    && c_green "✓ clock synced" \
    || c_yellow "⚠ clock sync failed (non-fatal)"
else
  c_yellow "⚠ no sntp/ntpdate found, skipping clock sync"
fi

# 4. Venv --------------------------------------------------------------------
echo
if [[ -d "$VENV_DIR" ]]; then
  c_green "✓ using existing venv $VENV_DIR"
else
  c_yellow "no venv at $VENV_DIR — creating one"
  if ! command -v uv >/dev/null 2>&1; then
    c_red "✗ uv is not installed. install: brew install uv  (or curl -LsSf https://astral.sh/uv/install.sh | sh)"
    exit 1
  fi
  uv venv --python 3.12 "$VENV_DIR"
  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"
  uv pip install 'dimos[base,unitree]'
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# 5. Env vars ----------------------------------------------------------------
export ROBOT_IP
# Agentic-gemini blueprint needs these; warn if missing for that path
if [[ "$BLUEPRINT" == *"gemini"* && -z "${GOOGLE_API_KEY:-}" ]]; then
  c_yellow "⚠ GOOGLE_API_KEY not set — gemini blueprint will fail when the agent runs"
fi
if [[ "$BLUEPRINT" == *"agentic"* && "$BLUEPRINT" != *"ollama"* && "$BLUEPRINT" != *"gemini"* ]]; then
  if [[ -z "${OPENAI_API_KEY:-}" ]]; then
    c_yellow "⚠ OPENAI_API_KEY not set — agentic blueprint will fail when the agent runs"
  fi
fi

# 6. Launch ------------------------------------------------------------------
echo
c_bold "▶ launching: dimos run $BLUEPRINT"
echo "  command center → http://localhost:7779"
echo "  ctrl-c to stop"
echo
exec dimos run "$BLUEPRINT"
