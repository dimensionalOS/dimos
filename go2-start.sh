#!/usr/bin/env bash
# go2-start.sh — hackathon quickstart for the dimairos05 Go2
#
# Usage:
#   ./go2-start.sh                                   # unitree-go2-basic + web bridges
#   ./go2-start.sh unitree-go2-agentic-gemini        # any blueprint
#   ROBOT_IP=10.0.0.42 ./go2-start.sh                # override IP
#   BRIDGES=0 ./go2-start.sh                         # skip camera-mjpeg + audio-ws
#   EXTRA="foo-module bar-module" ./go2-start.sh     # tack on more modules
#   SIMULATION=1 ./go2-start.sh                      # MuJoCo sim (camera only, no audio)
#
# Local LLM presets — pick exactly one (default model can be overridden):
#   LMSTUDIO=1 ./go2-start.sh unitree-go2-agentic
#     -> OpenAI-compat at http://127.0.0.1:1234/v1
#     -> LMSTUDIO_MODEL=qwen/qwen3-8b (override if a different model is loaded)
#
#   MLXVLM=1 ./go2-start.sh unitree-go2-agentic
#     -> OpenAI-compat at http://127.0.0.1:8080/v1  (mlxvlm Gemma-4 server)
#     -> MLXVLM_MODEL=lmstudio-community/gemma-4-E4B-it-MLX-4bit
#
# Default extras (with BRIDGES=1):
#   camera-mjpeg-module     http://127.0.0.1:7780/video_feed/color_image
#                           http://127.0.0.1:7780/snapshot/color_image
#   audio-ws-module         ws://127.0.0.1:7781/audio_out   (robot only)
#                           POST http://127.0.0.1:7781/play (robot only)

set -euo pipefail

# ---- config ----------------------------------------------------------------
ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
EXPECTED_SSID="${EXPECTED_SSID:-dimairos05}"
BLUEPRINT="${1:-unitree-go2-basic}"
VENV_DIR="${VENV_DIR:-.venv}"
BRIDGES="${BRIDGES:-1}"
EXTRA="${EXTRA:-}"
SIMULATION="${SIMULATION:-0}"
LMSTUDIO="${LMSTUDIO:-0}"
MLXVLM="${MLXVLM:-0}"
LMSTUDIO_MODEL="${LMSTUDIO_MODEL:-qwen/qwen3-8b}"
MLXVLM_MODEL="${MLXVLM_MODEL:-lmstudio-community/gemma-4-E4B-it-MLX-4bit}"
# ----------------------------------------------------------------------------

c_red()   { printf '\033[31m%s\033[0m\n' "$*"; }
c_green() { printf '\033[32m%s\033[0m\n' "$*"; }
c_yellow(){ printf '\033[33m%s\033[0m\n' "$*"; }
c_bold()  { printf '\033[1m%s\033[0m\n'  "$*"; }

# Sanity: only one LLM preset at a time.
if [[ "$LMSTUDIO" == "1" && "$MLXVLM" == "1" ]]; then
  c_red "✗ pick one: LMSTUDIO=1 or MLXVLM=1, not both"
  exit 1
fi

# Assemble the module list once so it's reused for echo + exec.
MODULES=("$BLUEPRINT")
if [[ "$BRIDGES" == "1" ]]; then
  MODULES+=("camera-mjpeg-module" "audio-ws-module")
fi
if [[ -n "$EXTRA" ]]; then
  # shellcheck disable=SC2206
  MODULES+=( $EXTRA )
fi

# Optional LLM endpoint override + model selection for the McpClient.
LLM_ARGS=()
LLM_NAME=""
LLM_URL=""
LLM_MODEL=""
if [[ "$LMSTUDIO" == "1" ]]; then
  LLM_NAME="LM Studio"
  LLM_URL="http://127.0.0.1:1234/v1"
  LLM_MODEL="$LMSTUDIO_MODEL"
elif [[ "$MLXVLM" == "1" ]]; then
  LLM_NAME="mlxvlm (Gemma 4)"
  LLM_URL="http://127.0.0.1:8080/v1"
  LLM_MODEL="$MLXVLM_MODEL"
fi
if [[ -n "$LLM_URL" ]]; then
  export OPENAI_BASE_URL="$LLM_URL"
  export OPENAI_API_KEY="${OPENAI_API_KEY:-local-llm}"  # placeholder; servers ignore it
  LLM_ARGS=( -o "mcpclient.model=openai:$LLM_MODEL" )
  if [[ "$BLUEPRINT" != *"agentic"* ]]; then
    c_yellow "⚠ LLM preset is set but blueprint '$BLUEPRINT' has no McpClient — override is a no-op"
  fi
fi

c_bold "▶ Go2 hackathon quickstart"
echo "  modules   : ${MODULES[*]}"
if [[ "$SIMULATION" == "1" ]]; then
  echo "  mode      : simulation (no robot)"
else
  echo "  robot ip  : $ROBOT_IP"
fi
if [[ -n "$LLM_NAME" ]]; then
  echo "  LLM       : $LLM_NAME → $LLM_URL"
  echo "  model     : $LLM_MODEL"
fi
echo

if [[ "$SIMULATION" != "1" ]]; then
  # 1. Wifi check (macOS) ----------------------------------------------------
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

  # 2. Reachability ----------------------------------------------------------
  echo
  echo "→ pinging $ROBOT_IP …"
  if ping -c 3 -W 1000 "$ROBOT_IP" >/dev/null 2>&1; then
    c_green "✓ robot reachable"
  else
    c_red "✗ cannot reach $ROBOT_IP"
    c_red "  check: joined dimairos05? robot powered on? sport mode on in the Unitree app?"
    c_red "  (use SIMULATION=1 ./go2-start.sh to run MuJoCo instead)"
    exit 1
  fi

  # 3. Clock sync (Unitree video desyncs without this) -----------------------
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
fi

# 4. LLM endpoint reachability check ----------------------------------------
if [[ -n "$LLM_URL" ]]; then
  echo
  echo "→ probing $LLM_NAME at $LLM_URL/models …"
  if curl -fsS -m 3 "$LLM_URL/models" >/dev/null 2>&1; then
    c_green "✓ $LLM_NAME reachable"
  else
    c_red "✗ cannot reach $LLM_NAME at $LLM_URL"
    if [[ "$LMSTUDIO" == "1" ]]; then
      c_red "  start LM Studio's Local Server (Cmd-Shift-2) and load a tool-capable model"
    else
      c_red "  start mlxvlm: cd /Users/tex/repos/ai/mlx/mlxvlm && scripts/start-all.sh"
    fi
    exit 1
  fi
fi

# 5. Venv --------------------------------------------------------------------
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
  if [[ "$SIMULATION" == "1" ]]; then
    uv pip install 'dimos[base,unitree,sim]'
  else
    uv pip install 'dimos[base,unitree]'
  fi
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# 6. Env vars ----------------------------------------------------------------
export ROBOT_IP
# Agentic-gemini blueprint needs these; warn if missing for that path
if [[ "$BLUEPRINT" == *"gemini"* && -z "${GOOGLE_API_KEY:-}" ]]; then
  c_yellow "⚠ GOOGLE_API_KEY not set — gemini blueprint will fail when the agent runs"
fi
if [[ -z "$LLM_URL" && "$BLUEPRINT" == *"agentic"* && "$BLUEPRINT" != *"ollama"* && "$BLUEPRINT" != *"gemini"* ]]; then
  if [[ -z "${OPENAI_API_KEY:-}" ]]; then
    c_yellow "⚠ OPENAI_API_KEY not set — agentic blueprint will fail when the agent runs"
    c_yellow "  (or set LMSTUDIO=1 / MLXVLM=1 to use a local LLM)"
  fi
fi

# 7. Launch ------------------------------------------------------------------
echo
c_bold "▶ endpoints"
echo "  command center : http://localhost:7779"
if [[ "$BRIDGES" == "1" ]]; then
  echo "  camera MJPEG   : http://127.0.0.1:7780/video_feed/color_image"
  echo "  camera snapshot: http://127.0.0.1:7780/snapshot/color_image"
  if [[ "$SIMULATION" == "1" ]]; then
    echo "  audio          : (robot only — sim has no audio)"
  else
    echo "  audio out (ws) : ws://127.0.0.1:7781/audio_out"
    echo "  audio info     : http://127.0.0.1:7781/audio_info"
    echo "  audio play     : POST http://127.0.0.1:7781/play"
  fi
fi
echo
c_bold "▶ launching: dimos $([[ "$SIMULATION" == "1" ]] && echo "--simulation ")run ${MODULES[*]} ${LLM_ARGS[*]:-}"
echo "  ctrl-c to stop"
echo

if [[ "$SIMULATION" == "1" ]]; then
  exec dimos --simulation run "${MODULES[@]}" "${LLM_ARGS[@]}"
else
  exec dimos run "${MODULES[@]}" "${LLM_ARGS[@]}"
fi
