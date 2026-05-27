#!/usr/bin/env bash
# sim-with-llm.sh — sim + Mac-friendly agentic + local LLM, one command.
#
# Defaults to `unitree-go2-agentic-gemini` because it's the only agentic
# variant that boots on Apple Silicon: it disables SecurityModule (EdgeTAM
# needs CUDA), swaps Moondream (crashes Metal) → Gemini for VL, and replaces
# the OpenAI-hardcoded TTS with Gemini TTS. We only override its *chat* LLM
# (the McpClient) to point at LM Studio or mlxvlm. The VL/embedding/TTS
# pieces still need GOOGLE_API_KEY.
#
# Usage:
#   ./sim-with-llm.sh                   # mlxvlm Gemma-4 (default chat LLM)
#   ./sim-with-llm.sh lmstudio          # LM Studio
#   ./sim-with-llm.sh mlxvlm qwen3      # mlxvlm with a different loaded model
#   ./sim-with-llm.sh ollama            # local ollama (uses native ollama blueprint)
#
# Override the base blueprint if you need a different shape:
#   BLUEPRINT=unitree-go2-basic ./sim-with-llm.sh lmstudio    # no agent, just streams
#
# Required env (for VL/embedding/TTS on Mac):
#   GOOGLE_API_KEY=…    # https://aistudio.google.com/app/apikey

set -euo pipefail

BACKEND="${1:-mlxvlm}"
shift || true

# Minimal Mac+local-LLM compose that imports cleanly without google-genai:
#   unitree-go2-basic   (camera + 3D viz)
#   + mcp-server + mcp-client            (the MCP agent loop)
# Skills exposed (from GO2Connection): observe, play_wav, play_wav_b64.
#
# To add movement / sport / tilt skills, also pass:
#   EXTRA="mcp-server mcp-client unitree-skill-container replanning-a-star-planner"
# (the planner satisfies UnitreeSkillContainer's NavigationInterfaceSpec dep,
#  and pulls in the nav-stack chain — heavier but enables relative_move).
BLUEPRINT="${BLUEPRINT:-unitree-go2-basic}"
export EXTRA="${EXTRA:-mcp-server mcp-client}"

case "$BACKEND" in
  mlxvlm)
    export MLXVLM=1
    if [[ -n "${1:-}" ]]; then export MLXVLM_MODEL="$1"; fi
    ;;
  lmstudio)
    export LMSTUDIO=1
    if [[ -n "${1:-}" ]]; then export LMSTUDIO_MODEL="$1"; fi
    ;;
  ollama)
    # The ollama agentic blueprint is already a clean Mac compose.
    BLUEPRINT="unitree-go2-agentic-ollama"
    export EXTRA=""
    ;;
  *)
    echo "unknown backend: $BACKEND  (use mlxvlm | lmstudio | ollama)" >&2
    exit 2
    ;;
esac

exec env SIMULATION=1 ./go2-start.sh "$BLUEPRINT"
