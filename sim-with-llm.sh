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

# Pick a blueprint compose that imports cleanly on Apple Silicon without
# google-genai. unitree-go2-agentic-gemini imports GeminiSpeakSkill at
# module load (before --disable can take effect), so we don't use it here.
# unitree-go2-basic + mcp-server + mcp-client gives the full agent loop;
# unitree-skill-container adds wait / current_time / execute_sport_command
# / tilt_body skills. relative_move needs the nav stack and is not in this
# compose — publish to /cmd_vel directly if you need movement.
BLUEPRINT="${BLUEPRINT:-unitree-go2-basic}"
export EXTRA="${EXTRA:-mcp-server mcp-client unitree-skill-container}"

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
