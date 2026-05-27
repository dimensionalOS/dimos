#!/usr/bin/env bash
# sim-with-llm.sh — sim + agentic + local LLM, one command.
#
# Usage:
#   ./sim-with-llm.sh                   # mlxvlm Gemma-4 (default)
#   ./sim-with-llm.sh lmstudio          # LM Studio
#   ./sim-with-llm.sh mlxvlm qwen3      # mlxvlm with a different loaded model
#   BLUEPRINT=unitree-go2-agentic-ollama ./sim-with-llm.sh ollama
#
# Forwards to go2-start.sh with SIMULATION=1 and the right preset.

set -euo pipefail

BACKEND="${1:-mlxvlm}"
shift || true

BLUEPRINT="${BLUEPRINT:-unitree-go2-agentic}"

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
    BLUEPRINT="unitree-go2-agentic-ollama"
    ;;
  *)
    echo "unknown backend: $BACKEND  (use mlxvlm | lmstudio | ollama)" >&2
    exit 2
    ;;
esac

exec env SIMULATION=1 ./go2-start.sh "$BLUEPRINT"
