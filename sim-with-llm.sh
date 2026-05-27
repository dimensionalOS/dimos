#!/usr/bin/env bash
# sim-with-llm.sh — sim + local LLM, one command.
#
# Thin wrapper around go2-start.sh: sets SIMULATION=1 and the backend preset.
# go2-start.sh handles the rest (auto-injects mcp-server/mcp-client when an
# LLM preset is set against a non-agentic blueprint).
#
# Usage:
#   ./sim-with-llm.sh                   # mlxvlm Gemma-4 (default chat LLM)
#   ./sim-with-llm.sh lmstudio          # LM Studio
#   ./sim-with-llm.sh mlxvlm qwen3      # mlxvlm with a specific loaded model
#   ./sim-with-llm.sh ollama            # local ollama (uses native ollama blueprint)
#
# Override the base blueprint if you want a different shape:
#   BLUEPRINT=unitree-go2-basic ./sim-with-llm.sh lmstudio   # default
#   BLUEPRINT=unitree-go2-agentic-ollama ./sim-with-llm.sh   # full agentic compose

set -euo pipefail

BACKEND="${1:-mlxvlm}"
shift || true

BLUEPRINT="${BLUEPRINT:-unitree-go2-basic}"

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
    # The ollama agentic blueprint already composes mcp-server/mcp-client.
    BLUEPRINT="unitree-go2-agentic-ollama"
    ;;
  *)
    echo "unknown backend: $BACKEND  (use mlxvlm | lmstudio | ollama)" >&2
    exit 2
    ;;
esac

exec env SIMULATION=1 ./go2-start.sh "$BLUEPRINT"
