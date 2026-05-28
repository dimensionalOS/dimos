#!/usr/bin/env bash
#
# Run the greeter end-to-end in the MuJoCo office sim.
#
# Encodes the sim gotchas we hit the hard way:
#   - MuJoCo odom/TF/sensors are gated on a display, and MUST NOT use EGL.
#   - the Clash proxy breaks DimOS signaling + the MCP httpx client.
#   - the perception models are already cached, so run HuggingFace offline
#     (a live revalidation times out behind the CN network/proxy).
#
# The office scene provides chairs and a person mesh (the old red ball was
# removed). Plant the person and start the loop from a second shell:
#
#   PYTHONPATH=src python scripts/sim_person_pose.py --x 2.5 --y 0.0
#   dimos mcp call start_greeting        # begin wandering + greeting
#   dimos mcp call greeter_status        # current phase
#   dimos mcp call stop_greeting
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DIMOS_HOME="${DIMOS_HOME:-$HOME/sam/dimos}"
export DIMOS_VENV="${DIMOS_VENV:-$HOME/dimos-env}"
export PYTHONPATH="$REPO_ROOT/src:$DIMOS_HOME:${PYTHONPATH:-}"

# Sim sensors/odom need a display; do not force EGL (it stalls the sensor loop).
export DISPLAY="${DISPLAY:-:1}"
unset MUJOCO_GL || true

# The proxy breaks DimOS networking; perception models are cached, so go offline.
unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy all_proxy ALL_PROXY || true
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1

# The autonomous loop does not use the agent LLM, but the stack expects a model.
export PAWTRACK_MODEL="${PAWTRACK_MODEL:-openai:deepseek-chat}"

# The sim person mesh stands rather than sits, so target "a person" in sim.
# Override with GREETER_SUBJECT if your scene differs.
GREETER_SUBJECT="${GREETER_SUBJECT:-a person}"

# shellcheck disable=SC1091
source "$DIMOS_VENV/bin/activate"
cd "$DIMOS_HOME"

exec dimos --simulation run greeter-agentic \
  -o "greeterskillcontainer.description=${GREETER_SUBJECT}"
