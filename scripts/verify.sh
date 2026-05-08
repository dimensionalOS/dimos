#!/usr/bin/env bash
# verify.sh - single source of truth for the agent pipeline.
# Both local developers and CI invoke this exact script.
#
# Tier: fast + mypy
#   - uv sync (dependency lock)
#   - uv run pytest -q (excludes slow / tool / mujoco markers by default)
#   - uv run mypy dimos/ (strict type check)
#
# Slow tier (`./bin/pytest-slow`) is invoked separately by full CI; this
# script is intended for the agent's per-PR feedback loop.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "================================================================"
echo " verify.sh @ $REPO_ROOT"
echo " host:  $(uname -srm)"
echo " shell: ${BASH_VERSION:-unknown}"
echo " date:  $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "================================================================"

echo ""
echo ">>> [1/3] uv sync (install/refresh deps)"
# AGENTS.md baseline: `uv sync --all-extras --no-extra dds`.
# On macOS arm64 we additionally exclude `cuda` (no wheel) and `unitree-dds`
# (pulls in cyclonedds, which needs CYCLONEDDS_HOME to build from source).
UV_EXTRA_FLAGS=(--all-extras --no-extra dds)
case "$(uname -s)" in
    Darwin) UV_EXTRA_FLAGS+=(--no-extra cuda --no-extra unitree-dds) ;;
esac
echo "uv sync ${UV_EXTRA_FLAGS[*]}"
uv sync "${UV_EXTRA_FLAGS[@]}"
echo "<<< OK"

echo ""
echo ">>> [2/3] uv run pytest -q  (fast suite: excludes slow/tool/mujoco)"
uv run pytest -q
echo "<<< OK"

echo ""
echo ">>> [3/3] uv run mypy dimos/  (strict type check)"
uv run mypy dimos/
echo "<<< OK"

echo ""
echo "================================================================"
echo " verify.sh: ALL GREEN"
echo "================================================================"
