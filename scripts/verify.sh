#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "================================================================"
echo " verify.sh @ $REPO_ROOT"
echo " host: $(uname -srm)"
echo "================================================================"

run_python_check() {
    local command="$1"

    if [[ -x /entrypoint.sh ]]; then
        /entrypoint.sh bash -c "source .venv/bin/activate && $command"
    else
        bash -c "source .venv/bin/activate && $command"
    fi
}

echo ">>> [1/3] Install Python dependencies"
uv sync --all-extras --no-extra dds --no-extra unitree-dds --frozen
echo "<<< OK"

echo ">>> [setup] Remove pydrake stubs when present"
find .venv/lib/*/site-packages/pydrake -name '*.pyi' -delete 2>/dev/null || true
echo "<<< OK"

echo ">>> [2/3] Run tests"
run_python_check "pytest --durations=0 -m 'not (tool or mujoco)'"
echo "<<< OK"

echo ">>> [3/3] Run mypy"
run_python_check "MYPYPATH=/opt/ros/humble/lib/python3.10/site-packages mypy dimos"
echo "<<< OK"

