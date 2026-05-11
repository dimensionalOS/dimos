#!/usr/bin/env bash
set -euo pipefail

# Headless smoke for DimOS Unitree G1 simulation entrypoints.
# Use RUN_SECONDS to limit runtime; set DIMOS_INSTALL=1 to install deps into .venv.
# Set DIMOS_USE_SYSTEM=1 inside containers where dependencies are already installed
# into the system Python.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

: "${PYTHON_VERSION:=3.12}"
: "${RUN_SECONDS:=20}"
: "${BLUEPRINT:=unitree-g1-sim}"
: "${DIMOS_INSTALL:=0}"
: "${DIMOS_USE_SYSTEM:=0}"

if [[ "$DIMOS_USE_SYSTEM" != "1" ]]; then
  if [[ ! -d .venv ]]; then
    uv venv --python "$PYTHON_VERSION"
  fi
  # shellcheck disable=SC1091
  source .venv/bin/activate

  if [[ "$DIMOS_INSTALL" == "1" ]]; then
    uv pip install '.[unitree,sim]'
  fi
elif [[ "$DIMOS_INSTALL" == "1" ]]; then
  uv pip install --system '.[unitree,sim]'
fi

python - <<'PY'
from dimos.robot.all_blueprints import all_blueprints
required = {
    "unitree-g1-sim",
    "unitree-g1-basic-sim",
    "unitree-g1-agentic-sim",
    "unitree-g1-nav-sim",
}
missing = sorted(required - set(all_blueprints))
if missing:
    raise SystemExit(f"Missing G1 simulation blueprints: {missing}")
print("G1 simulation blueprints OK:", ", ".join(sorted(required)))
PY

run_limited() {
  if command -v timeout >/dev/null 2>&1; then
    timeout "$RUN_SECONDS" "$@"
    return $?
  fi
  if command -v gtimeout >/dev/null 2>&1; then
    gtimeout "$RUN_SECONDS" "$@"
    return $?
  fi
  python - "$RUN_SECONDS" "$@" <<'PY'
import subprocess
import sys

seconds = float(sys.argv[1])
cmd = sys.argv[2:]
try:
    raise SystemExit(subprocess.run(cmd, timeout=seconds).returncode)
except subprocess.TimeoutExpired:
    raise SystemExit(124)
PY
}

set +e
run_limited dimos --simulation --viewer none run "$BLUEPRINT"
status=$?
set -e

if [[ "$status" == "124" ]]; then
  echo "G1 simulation smoke reached RUN_SECONDS=${RUN_SECONDS}; treating bounded run as pass."
  exit 0
fi
exit "$status"
