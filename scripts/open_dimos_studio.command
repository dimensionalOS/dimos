#!/bin/zsh

set -e

SCRIPT_DIR="${0:A:h}"
PROJECT_DIR="${SCRIPT_DIR:h}"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
STUDIO_URL="http://127.0.0.1:8765"
LOG_FILE="${PROJECT_DIR}/logs/dimos-studio-app.log"

mkdir -p "${PROJECT_DIR}/logs"

export NO_PROXY="${NO_PROXY:+${NO_PROXY},}localhost,127.0.0.1"
export no_proxy="${NO_PROXY}"

if ! curl --silent --fail "${STUDIO_URL}/api/health" >/dev/null 2>&1; then
  # Start in normal headed-browser mode. ``dimos.web.studio`` opens the
  # system browser once the local server is ready.
  nohup "${PYTHON_BIN}" -m dimos.web.studio >>"${LOG_FILE}" 2>&1 &
  for _ in {1..30}; do
    if curl --silent --fail "${STUDIO_URL}/api/health" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
else
  # When Studio is already running, bring it forward in a regular browser tab.
  open "${STUDIO_URL}"
fi
