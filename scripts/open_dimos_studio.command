#!/bin/zsh

set -e

SCRIPT_DIR="${0:A:h}"
PROJECT_DIR="${SCRIPT_DIR:h}"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
STUDIO_URL="http://127.0.0.1:8765"
LOG_FILE="${PROJECT_DIR}/logs/dimos-studio-app.log"

mkdir -p "${PROJECT_DIR}/logs"

if ! curl --silent --fail "${STUDIO_URL}/api/health" >/dev/null 2>&1; then
  nohup "${PYTHON_BIN}" -m dimos.web.studio --no-open >>"${LOG_FILE}" 2>&1 &
  for _ in {1..30}; do
    if curl --silent --fail "${STUDIO_URL}/api/health" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
fi

if [[ -d "/Applications/Google Chrome.app" ]]; then
  open -na "Google Chrome" --args --app="${STUDIO_URL}"
else
  open "${STUDIO_URL}"
fi
