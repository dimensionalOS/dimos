#!/bin/zsh

# Copyright 2026 dimos contributors
# SPDX-License-Identifier: MIT

set -u

SCRIPT_DIR="${0:A:h}"
PROJECT_DIR="${SCRIPT_DIR:h}"
DIMOS_BIN="${PROJECT_DIR}/.venv/bin/dimos"
CONTROL_PORTS=(7779 9990 3030 9877)

if [[ -x "${DIMOS_BIN}" ]]; then
  "${DIMOS_BIN}" stop >/dev/null 2>&1 || true
fi

# The older agent checkout starts its own DimOS/MCP process and can retain
# ports 7779 and 9990. Only terminate process groups whose parent command
# exactly contains that legacy module name.
LEGACY_PIDS=()
while IFS= read -r pid; do
  [[ -n "${pid}" ]] && LEGACY_PIDS+=("${pid}")
done < <(pgrep -f -- '-m dimos_dog_mcp\.blueprint' 2>/dev/null || true)

for pid in "${LEGACY_PIDS[@]}"; do
  command_line="$(ps -p "${pid}" -o command= 2>/dev/null || true)"
  if [[ "${command_line}" != *"-m dimos_dog_mcp.blueprint"* ]]; then
    continue
  fi

  process_group="$(ps -p "${pid}" -o pgid= 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "${process_group}" ]]; then
    kill -TERM -- "-${process_group}" 2>/dev/null || true
  else
    kill -TERM "${pid}" 2>/dev/null || true
  fi
done

for _ in {1..100}; do
  busy_pids="$(
    for port in "${CONTROL_PORTS[@]}"; do
      lsof -nP -iTCP:"${port}" -sTCP:LISTEN -t 2>/dev/null || true
    done | sort -u
  )"
  [[ -z "${busy_pids}" ]] && {
    echo "DimOS control ports are ready."
    exit 0
  }
  sleep 0.1
done

echo "Unable to release DimOS control ports:" >&2
for port in "${CONTROL_PORTS[@]}"; do
  listeners="$(lsof -nP -iTCP:"${port}" -sTCP:LISTEN -Fpct 2>/dev/null || true)"
  [[ -n "${listeners}" ]] && {
    echo "port ${port}" >&2
    echo "${listeners}" >&2
  }
done
exit 2
