#!/usr/bin/env bash
set -euo pipefail

export UV_CACHE_DIR="${UV_CACHE_DIR:-${TMPDIR:-/tmp}/dogops-uv-cache}"
export NO_PROXY="${NO_PROXY:-127.0.0.1,localhost}"
export no_proxy="${no_proxy:-127.0.0.1,localhost}"

RUN_DIR="${RUN_DIR:-.dogops/runs/latest}"
LOG_DIR="${LOG_DIR:-.dogops/preflight/$(date +%Y%m%d-%H%M%S)}"
ROBOT_STARTED=0
DOGOPS_REQUIRED_TOOLS=(
  run_mission
  go_to
  follow_route
  stop_route
  route_status
  scan_zone
  read_gauge
  check_clearance
  detect_blocked_aisle
  scan_receiving_manifest
  verify_work_order
  nav_eval_report
)
if [[ -n "${UV_RUN_ARGS:-}" ]]; then
  read -r -a uv_run_args <<< "${UV_RUN_ARGS}"
else
  uv_run_args=(--no-sync)
fi
mkdir -p "${LOG_DIR}"

log() {
  printf '%s\n' "$*" | tee -a "${LOG_DIR}/preflight.log"
}

run() {
  log "+ $*"
  "$@" 2>&1 | tee -a "${LOG_DIR}/preflight.log"
}

run_optional() {
  log "+ $*"
  if ! "$@" 2>&1 | tee -a "${LOG_DIR}/preflight.log"; then
    log "WARN: optional command failed: $*"
  fi
}

uv_run() {
  uv run "${uv_run_args[@]}" "$@"
}

require_match() {
  local pattern="$1"
  local path="$2"
  local label="$3"
  if ! rg "${pattern}" "${path}" >/dev/null; then
    log "FAILED: ${label} did not match ${pattern}"
    log "Saved evidence in ${LOG_DIR}"
    return 1
  fi
  log "OK: ${label}"
}

cleanup() {
  if [[ "${ROBOT_STARTED}" == "1" ]]; then
    log "cleanup: stopping DimOS robot runtime"
    uv_run dimos stop --force 2>&1 | tee -a "${LOG_DIR}/preflight.log" || true
  fi
}

trap cleanup EXIT

log "DogOps Go2 preflight"
log "cwd=$(pwd)"
log "run_dir=${RUN_DIR}"
log "log_dir=${LOG_DIR}"
log "uv_run_args=${uv_run_args[*]}"
log "stop_command=uv run ${uv_run_args[*]} dimos stop --force"

run uv_run python --version
run uv_run python -m dimos.experimental.dogops.cli validate
run uv_run pytest -q dimos/experimental/dogops
run uv_run python -m dimos.experimental.dogops.cli simulate --out "${RUN_DIR}"

log "+ uv_run dimos list"
uv_run dimos list >"${LOG_DIR}/dimos-list.txt" 2>"${LOG_DIR}/dimos-list.err"
require_match "unitree-go2" "${LOG_DIR}/dimos-list.txt" "base Go2 registry"
require_match "dogops" "${LOG_DIR}/dimos-list.txt" "DogOps registry"

if [[ -n "${GO2_IP:-}" ]]; then
  run ping -c 3 "${GO2_IP}"
else
  log "GO2_IP is not set; hardware ping and smoke are skipped."
  log "Set GO2_IP, then rerun this script before touching the real robot."
fi

if [[ "${RUN_GO2_SMOKE:-0}" == "1" ]]; then
  if [[ -z "${GO2_IP:-}" ]]; then
    log "FAILED: RUN_GO2_SMOKE=1 requires GO2_IP"
    exit 1
  fi
  run_optional uv_run dimos stop --force
  run uv_run dimos --viewer none run unitree-go2 -o "go2connection.ip=${GO2_IP}" --daemon
  ROBOT_STARTED=1
  run uv_run dimos status
  run uv_run dimos log -n 100
  run uv_run dimos stop --force
  ROBOT_STARTED=0
else
  log "Base Go2 smoke not started. Use RUN_GO2_SMOKE=1 after the route is clear."
fi

if [[ "${RUN_DOGOPS_SMOKE:-0}" == "1" ]]; then
  if [[ -z "${GO2_IP:-}" ]]; then
    log "FAILED: RUN_DOGOPS_SMOKE=1 requires GO2_IP"
    exit 1
  fi
  run_optional uv_run dimos stop --force
  run uv_run dimos --viewer none run unitree-go2-dogops -o "go2connection.ip=${GO2_IP}" --daemon
  ROBOT_STARTED=1
  run uv_run dimos status
  log "+ uv_run dimos mcp list-tools"
  uv_run dimos mcp list-tools >"${LOG_DIR}/mcp-tools.txt" 2>"${LOG_DIR}/mcp-tools.err"
  for tool in "${DOGOPS_REQUIRED_TOOLS[@]}"; do
    require_match "${tool}" "${LOG_DIR}/mcp-tools.txt" "DogOps MCP tool ${tool}"
  done
  run uv_run dimos mcp call run_mission --json-args '{"mission_id":"receiving_sre_demo"}'
  run uv_run dimos mcp call scan_zone --json-args '{"zone_id":"INBOUND_DOCK"}'
  run uv_run dimos mcp call scan_receiving_manifest --json-args '{"zone_id":"INBOUND_DOCK"}'
  run uv_run dimos mcp call read_gauge --json-args '{"asset_id":"TEMP_1"}'
  run uv_run dimos mcp call check_clearance --json-args '{"asset_id":"COOLING_1"}'
  run uv_run dimos mcp call detect_blocked_aisle --json-args '{"zone_id":"AISLE_1"}'
  run uv_run dimos mcp call route_status
  run uv_run dimos mcp call nav_eval_report
  run uv_run dimos log -n 200
  run uv_run dimos stop --force
  ROBOT_STARTED=0
else
  log "DogOps hardware smoke not started. Use RUN_DOGOPS_SMOKE=1 only after base Go2 smoke passes."
fi

log "DogOps Go2 preflight complete."
