#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/compose.yaml"
SCENARIO="${1:-all}"

cleanup() {
    docker compose \
        -f "${COMPOSE_FILE}" \
        --profile basic \
        --profile replay \
        --profile sim \
        --profile visual \
        down \
        --remove-orphans
}

run_visual() {
    echo "Running hosted visual scenario"
    echo "After VISUAL:READY, open:"
    echo "http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy"
    echo "Press Ctrl-C to stop"
    docker compose \
        -f "${COMPOSE_FILE}" \
        --profile visual \
        up \
        --build
}
trap cleanup EXIT

run_scenario() {
    local scenario="$1"
    echo "Running hosted ${scenario} scenario"
    docker compose \
        -f "${COMPOSE_FILE}" \
        --profile "${scenario}" \
        up \
        --build \
        --abort-on-container-exit \
        --exit-code-from "verify-${scenario}"
    cleanup
}

case "${SCENARIO}" in
    basic|replay|sim)
        run_scenario "${SCENARIO}"
        ;;
    visual)
        run_visual
        ;;
    all)
        run_scenario basic
        run_scenario replay
        run_scenario sim
        ;;
    *)
        echo "Usage: $0 [basic|replay|sim|visual|all]" >&2
        exit 2
        ;;
esac
