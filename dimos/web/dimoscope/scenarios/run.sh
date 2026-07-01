#!/usr/bin/env bash
# Launch ONE scenario blueprint. Ctrl-C to stop, then run another — the gateway taps the bus, so
# the browser's live topic set follows whatever is publishing.
#
#   ./scenarios/run.sh nav      # mobile navigation  → /nav/*
#   ./scenarios/run.sh arm      # manipulation       → /arm/*
#   ./scenarios/run.sh cam      # perception         → /cam/*
#
# Env: DIMOS_TRANSPORT=zenoh|lcm (default zenoh, matching the app + `deno task serve`).
# Prereq: a gateway is running (`deno task serve`) and the app (`deno task app`).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # → dimos/web/dimoscope
S="${1:-}"
case "$S" in
  nav | arm | cam) ;;
  *)
    echo "usage: $0 <nav|arm|cam>" >&2
    exit 2
    ;;
esac
export DIMOS_TRANSPORT="${DIMOS_TRANSPORT:-zenoh}"
echo "▶ scope-$S  (DIMOS_TRANSPORT=$DIMOS_TRANSPORT)  — Ctrl-C to stop, then run another"
cd "$HERE"
exec uv run python "scenarios/$S.py"
