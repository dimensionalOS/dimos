#!/usr/bin/env bash
# Bring up CAN interfaces for OpenArm. Default is CAN-FD @ 1M/5M.
# Use MODE=classical if you need classical CAN @ 1 Mbit.
# Run with sudo or as root.
#
# Usage:
#   sudo ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh                 # CAN-FD 1M/5M, can0 and can1
#   sudo ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh can0            # single interface
#   sudo MODE=classical ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh can0
set -euo pipefail

BITRATE=1000000
DBITRATE=5000000
MODE="${MODE:-fd}"   # classical | fd
IFACES_ARG="${*:-can0 can1}"
# shellcheck disable=SC2206
IFACES=(${IFACES_ARG[@]})

for IF in "${IFACES[@]}"; do
    if ! ip link show "$IF" >/dev/null 2>&1; then
        echo "[skip] $IF not present"
        continue
    fi
    ip link set "$IF" down || true
    if [ "$MODE" = "classical" ]; then
        echo "[up  ] $IF  ${BITRATE}  (classical CAN)"
        ip link set "$IF" type can bitrate "$BITRATE"
    else
        echo "[up  ] $IF  ${BITRATE}/${DBITRATE} fd on"
        ip link set "$IF" type can bitrate "$BITRATE" dbitrate "$DBITRATE" fd on
    fi
    ip link set "$IF" up
    ip link set "$IF" txqueuelen 1000
    ip -details link show "$IF" | grep -E "can |bitrate" || true
done
