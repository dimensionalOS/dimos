#!/usr/bin/env bash
# Bring up a SocketCAN interface for OpenYAM (Linux only — on macOS the
# adapter talks to the dongle directly via gs_usb/slcan, no bring-up needed).
# Default is classical CAN @ 1 Mbit, which is what candlelight/gs_usb
# USB-CAN adapters support. Use MODE=fd if you have a CAN-FD-capable
# adapter (OpenYAM advertises 1 kHz control over CAN-FD).
# Run with sudo or as root.
#
# Usage:
#   sudo ./dimos/robot/manipulators/openyam/scripts/openyam_can_up.sh            # can0
#   sudo ./dimos/robot/manipulators/openyam/scripts/openyam_can_up.sh can1
#   sudo MODE=fd ./dimos/robot/manipulators/openyam/scripts/openyam_can_up.sh can0
set -euo pipefail

BITRATE=1000000
DBITRATE=5000000
MODE="${MODE:-classical}"   # classical | fd
IFACES_ARG="${*:-can0}"
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
