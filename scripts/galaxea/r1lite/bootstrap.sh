#!/bin/bash
# Production bootstrap for a Galaxea R1 Lite host (Final plan Step 0).
# Installs the host-side prerequisites dimos cannot apply from its container:
# persistent sysctls, the lo-multicast oneshot, and the release lifecycle
# unit. Idempotent. Run as root on the robot:
#     sudo bash scripts/galaxea/r1lite/bootstrap.sh
#
# Deliberately does NOT activate a release (Final plan: activation is always
# an explicit operator action) and does NOT touch the Galaxea vendor stack.
set -e

[ "$(id -u)" = "0" ] || { echo "run as root (sudo)"; exit 1; }
HOST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/host" && pwd)"

echo "[bootstrap] installing sysctls"
install -m 0644 "$HOST_DIR/60-dimos.conf" /etc/sysctl.d/60-dimos.conf
sysctl --system >/dev/null

echo "[bootstrap] installing systemd units"
install -m 0644 "$HOST_DIR/dimos-hostnet.service" /etc/systemd/system/
install -m 0644 "$HOST_DIR/dimos-r1lite.service" /etc/systemd/system/
systemctl daemon-reload

echo "[bootstrap] enabling units (starting hostnet only; release activation is operator-driven)"
systemctl enable --now dimos-hostnet.service
systemctl enable dimos-r1lite.service

mkdir -p /opt/dimos/releases

echo "[bootstrap] verification:"
bash "$HOST_DIR/bootstrap-verify.sh" && echo "[bootstrap] all prerequisites OK" || {
    echo "[bootstrap] FAILED checks above"; exit 1; }
