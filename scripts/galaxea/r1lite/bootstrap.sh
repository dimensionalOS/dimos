#!/bin/bash
# Install the host prerequisites and systemd units for dimos on an R1 Lite.
# Does not activate a release. Run as root.
set -e

[ "$(id -u)" = "0" ] || { echo "run as root (sudo)"; exit 1; }
HOST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/host" && pwd)"

install -m 0644 "$HOST_DIR/60-dimos.conf" /etc/sysctl.d/60-dimos.conf
sysctl --system >/dev/null

install -m 0644 "$HOST_DIR/dimos-hostnet.service" /etc/systemd/system/
install -m 0644 "$HOST_DIR/dimos-r1lite.service" /etc/systemd/system/
systemctl daemon-reload

mkdir -p /opt/dimos/releases

systemctl enable --now dimos-hostnet.service
systemctl enable dimos-r1lite.service

bash "$HOST_DIR/bootstrap-verify.sh" && echo "[bootstrap] all prerequisites OK" || {
    echo "[bootstrap] FAILED checks above"; exit 1; }
