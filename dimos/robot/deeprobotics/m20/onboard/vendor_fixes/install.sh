#!/bin/bash
# Builds hide_tailscale.so on AOS and installs the ctrlmcu drop-in (run with sudo from this folder). --remove undoes it.
set -euo pipefail
dropin=/etc/systemd/system/ctrlmcu.service.d/hide-tailscale.conf
if [ "${1:-}" = "--remove" ]; then
    rm -f "$dropin"
    systemctl daemon-reload
    echo "removed $dropin (hide_tailscale.so left in /opt/robot/vendor_fixes); restart ctrlmcu to apply"
    exit 0
fi
mkdir -p /opt/robot/vendor_fixes "$(dirname "$dropin")"
gcc -O2 -Wall -Wextra -shared -fPIC -o /opt/robot/vendor_fixes/hide_tailscale.so hide_tailscale.c -ldl
install -m 644 ctrlmcu-hide-tailscale.conf "$dropin"
systemctl daemon-reload
echo "installed $dropin; start/restart ctrlmcu to apply"
