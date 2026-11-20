#!/bin/bash
# Installs (or with --remove, removes) the drop-ins that keep lio_perception, rsdriver, yesense and height_map_nav on cores 6-7. Run with sudo from this folder; restart those units (then rl_deploy) to apply.
set -euo pipefail
units="lio_perception rsdriver yesense height_map_nav"
if [ "${1:-}" = "--remove" ]; then
    for u in $units; do rm -f "/etc/systemd/system/$u.service.d/cpuset-6-7.conf"; done
    systemctl daemon-reload
    echo "removed; restart $units (then rl_deploy) to go back to the vendor's 4-7"
    exit 0
fi
mkdir -p /opt/robot/vendor_fixes
install -m 755 in_cpuset.sh /opt/robot/vendor_fixes/in_cpuset.sh
for u in $units; do
    mkdir -p "/etc/systemd/system/$u.service.d"
    install -m 644 "cpuset-$u.conf" "/etc/systemd/system/$u.service.d/cpuset-6-7.conf"
done
systemctl daemon-reload
echo "installed; restart $units (then rl_deploy) to apply"
