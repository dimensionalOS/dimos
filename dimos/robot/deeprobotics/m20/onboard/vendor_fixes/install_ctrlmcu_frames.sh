#!/bin/bash
# Installs (or with --remove, removes) the ctrlmcu open-print drop-in, its logrotate rule and an hourly size check. Run with sudo from this folder; restart ctrlmcu (then rl_deploy) to apply.
set -euo pipefail
dropin=/etc/systemd/system/ctrlmcu.service.d/open-print.conf
rotate=/etc/logrotate.d/ctrlmcu-frames
hourly=/etc/cron.hourly/ctrlmcu-frames
if [ "${1:-}" = "--remove" ]; then
    rm -f "$dropin" "$rotate" "$hourly"
    systemctl daemon-reload
    echo "removed (log kept in /var/opt/robot/data/ctrlmcu_frames); restart ctrlmcu, then rl_deploy"
    exit 0
fi
mkdir -p "$(dirname "$dropin")" /var/opt/robot/data/ctrlmcu_frames
install -m 644 ctrlmcu-open-print.conf "$dropin"
install -m 644 logrotate-ctrlmcu-frames "$rotate"
printf '#!/bin/sh\n# Rotates the ctrlmcu frame log when it passes its size cap (the daily logrotate timer alone is too slow for a burst).\nexec /usr/sbin/logrotate %s\n' "$rotate" > "$hourly"
chmod 755 "$hourly"
logrotate -d "$rotate" 2>&1 | grep -i error || true
systemctl daemon-reload
echo "installed; restart ctrlmcu, then rl_deploy"
