#!/bin/bash
# Stops m20-shadow if rl_deploy's "send Data update timeout" count (its log of a late 1 kHz cycle) rises faster than
# its background rate while the shadow runs. Rate and slack are arguments, so they live in the unit file.
#   guard.sh PER_HOUR SLACK CHECK_S
set -uo pipefail
per_hour="${1:?per hour}"; slack="${2:?slack}"; every="${3:?check seconds}"
log() { echo "/var/opt/robot/log/$(date +%Y_%m%d)/rl_deploy.$(date +%Y_%m%d).log"; }
count() { grep -c 'send Data update timeout' "$(log)" 2>/dev/null || echo 0; }
start=$(date +%s); base=$(count); day=$(date +%Y_%m%d)
echo "guard: start count $base, allowing $per_hour/h + $slack"
while sleep "$every"; do
    if [ "$(date +%Y_%m%d)" != "$day" ]; then  # a new day's log starts from zero
        day=$(date +%Y_%m%d); base=0; start=$(date +%s)
    fi
    now=$(count); elapsed=$(( $(date +%s) - start ))
    allowed=$(( per_hour * elapsed / 3600 + slack ))
    echo "guard: +$(( now - base )) timeouts in ${elapsed}s (allowed $allowed)"
    if [ $(( now - base )) -gt "$allowed" ]; then
        echo "guard: rl_deploy timeouts rising faster than background: stopping m20-shadow"
        systemctl stop m20-shadow
        exit 0
    fi
done
