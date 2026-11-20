#!/bin/bash
# Stops m20-shadow if rl_deploy's "send Data update timeout" count (its log of a late 1 kHz cycle) rises faster than
# its background rate while the shadow runs. Event-driven: it follows rl_deploy's log and judges on each line. A
# restart of rl_deploy (its "start logging" banner, or a new MainPID seen on any event) resets the count, and the
# timeouts of its first SETTLE_S are not counted (a fresh rl_deploy logs a burst while the bus comes up).
#   guard.sh PER_HOUR SLACK SETTLE_S
set -uo pipefail
per_hour="${1:?per hour}"; slack="${2:?slack}"; settle="${3:?settle seconds}"
log() { echo "/var/opt/robot/log/$(date +%Y_%m%d)/rl_deploy.$(date +%Y_%m%d).log"; }
pid() { systemctl show -p MainPID --value rl_deploy; }
restart() {
    rl_pid=$(pid); counted=0; settled_at=$(( $(date +%s) + settle ))
    echo "guard: rl_deploy pid $rl_pid; counting timeouts from $(date -d @"$settled_at" +%T), $per_hour/h + $slack allowed"
}
restart
while true; do
    # follow today's log until midnight, then the next day's (tail -F waits for a file that is not there yet)
    until_midnight=$(( $(date -d 'tomorrow 00:00:00' +%s) - $(date +%s) + 1 ))
    while IFS= read -r line; do
        now=$(date +%s)
        if [[ "$line" == *"start logging"* ]] || [ "$(pid)" != "$rl_pid" ]; then
            restart
            continue
        fi
        [[ "$line" == *"send Data update timeout"* ]] || continue
        [ "$now" -ge "$settled_at" ] || continue
        counted=$(( counted + 1 ))
        allowed=$(( per_hour * (now - settled_at) / 3600 + slack ))
        echo "guard: timeout $counted in $(( now - settled_at ))s since settle (allowed $allowed)"
        if [ "$counted" -gt "$allowed" ]; then
            echo "guard: rl_deploy timeouts rising faster than background: stopping m20-shadow"
            systemctl stop m20-shadow
            exit 0
        fi
    done < <(timeout "$until_midnight" tail -F -n 0 "$(log)" 2>/dev/null)
done
