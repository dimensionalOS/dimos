#!/bin/bash
# Stops m20-shadow if rl_deploy's "send Data update timeout" count (its log of a late 1 kHz cycle) rises faster than
# its own background while the shadow runs. The background is measured from rl_deploy's log over the
# guard_baseline_s before the guard starts (or before a restart's settle ends), so a rate that was already high
# without the shadow does not stop it. Event-driven: it follows rl_deploy's log and judges on each line. A restart of
# rl_deploy (its "start logging" banner, or a new MainPID seen on any event) re-measures, and the timeouts of its
# first guard_settle_s are not counted (a fresh rl_deploy logs a burst while the bus comes up).
#   guard.sh SHADOW_TOML     (guard_* values in it; decisions are also appended to guard_log)
set -uo pipefail
conf="${1:?shadow.toml}"
val() {
    local v
    v=$(awk -F= -v k="$1" '{gsub(/[ \t]/, "", $1)} $1 == k {gsub(/[ \t"]/, "", $2); sub(/#.*/, "", $2); print $2; exit}' "$conf")
    [ -n "$v" ] || { echo "guard: $1 missing from $conf" >&2; exit 2; }
    echo "$v"
}
min_per_hour=$(val guard_per_hour) || exit 2
factor=$(val guard_factor) || exit 2
slack=$(val guard_slack) || exit 2
settle=$(val guard_settle_s) || exit 2
baseline_s=$(val guard_baseline_s) || exit 2
out=$(val guard_log) || exit 2
say() { echo "$(date -Is) $*" | tee -a "$out"; }
log() { echo "/var/opt/robot/log/$(date +%Y_%m%d)/rl_deploy.$(date +%Y_%m%d).log"; }
pid() { systemctl show -p MainPID --value rl_deploy; }
# timeouts rl_deploy logged in the baseline_s before $1 (epoch s), excluding a restart's first settle_s
background() {
    local from to
    from=$(date -d @"$(( $1 - baseline_s ))" '+[%Y-%m-%d %H:%M:%S'); to=$(date -d @"$1" '+[%Y-%m-%d %H:%M:%S')
    awk -v a="$from" -v b="$to" '$0 >= a && $0 < b && /send Data update timeout/' "$(log)" | wc -l
}
measure() {
    local base rate
    base=$(background "$1")
    # per hour, scaled by the factor, never below the configured floor
    rate=$(awk -v n="$base" -v s="$baseline_s" -v f="$factor" -v m="$min_per_hour" 'BEGIN {r = f * n * 3600 / s; print int(r > m ? r : m)}')
    per_hour=$rate
    say "guard: background $base timeouts in the ${baseline_s}s before $(date -d @"$1" +%T): allowing $per_hour/h + $slack"
}
restart() {
    rl_pid=$(pid); counted=0; settled_at=$(( $(date +%s) + settle ))
    say "guard: rl_deploy pid $rl_pid; counting timeouts from $(date -d @"$settled_at" +%T)"
    measured=0
}
restart
measure "$(date +%s)"; measured=1
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
        # after a restart the background is the settled rl_deploy's own: measure it once the settle ends
        [ "$measured" = 1 ] || { measure "$now"; measured=1; }
        counted=$(( counted + 1 ))
        allowed=$(( per_hour * (now - settled_at) / 3600 + slack ))
        echo "guard: timeout $counted in $(( now - settled_at ))s since settle (allowed $allowed)"
        if [ "$counted" -gt "$allowed" ]; then
            say "guard: STOPPING m20-shadow: $counted rl_deploy timeouts in $(( now - settled_at ))s, allowed $allowed ($per_hour/h + $slack)"
            systemctl stop m20-shadow
            exit 0
        fi
    done < <(timeout "$until_midnight" tail -F -n 0 "$(log)" 2>/dev/null)
done
