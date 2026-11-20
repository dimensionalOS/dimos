#!/bin/bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
#
# M20 motor-bus recorder: the ECAN frames rl_deploy exchanges with the motor board (via the ecan_tap.so LD_PRELOAD
# shim, which the kernel's private ECAN path makes the only way to see them) and, time-aligned, the commands sent to
# the robot. Runs as root on AOS (10.21.31.103); `wrap`/`unwrap` also work on GOS.
#   record.sh install       shim into rl_deploy.service + restart rl_deploy (robot sitting/docked), wrap m20tool here
#   record.sh <seconds>     one session in session_<time>/: motor frames, basic_server + DDS + m20tool traffic,
#                           rl_deploy/basic_server logs and the vendor CSV of those seconds; then timeline.py on it
#   record.sh stop          end a running recording early; it still saves everything
#   record.sh status        shim loaded?, live reader?, recent sessions
#   record.sh uninstall     remove the shim + restart rl_deploy (robot sitting/docked), unwrap m20tool
#   record.sh wrap|unwrap   log every m20tool run on this board (bind mount, gone after a reboot)
#                           (on GOS: dir=/var/tmp/ecan_capture record.sh wrap)
# Charge-enable test controls in the shim (off unless the file exists, no restart needed):
#   touch $dir/drop_charge_enable    rl_deploy's 0xb3 charge-enable frames are swallowed; rm it to stop
#   touch $dir/inject_charge_enable  the next 3 zero-gain joint commands go out as 0xb3 (one request); the shim removes it
# Removing the shim entirely: record.sh uninstall (restarts rl_deploy: robot sitting/docked).
# Options: --force skips the robot-is-sitting check before a restart.
set -euo pipefail
dir=${dir:-/var/opt/robot/data/ecan_capture}
dropin=/etc/systemd/system/rl_deploy.service.d/ecan-tap.conf
aos=10.21.31.103
# basic_server's UDP/TCP ports, where wrapped m20tool runs announce themselves, and Fast-DDS's port range
basic_udp=30000
basic_tcp=30001
note_port=30999
dds_ports=7400-7700
# keep enough of each packet for every command and DDS sample header; lidar/camera payloads are not commands
snaplen=512
log_dir() { echo "/var/opt/robot/log/$(date +%Y_%m%d)"; }
rl_log() { echo "$(log_dir)/rl_deploy.$(date +%Y_%m%d).log"; }
bs_log() { echo "$(log_dir)/Basic_server.$(date +%Y_%m%d).log"; }
force=0
args=()
for arg in "$@"; do
    if [ "$arg" = --force ]; then force=1; else args+=("$arg"); fi
done
set -- "${args[@]+"${args[@]}"}"

pid() { pgrep -x rl_deploy | head -1; }
shim_loaded() { local p; p=$(pid) && grep -q ecan_tap.so /proc/$p/maps; }
timeouts() { grep -c 'send Data update timeout' "$(rl_log)" 2>/dev/null || true; }
imu_timeouts() { grep -c 'IMU数据更新超时' "$(rl_log)" 2>/dev/null || true; }

require_sitting() {
    [ "$force" = 1 ] && return 0
    local status
    status=$(m20tool status 2>&1 || true)
    echo "$status" | head -3
    if echo "$status" | grep -qiE '^motion: *(idle|sit|lie|damp)' || echo "$status" | grep -qiE '^charge: *(charging|on dock)'; then
        return 0
    fi
    echo "the robot does not look sitting or docked; restarting rl_deploy now would drop it (--force to override)" >&2
    exit 1
}

restart_and_wait() {
    require_sitting
    systemctl daemon-reload
    systemctl restart rl_deploy
    echo "restarted rl_deploy; waiting for it to come up (run.sh sleeps 12 s; it can crash-loop a few times on bad_alloc first)"
    for _ in $(seq 1 120); do
        sleep 2
        if p=$(pid) && [ -n "$p" ] && [ "$(ps -o etimes= -p "$p" | tr -d " ")" -gt 15 ]; then
            echo "rl_deploy up, pid $p"
            return 0
        fi
    done
    echo "rl_deploy did not stay up within 4 min; check: systemctl status rl_deploy" >&2
    return 1
}

# a copy, not a second bind mount: with shared mount propagation the wrapper's bind would land on that one too
# (the wrapper would then call itself); under $dir, not /run, which is noexec
real=$dir/m20tool-real
wrapper=$dir/m20tool-wrapper
mounted() { grep -q " $1 " /proc/self/mountinfo; }
wrap() {
    mkdir -p "$dir"
    mounted /usr/local/bin/m20tool && { echo "m20tool already wrapped"; return 0; }
    [ -x /usr/local/bin/m20tool ] || { echo "no /usr/local/bin/m20tool here"; return 0; }
    # only ever bind the real binary, never a wrapper (a wrapper calling itself forks without end)
    [ "$(head -c 4 /usr/local/bin/m20tool | od -An -tx1 | tr -d ' ')" = 7f454c46 ] || { echo "/usr/local/bin/m20tool is not the ELF binary; not wrapping" >&2; return 1; }
    cp /usr/local/bin/m20tool "$real"
    touch /var/tmp/m20tool_invocations.log
    chmod 666 /var/tmp/m20tool_invocations.log
    # Each run: one line in the local log and the same line as a UDP note to AOS, where the session's tcpdump has it.
    cat > "$wrapper" <<EOF
#!/bin/bash
[ -n "\$M20TOOL_WRAPPED" ] && { echo "m20tool wrapper loop; run record.sh unwrap" >&2; exit 99; }
export M20TOOL_WRAPPED=1
note() { local line="\$(date +%s%N) \$(hostname) \$(id -un) ppid=\$PPID \$*"; echo "\$line" >> /var/tmp/m20tool_invocations.log 2>/dev/null; echo "M20TOOL \$line" > /dev/udp/$aos/$note_port 2>/dev/null || true; }
note start m20tool "\$@"
$real "\$@"
rc=\$?
note end rc=\$rc m20tool "\$@"
exit \$rc
EOF
    chmod 755 "$wrapper"
    mount --bind "$wrapper" /usr/local/bin/m20tool
    echo "m20tool wrapped: runs go to /var/tmp/m20tool_invocations.log and UDP $aos:$note_port"
}
unwrap() {
    while mounted /usr/local/bin/m20tool; do umount /usr/local/bin/m20tool || break; done
    while mounted "$real"; do umount "$real" || break; done
    rm -f "$real" "$wrapper"
    echo "m20tool unwrapped"
}

record() {
    local seconds=$1 stamp session start_ns end_ns tcpdump_pid rl_offset bs_offset bs_since
    shim_loaded || { echo "shim not loaded; run: $0 install" >&2; exit 1; }
    # since 2026-11-21 yesense_node hands /IMU_DATA to rl_deploy over shared memory, which no capture sees
    echo "note: the body IMU may be missing from net.pcap (vendor shared memory)" >&2
    stamp=$(date +%Y%m%d_%H%M%S)
    session="$dir/session_$stamp"
    mkdir -p "$session"
    local before_net before_imu tx0 rx0 tx1 rx1
    before_net=$(timeouts); before_imu=$(imu_timeouts)
    tx0=$(cat /sys/class/net/eth1/statistics/tx_packets); rx0=$(cat /sys/class/net/eth1/statistics/rx_packets)
    rl_offset=$(stat -c %s "$(rl_log)" 2>/dev/null || echo 0)
    bs_offset=$(stat -c %s "$(bs_log)" 2>/dev/null || echo 0)
    bs_since=$(date '+%Y-%m-%d %H:%M:%S')
    # commands: basic_server UDP/TCP (both ways), m20tool notes, every DDS packet (cmd_vel reaches rl_deploy as /NAV_CMD)
    taskset -c 1-3 nice -n 10 tcpdump -i any -nn -s "$snaplen" -B 16384 --immediate-mode -w "$session/net.pcap" \
        "udp port $basic_udp or tcp port $basic_tcp or udp port $note_port or (udp portrange $dds_ports)" \
        2> "$session/tcpdump.log" &
    tcpdump_pid=$!
    sleep 1  # tcpdump gives no readiness signal; let it open its capture before the session starts
    rm -f "$dir/last_stats"
    start_ns=$(date +%s%N)
    touch "$dir/enable"
    echo "M20TOOL $start_ns $(hostname) record.sh session start $session" > /dev/udp/127.0.0.1/$note_port || true
    echo "recording $seconds s into $session (end early: $0 stop, or Ctrl-C)"
    # the wait ends at the timeout, on `record.sh stop` (SIGUSR1) or on Ctrl-C/TERM; the session is saved either way
    echo $$ > "$dir/recording.pid"
    sleep "$seconds" &
    local sleeper=$!
    trap 'kill $sleeper 2>/dev/null' USR1 INT TERM HUP
    wait "$sleeper" 2>/dev/null || true
    trap - USR1 INT TERM HUP
    kill "$sleeper" 2>/dev/null || true
    rm -f "$dir/enable" "$dir/recording.pid"
    end_ns=$(date +%s%N)
    echo "M20TOOL $end_ns $(hostname) record.sh session end" > /dev/udp/127.0.0.1/$note_port || true
    # the shim's writer writes last_stats when it closes the capture (it checks the enable file every 50 ms)
    for _ in $(seq 1 50); do [ -f "$dir/last_stats" ] && break; sleep 0.1; done
    sleep 0.5  # tcpdump has no flush signal; let its last packets land before stopping it
    kill -INT "$tcpdump_pid" 2>/dev/null || true
    wait "$tcpdump_pid" 2>/dev/null || true
    tx1=$(cat /sys/class/net/eth1/statistics/tx_packets); rx1=$(cat /sys/class/net/eth1/statistics/rx_packets)
    mv "$(cat "$dir/latest_path")" "$session/ecan.pcapng"
    [ -f /var/tmp/m20tool_invocations.log ] && cp /var/tmp/m20tool_invocations.log "$session/m20tool_aos.log"
    tail -c +"$((rl_offset + 1))" "$(rl_log)" > "$session/rl_deploy.log" 2>/dev/null || true
    tail -c +"$((bs_offset + 1))" "$(bs_log)" > "$session/basic_server.log" 2>/dev/null || true
    journalctl -u basic_server -u rl_deploy --since "$bs_since" -o short-iso-precise --no-pager > "$session/journal.log" 2>/dev/null || true
    # rl_deploy's once-a-second CSV (battery, temperatures) for the same seconds
    local csv
    csv=$(ls -t /var/opt/robot/log/system/2*.csv 2>/dev/null | head -1 || true)
    if [ -n "$csv" ]; then
        { head -1 "$csv"; awk -F, -v a="$(date -d @$((start_ns / 1000000000)) +%Y%m%d_%H%M%S)" -v b="$(date -d @$((end_ns / 1000000000 + 1)) +%Y%m%d_%H%M%S)" 'NR > 1 && $1 >= a && $1 <= b' "$csv"; } > "$session/vendor_log.csv"
    fi
    {
        echo "start_ns=$start_ns"
        echo "end_ns=$end_ns"
        echo "shim: $(cat "$dir/last_stats" 2>/dev/null || echo 'no stats')"
        echo "eth1 tx $((tx1 - tx0)) rx $((rx1 - rx0))"
        echo "rl_deploy 'send Data update timeout' +$(( $(timeouts) - before_net )), 'IMU update timeout' +$(( $(imu_timeouts) - before_imu ))"
        grep -E 'packets (captured|dropped)' "$session/tcpdump.log" || true
    } | tee "$session/meta.txt"
    if command -v python3 >/dev/null && [ -f "$dir/timeline.py" ]; then
        taskset -c 1-3 nice -n 10 python3 "$dir/timeline.py" "$session" || echo "timeline.py failed; run it again on a laptop" >&2
    fi
    echo "session: $session ($(du -sh "$session" | cut -f1))"
}

case "${1:-status}" in
    install)
        [ -f "$dir/ecan_tap.so" ] || { echo "missing $dir/ecan_tap.so (gcc -O2 -shared -fPIC -o ecan_tap.so ecan_tap.c -ldl -lpthread)" >&2; exit 1; }
        rm -f "$dir/enable"
        printf '[Service]\nEnvironment=LD_PRELOAD=%s/ecan_tap.so\nEnvironment=ECAN_TAP_DIR=%s\n' "$dir" "$dir" > "$dropin"
        restart_and_wait
        shim_loaded && echo "shim loaded" || { echo "shim NOT loaded" >&2; exit 1; }
        wrap
        ;;
    uninstall)
        rm -f "$dir/enable" "$dropin"
        restart_and_wait
        shim_loaded && { echo "shim still loaded?" >&2; exit 1; } || echo "shim removed, rl_deploy running normally"
        unwrap
        ;;
    wrap) wrap ;;
    unwrap) unwrap ;;
    stop)
        [ -f "$dir/recording.pid" ] && kill -USR1 "$(cat "$dir/recording.pid")" 2>/dev/null \
            && echo "asked the recording (pid $(cat "$dir/recording.pid")) to stop; it saves the session and exits" \
            || { echo "no recording running" >&2; exit 1; }
        ;;
    status)
        if shim_loaded; then echo "shim loaded in rl_deploy pid $(pid)"; else echo "shim not loaded (rl_deploy pid $(pid || true))"; fi
        [ -f "$dropin" ] && echo "drop-in present: $dropin" || echo "no drop-in"
        [ -f "$dir/recording.pid" ] && kill -0 "$(cat "$dir/recording.pid")" 2>/dev/null && echo "recording now (pid $(cat "$dir/recording.pid"))"
        [ -f "$dir/drop_charge_enable" ] && echo "DROPPING rl_deploy's charge-enable frames (rm $dir/drop_charge_enable)"
        [ -f "$dir/charge_control_stats" ] && echo "charge controls: $(cat "$dir/charge_control_stats")"
        [ -S "$dir/live.sock" ] && echo "live reader bound: $dir/live.sock" || echo "no live reader"
        mounted /usr/local/bin/m20tool && echo "m20tool wrapped" || echo "m20tool not wrapped"
        ls -dt "$dir"/session_* "$dir"/capture_*.pcapng 2>/dev/null | head -5 || true
        ;;
    *[!0-9]*)
        sed -n '5,19p' "$0"; exit 1
        ;;
    *)
        record "$1"
        ;;
esac
