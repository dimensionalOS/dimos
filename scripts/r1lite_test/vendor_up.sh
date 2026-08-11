#!/bin/bash
# Galaxea vendor stack bring-up with health verification, for the R1 Lite
# onboard PC. Usage:
#     vendor_up.sh          bring everything up, verify, report PASS/FAIL
#     vendor_up.sh status   verify only, change nothing
#     vendor_up.sh stop     tear the vendor stack down, verify it died
#
# Design record: FDEIssues/galaxea-integration/VENDOR_BRINGUP_PLAN.md.
# Field history (2026-07-28): the vendor tmux boot reports success
# unconditionally while its HDAS pane can die silently; the fallback that
# always worked is an HDAS launch with ROS_DOMAIN_ID exported, held in a
# session of our own. Health truth is message rates, not process
# existence. Never two HDAS instances (duplicate CAN publishers).

STARTUP_ROOT="$HOME/galaxea/install/startup_config/share/startup_config"
STARTUP_SCRIPT_DIR="$STARTUP_ROOT/script"
SESSION_CFG="../sessions.d/ATCStandard/R1LITEBody.d"
HDAS_DIR="$STARTUP_ROOT/script/boot/modules/hdas"
HDAS_SCRIPT="./start_hdas_r1lite.sh"
FALLBACK_SESSION="hdas_fg"
DOMAIN="${ROS_DOMAIN_ID:-2}"

ARM_TOPIC="/hdas/feedback_arm_left"
ARM_MIN_HZ=150
CHASSIS_TOPIC="/motion_control/chassis_speed"
CHASSIS_MIN_HZ=25
HZ_WINDOW_S=10

LOG="/tmp/vendor_up_$(date +%Y%m%d_%H%M%S).log"

say()  { echo "[vendor_up] $*" | tee -a "$LOG"; }
fail() { say "FAIL: $*"; say "log: $LOG"; exit 1; }

require_robot() {
    [ -d "$STARTUP_SCRIPT_DIR" ] \
        || fail "no Galaxea install at $STARTUP_SCRIPT_DIR — run this ON the robot"
}

ros_env() {
    export ROS_DOMAIN_ID="$DOMAIN"
    # shellcheck disable=SC1091
    source "$HOME/galaxea/install/setup.bash" 2>/dev/null \
        || source /opt/ros/humble/setup.bash
}

# Measure a topic's rate over a fixed window; prints the average Hz
# (integer, 0 when silent). The measurement process is ended with SIGINT
# so rclpy destroys its DDS participant cleanly — a TERM-killed reader
# can leak into a vendor stack with static allocation, the exact failure
# this script exists to guard against.
measure_hz() {
    local topic="$1"
    timeout --signal=INT --kill-after=5 "$HZ_WINDOW_S" \
        ros2 topic hz --window 500 "$topic" 2>/dev/null \
        | grep -o 'average rate: [0-9.]*' | tail -1 | grep -o '[0-9.]*' | cut -d. -f1
}

hdas_pids() { pgrep -f "$HDAS_SCRIPT|hdas_r1lite" 2>/dev/null; }
hdas_count() { hdas_pids | wc -l; }

kill_gello() {
    if tmux has-session -t r1lite_teleop 2>/dev/null; then
        tmux kill-session -t r1lite_teleop 2>/dev/null || true
        say "killed the factory GELLO teleop session (it silently grabs the arms)"
    fi
    # The vendor's own teleop IK co-publishes on the arm target topics
    # (fails preflight's sole-writer gate; found on-site 2026-07-24 and
    # again 2026-08-11 — every vendor boot resurrects it).
    if pgrep -f relaxed_ik >/dev/null 2>&1; then
        pkill -f relaxed_ik || true
        say "killed vendor relaxed_ik nodes (they co-publish arm targets)"
    fi
}

host_prereqs() {
    # Idempotent; both are required for loopback DDS and reset on reboot.
    if ! ip link show lo | grep -q MULTICAST; then
        say "enabling multicast on lo (sudo)"
        sudo ip link set lo multicast on || fail "could not enable lo multicast"
    fi
    if ! ip route show | grep -q '^224\.0\.0\.0/4 '; then
        say "adding 224.0.0.0/4 route via lo (sudo)"
        sudo ip route add 224.0.0.0/4 dev lo 2>/dev/null \
            || say "route add returned nonzero (already present is fine)"
    fi
}

gate_report() {
    # Verifies the full contract; prints measured numbers; returns nonzero
    # with the first failing gate named.
    local n
    n=$(hdas_count)
    if [ "$n" -eq 0 ]; then
        say "GATE FAIL: no HDAS process"; return 1
    elif [ "$n" -gt 1 ]; then
        say "GATE FAIL: $n HDAS processes (duplicate CAN publishers!) — pids: $(hdas_pids | tr '\n' ' ')"
        return 1
    fi
    say "HDAS process: exactly one"
    local arm_hz chassis_hz
    arm_hz=$(measure_hz "$ARM_TOPIC"); arm_hz=${arm_hz:-0}
    say "$ARM_TOPIC: ${arm_hz} Hz (need >= $ARM_MIN_HZ)"
    [ "$arm_hz" -ge "$ARM_MIN_HZ" ] || { say "GATE FAIL: arm feedback below threshold"; return 1; }
    chassis_hz=$(measure_hz "$CHASSIS_TOPIC"); chassis_hz=${chassis_hz:-0}
    say "$CHASSIS_TOPIC: ${chassis_hz} Hz (need >= $CHASSIS_MIN_HZ)"
    [ "$chassis_hz" -ge "$CHASSIS_MIN_HZ" ] || { say "GATE FAIL: chassis feedback below threshold"; return 1; }
    if tmux has-session -t r1lite_teleop 2>/dev/null; then
        say "GATE FAIL: factory GELLO session is alive and holds the arms"; return 1
    fi
    say "GELLO: absent"
    return 0
}

hdas_fallback() {
    # The field workaround, formalized: kill every HDAS path, then hold a
    # single foreground HDAS in our own tmux session with the domain baked
    # into the pane command (a plain background job dies with SSH).
    say "HDAS fallback: killing tmux hdas session and any strays"
    tmux kill-session -t hdas 2>/dev/null || true
    tmux kill-session -t "$FALLBACK_SESSION" 2>/dev/null || true
    local p
    for p in $(hdas_pids); do kill "$p" 2>/dev/null; done
    sleep 2
    [ "$(hdas_count)" -eq 0 ] || fail "could not clear existing HDAS processes"
    say "launching HDAS in tmux session '$FALLBACK_SESSION' (domain $DOMAIN)"
    tmux new-session -d -s "$FALLBACK_SESSION" \
        "export ROS_DOMAIN_ID=$DOMAIN; cd '$HDAS_DIR' && $HDAS_SCRIPT"
    sleep 12
    [ "$(hdas_count)" -ge 1 ] || fail "fallback HDAS did not start (tmux attach -t $FALLBACK_SESSION)"
}

do_status() {
    require_robot; ros_env
    say "status check (domain $DOMAIN, window ${HZ_WINDOW_S}s per topic)"
    if gate_report; then say "PASS: vendor stack healthy"; else
        say "tmux sessions:"; tmux ls 2>&1 | tee -a "$LOG"
        fail "vendor stack NOT healthy — run: vendor_up.sh"
    fi
}

do_stop() {
    require_robot
    say "stopping vendor stack"
    ( cd "$STARTUP_SCRIPT_DIR" && ./robot_startup.sh kill ) 2>&1 | tee -a "$LOG"
    tmux kill-session -t "$FALLBACK_SESSION" 2>/dev/null || true
    sleep 3
    local p
    for p in $(hdas_pids); do kill "$p" 2>/dev/null; done
    sleep 2
    [ "$(hdas_count)" -eq 0 ] || fail "HDAS still running after stop: $(hdas_pids | tr '\n' ' ')"
    say "PASS: vendor stack down (no HDAS process)"
}

do_up() {
    require_robot; ros_env; host_prereqs
    if gate_report; then
        kill_gello
        say "PASS: vendor stack already healthy (idempotent no-op)"
        return 0
    fi
    if [ "$(hdas_count)" -gt 1 ]; then
        say "clearing duplicate HDAS before anything else"
        hdas_fallback
    elif ! tmux ls 2>/dev/null | grep -q .; then
        say "no tmux server: full vendor boot. ARMS AND GRIPPERS WILL"
        say "TWITCH — robot clear, e-stop in reach."
        ( cd "$STARTUP_SCRIPT_DIR" && ./robot_startup.sh boot "$SESSION_CFG" ) 2>&1 | tee -a "$LOG"
        # Poll for HDAS liveness instead of the old blind 45s sleep: pass
        # as soon as a process exists (rates still gated below), fail the
        # wait no later than the old fixed delay.
        local waited=0
        while [ "$waited" -lt 45 ] && [ "$(hdas_count)" -eq 0 ]; do
            sleep 5; waited=$((waited + 5))
        done
        say "boot settle: HDAS appeared after ~${waited}s (0 processes means the tmux flake)"
        sleep 5
    fi
    kill_gello
    if gate_report; then say "PASS: vendor stack healthy"; return 0; fi
    say "boot path unhealthy — taking the HDAS fallback (the 07-28 field workaround)"
    hdas_fallback
    kill_gello
    if gate_report; then
        say "PASS: vendor stack healthy (tmux boot + foreground-HDAS hybrid)"
        return 0
    fi
    say "tmux sessions:"; tmux ls 2>&1 | tee -a "$LOG"
    fail "still unhealthy after fallback — inspect: tmux attach -t $FALLBACK_SESSION; log: $LOG"
}

case "${1:-up}" in
    up)      do_up ;;
    status)  do_status ;;
    stop)    do_stop ;;
    *)       echo "usage: vendor_up.sh [up|status|stop]" >&2; exit 2 ;;
esac
