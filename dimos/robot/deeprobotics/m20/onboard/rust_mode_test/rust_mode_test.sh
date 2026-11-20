# rust_mode_test.sh: one rust-mode test on the M20, run on GOS as user.
# Copy this directory's files to GOS /tmp first, and install_dimos_host.sh as /tmp/install_dimos_host_rustmode.sh: the
# test switches GOS's ~/dimos checkout, so nothing it runs may live in it. Push the branch under test to GOS as $BRANCH. Every vendor DrDDS service is STOPPED (never
# disabled or masked, so a reboot or the revert brings vendor mode back), m20 runs with the bus owner (m20_ecan
# owner mode: rl_deploy's startup, then zero-gain sitting commands, battery, charge enable), it is measured, and the
# full vendor stack comes back in boot order. The revert runs on exit, also on failure or Ctrl-C.
set -u

# --- config ---
RUN_SECS=330                 # m20 in rust mode at least this long (5 min + margin)
CHECK_EVERY_S=30             # health check period while it runs
TEMP_ABORT_C=85              # any board's hottest thermal zone above this ends the test (revert)
BRANCH=m20_rust_rustmode     # jeff/feat/m20_rust pushed here before the run
LAUNCH=/tmp/launch_m20_rust_rustmode.json
VENDOR_LAUNCH=/tmp/launch3.json
SAVED_PATCH=~/gos_revert_20261120/uncommitted.patch
SLAM_BIN=/var/opt/robot/data/user_home/e1_slam/target_mod/release/dim_slam_lio
AIRY_BIN=/tmp/airy_native_body_filter
SNIFF_BIN=/tmp/m20_body_imu_sniff
IMU_BIN=/tmp/m20_imu
ECAN_BIN=/tmp/m20_ecan
STAGE=/var/opt/robot/data/dimos_host
OUT=/tmp/m20_rustmode_test
PW="'"
AOS=10.21.31.103
NOS=10.21.31.106
S=~/.dimos/desktop/sockets/dimos-server.sock

# Every vendor DrDDS unit, in rust mode's stop order (m20tool modes.ts VENDOR_UNITS, f5a6234): what commands the legs
# first, then rl_deploy, then the NOS bridge and the SLAM bound to it, then the drivers. Start order is the reverse.
# ctrlmcu (AOS) is left alone: it crash-loops on the vendor's IP guard (AOS's tailscale address), a fix pending Jeff.
LEFT_ALONE=ctrlmcu_node
UNITS=(
    "NOS charge_manager.service"
    "NOS reflective_column.service"
    "NOS handler.service"
    "AOS basic_server.service"
    "AOS rl_deploy.service"
    "NOS drdds-zenoh-bridge.service"
    "NOS localization.service"
    "NOS rsdriver.service"
    "NOS yesense.service"
    "NOS cpunode.service"
    "AOS height_map_nav.service"
    "AOS lio_perception.service"
    "AOS rsdriver.service"
    "AOS yesense.service"
    "AOS gps_node.service"
    "AOS cpunode.service"
    "GOS cpunode.service"
    "GOS encryption_node.service"
)

mkdir -p $OUT
api() { curl -s --unix-socket $S "$@"; }
log() { echo "$(date +%H:%M:%S) $*" | tee -a $OUT/steps.log; }
ip_of() { case $1 in AOS) echo $AOS ;; NOS) echo $NOS ;; GOS) echo local ;; esac; }
# run a command on a board, as root when $3 is "root"
on() {
    local cmd="$2"
    [ "${3:-}" = root ] && cmd="echo \"$PW\" | sudo -S -p '' bash -c $(printf '%q' "$2")"
    if [ "$1" = GOS ]; then bash -c "$cmd"; else ssh -o ConnectTimeout=10 "$(ip_of $1)" "$cmd"; fi
}
run_id() { api http://x/active | python3 -c 'import json,sys; print(json.load(sys.stdin).get("runId"))'; }
# every process on a board that maps a DrDDS / Fast-DDS library, with its unit
dds_procs() {
    on $1 'for p in $(ls /proc | grep -E "^[0-9]+$"); do grep -qsE "drdds|fastrtps|fastdds" /proc/$p/maps && echo "$(cat /proc/$p/comm) $(grep -o "[^/]*\.service" /proc/$p/cgroup | head -1)"; done | sort | uniq -c' root
}
hottest() { on $1 'm=0; for z in /sys/class/thermal/thermal_zone*/temp; do t=$(($(cat $z)/1000)); [ $t -gt $m ] && m=$t; done; echo "$m C, fan pwm $(cat /sys/class/hwmon/hwmon7/pwm1)"'; }

STOPPED=()          # units this test stopped, in stop order
SWITCHED_ROUTER=0
SHADOW_WAS_ACTIVE=0
STASHED=0

revert() {
    trap - EXIT INT TERM
    log "=== REVERT"
    # 1. the bus owner and everything else of m20 stop first, so rl_deploy never shares the bus
    api -X POST http://x/active/stop > /dev/null
    for i in $(seq 1 30); do on AOS 'pgrep -x m20_ecan > /dev/null' || break; sleep 1; done
    on AOS 'pgrep -x m20_ecan > /dev/null' && { log "m20_ecan still running: killing it before rl_deploy starts"; on AOS 'pkill -9 -x m20_ecan' root; sleep 1; }
    log "m20_ecan on AOS: $(on AOS 'pgrep -x m20_ecan || echo none')"
    on GOS 'systemctl disable --now dimos-host.service > /dev/null 2>&1' root
    # 2. the checkout as it was, before anything vendor starts reading it
    cd ~/dimos
    [ "$(git rev-parse --abbrev-ref HEAD)" = jeff/deploy/m20_continuous_reloc ] || git checkout -q jeff/deploy/m20_continuous_reloc
    [ $STASHED = 0 ] || git stash pop -q
    log "checkout $(git rev-parse --short HEAD) patch $(git diff --binary HEAD | md5sum | cut -c1-12) (saved $(md5sum < $SAVED_PATCH | cut -c1-12))"
    # 3. the vendor units in boot order (the reverse of the stop order); the bridge's :7447 back before it starts
    for (( i=${#STOPPED[@]}-1; i>=0; i-- )); do
        read -r board unit <<< "${STOPPED[$i]}"
        if [ "$unit" = drdds-zenoh-bridge.service ] && [ $SWITCHED_ROUTER = 1 ]; then
            on NOS 'systemctl stop dimos-host-router.service' root
            log "NOS dimos-host-router: $(on NOS 'systemctl is-active dimos-host-router.service')"
        fi
        on $board "systemctl start $unit" root
        log "start $board $unit: $(on $board "systemctl is-active $unit")"
        if [ "$unit" = drdds-zenoh-bridge.service ] && [ $SWITCHED_ROUTER = 1 ]; then
            on NOS 'systemctl start dimos-host.service' root
            log "NOS dimos-host: $(on NOS 'systemctl is-active dimos-host.service')"
        fi
        if [ "$unit" = rl_deploy.service ]; then
            # run.sh starts the rl_deploy process a few seconds after the unit
            for j in $(seq 1 30); do on AOS 'pgrep -x rl_deploy > /dev/null' && break; sleep 1; done
            sleep 2
            log "rl_deploy pid $(on AOS 'pgrep -x rl_deploy') shim $(on AOS 'grep -c ecan_tap /proc/$(pgrep -x rl_deploy)/maps' root) lines"
        fi
    done
    # 4. the boards' hosts back on the vendor checkout's revision
    DIMOS_CHECKOUT=$HOME/dimos bash /tmp/install_dimos_host_rustmode.sh aos nos > $OUT/install_revert.log 2>&1
    log "board hosts: $(grep -c 'dimos-host.*: active' $OUT/install_revert.log) active"
    if true; then
        on AOS 'systemctl start m20-shadow.service m20-shadow-guard.service' root
        log "shadow: $(on AOS 'systemctl is-active m20-shadow.service m20-shadow-guard.service' | tr '\n' ' ')"
    fi
    # 5. the vendor run, and the checks that vendor mode is whole
    api -X POST http://x/active/exec -H "Content-Type: application/json" -d @$VENDOR_LAUNCH > /dev/null
    for i in $(seq 1 60); do api http://x/active | grep -q '"phase":"running"' && break; sleep 3; done
    log "vendor run $(run_id)"
    sleep 20
    # basic_server comes back in regular usage mode; the dimos cmd_vel / Controller path needs navigation (1).
    # Type 1101 / Command 5 only picks basic_server's input source, it moves nothing.
    log "usage mode -> navigation: $(python3 /tmp/set_usage_mode.py 1 2>&1)"
    sleep 2
    log "status: $(m20tool status 2>&1 | head -2 | tr '\n' '|')"
    m20tool status 2>&1 | grep -q 'usage mode: navigation (1)' || log "WARNING: usage mode is not navigation"
    log "MotionInfo on the bridge: $(cd ~/dimos && timeout 30 .venv/bin/python /tmp/count_topics.py dimos 10 tcp/10.21.31.106:7447 2>&1 | grep -E 'motion_state|gait_state|slam_odom|battery' | tr -s ' ' | tr '\n' '|')"
    log "e-stop: $(on AOS 'systemctl is-active m20-imu-estop.service')"
    for u in "${UNITS[@]}"; do read -r board unit <<< "$u"; st=$(on $board "systemctl is-active $unit"); [ "$st" = active ] || log "WARNING: $board $unit is $st after the revert"; done
    for b in GOS AOS NOS; do log "DDS processes on $b: $(dds_procs $b | wc -l)"; done
    log "=== REVERT DONE"
}

# --- pre-checks (nothing changed yet) ---
log "=== PRE-CHECK"
cd ~/dimos
[ "$(git rev-parse --abbrev-ref HEAD)" = jeff/deploy/m20_continuous_reloc ] || { log "unexpected branch, abort"; exit 1; }
[ "$(git diff --binary HEAD | md5sum)" = "$(md5sum < $SAVED_PATCH)" ] || { log "patch differs from the saved one, abort"; exit 1; }
status=$(m20tool status 2>&1); echo "$status" > $OUT/status_before.txt
log "status: $(echo "$status" | head -2 | tr '\n' '|')"
echo "$status" | grep -qE '^motion: (idle|damping|sit|lying|power-on damping)' || { log "the dog isn't down, abort"; exit 1; }
echo "$status" | grep -qE 'charge: (charging|on dock)' || { log "the dog isn't docked, abort"; exit 1; }
[ "$(on AOS 'systemctl is-active rl_deploy.service')" = active ] || { log "rl_deploy not active, abort"; exit 1; }
log "rl_deploy pid $(on AOS 'pgrep -x rl_deploy'), e-stop $(on AOS 'systemctl is-active m20-imu-estop.service')"
[ "$(on AOS 'systemctl is-active m20-imu-estop.service')" = active ] || { log "the IMU e-stop isn't armed, abort"; exit 1; }
for b in GOS AOS NOS; do dds_procs $b > $OUT/dds_before_$b.txt; log "DDS processes on $b: $(wc -l < $OUT/dds_before_$b.txt)"; log "temp $b: $(hottest $b)"; done
for u in "${UNITS[@]}"; do read -r board unit <<< "$u"; [ "$(on $board "systemctl is-active $unit")" = active ] || { log "$board $unit is not active before the test, abort"; exit 1; }; done
log "all ${#UNITS[@]} vendor units active"

trap revert EXIT
trap 'exit 130' INT TERM

# --- the e-stop also kills the bus owner on a trip (the motors then fall passive) ---
on AOS "python3 - <<'PY'
import json
p = '/var/opt/robot/data/imu_estop/config.json'
c = json.load(open(p))
names = c.setdefault('kill_process_names', [])
if 'm20_ecan' not in names:
    names.append('m20_ecan')
json.dump(c, open(p, 'w'))
PY
systemctl restart m20-imu-estop.service" root
log "e-stop: $(on AOS 'systemctl is-active m20-imu-estop.service'), config $(on AOS 'cat /var/opt/robot/data/imu_estop/config.json')"

# --- stop shadow, the vendor run; install m20 (vendor still up) ---
[ "$(on AOS 'systemctl is-active m20-shadow.service')" = active ] && SHADOW_WAS_ACTIVE=1
on AOS 'systemctl stop m20-shadow-guard.service m20-shadow.service' root
log "shadow: $(on AOS 'systemctl is-active m20-shadow.service m20-shadow-guard.service' | tr '\n' ' ')"
log "stop vendor run $(run_id)"
api -X POST http://x/active/stop > /dev/null
git stash push -u -q -m "gos-revert-20261120 rustmode" && STASHED=1
GIT_LFS_SKIP_SMUDGE=1 git -c advice.detachedHead=false checkout -q $BRANCH
place() { mkdir -p "$1/$2" && install -m 755 "$3" "$1/$2/$4"; }
for root in "$HOME/dimos" $STAGE; do
    place $root dimos/mapping/dim_slam_lio/rust/result/bin $SLAM_BIN dim_slam_lio
    place $root dimos/hardware/sensors/lidar/robosense/rust/result/bin $AIRY_BIN airy_native
    place $root dimos/robot/deeprobotics/m20/body_imu_sniff/rust/result/bin $SNIFF_BIN m20_body_imu_sniff
    place $root dimos/robot/deeprobotics/m20/imu/rust/result/bin $IMU_BIN m20_imu
    place $root dimos/robot/deeprobotics/m20/ecan/rust/result/bin $ECAN_BIN m20_ecan
done
log "checkout $(git log -1 --oneline | cut -c1-60)"
# while the bridge runs, so NOS's enabled Host unit stays the vendor-mode one (a reboot comes back as vendor mode)
bash dimos/robot/deeprobotics/m20/onboard/install_dimos_host.sh gos aos nos > $OUT/install.log 2>&1
log "hosts installed: $(grep -c 'dimos-host.*: active' $OUT/install.log) active, $(grep 'staged revision' $OUT/install.log)"

# --- rust mode: stop (never disable) every vendor unit, in order ---
log "=== RUST MODE"
for u in "${UNITS[@]}"; do
    read -r board unit <<< "$u"
    on $board "systemctl stop $unit" root
    STOPPED+=("$u")
    log "stop $board $unit: $(on $board "systemctl is-active $unit")"
    if [ "$unit" = drdds-zenoh-bridge.service ]; then
        # NOS's Host takes over the robot's router on :7447 (Conflicts= stops the vendor-mode Host unit)
        on NOS 'systemctl start dimos-host-router.service' root
        SWITCHED_ROUTER=1
        log "NOS dimos-host-router: $(on NOS 'systemctl is-active dimos-host-router.service'), dimos-host: $(on NOS 'systemctl is-active dimos-host.service')"
    fi
done
for b in GOS AOS NOS; do dds_procs $b > $OUT/dds_rust_$b.txt; log "DDS processes on $b in rust mode: $(wc -l < $OUT/dds_rust_$b.txt)"; done
grep -hv "$LEFT_ALONE" $OUT/dds_rust_*.txt | grep . && { log "a DrDDS process is still running, abort"; exit 1; }
log "left alone: $(grep -h "$LEFT_ALONE" $OUT/dds_rust_*.txt | tr -s ' ' || true) (ctrlmcu crash-looping, not part of this test)"
rev=$(git rev-parse --short=10 HEAD)
for i in $(seq 1 40); do [ "$(ZENOH_CONNECT=tcp/127.0.0.1:7447 timeout 30 .venv/bin/dimos host ls 2>/dev/null | grep -E '^(aos|gos|nos) ' | grep -c $rev)" = 3 ] && break; sleep 3; done
log "hosts: $(ZENOH_CONNECT=tcp/127.0.0.1:7447 timeout 30 .venv/bin/dimos host ls 2>/dev/null | grep -E '^(aos|gos|nos) ' | awk '{print $1}' | tr '\n' ' ')"

# --- m20 with the bus owner ---
api -X POST http://x/active/exec -H "Content-Type: application/json" -d @$LAUNCH > /dev/null
until api http://x/active | grep -q 'Running; Ctrl-C'; do
    api http://x/active | grep -q '"phase":"\(failed\|stopped\)"' && { log "m20 failed to start"; api http://x/active > $OUT/failed.json; exit 1; }
    sleep 3
done
R=$(run_id); echo $R > $OUT/run_id
log "m20 running $R; m20_ecan pid $(on AOS 'pgrep -x m20_ecan')"
started=$(date +%s); next_measure=60; measured_early=0
while [ $(( $(date +%s) - started )) -lt $RUN_SECS ]; do
    sleep $CHECK_EVERY_S
    on AOS 'pgrep -x m20_ecan > /dev/null' || { log "m20_ecan exited (its guard, or a crash): end the test"; exit 1; }
    # the bus owner runs in AOS's Host, whose fragment log (not the root's on GOS) carries its status and guard lines.
    # Its own guard exits on torque / speed / a status other than 1 after startup (caught above); this also catches a
    # non-1 status it logged after startup, and any guard line
    owner=$(on AOS "python3 - /var/opt/robot/data/dimos_host/logs/$R/main.jsonl" <<'PY'
import json, sys
bad = []
try:
    lines = open(sys.argv[1])
except OSError as error:
    print(f"no bus owner log: {error}")
    sys.exit()
for line in lines:
    try:
        e = json.loads(line)
    except ValueError:
        continue
    if "M20Ecan" not in str(e.get("module", "")):
        continue
    ev = str(e.get("event", ""))
    if ev == "joint status words" and str(e.get("startup", "")).lower() not in ("true", "1") and str(e.get("statuses")) != "{1}":
        bad.append(f"status {e.get('statuses')}")
    if ev.startswith("guard"):
        bad.append(ev)
print("; ".join(bad))
PY
)
    [ -z "$owner" ] || { log "bus owner: $owner: end the test"; exit 1; }
    cams=$(timeout 20 ./.venv/bin/python /tmp/count_topics.py dimos 3 tcp/10.21.31.106:7447 2>/dev/null | grep -cE "(front|rear)_camera/foxglove")
    log "bus owner on AOS: alive, statuses 1, no guard line; cameras streaming on NOS's router: $cams of 2"
    [ "$cams" = 2 ] || { log "camera frames stopped (link: $(on AOS 'ss -tn state established | grep -E "10.21.33.106:744[78]" | awk "{print \$4}" | tr "\n" " "')): end the test"; exit 1; }
    api http://x/active | grep -q '"running":true' || { log "m20 stopped: end the test"; exit 1; }
    for b in GOS AOS NOS; do
        t=$(hottest $b); log "temp $b: $t"
        [ "${t%% *}" -lt $TEMP_ABORT_C ] || { log "$b at ${t%% *} C >= $TEMP_ABORT_C C: end the test"; exit 1; }
    done
    if [ $measured_early = 0 ] && [ $(( $(date +%s) - started )) -ge $next_measure ]; then
        measured_early=1
        log "measure early"
        ./.venv/bin/python /tmp/latency.py tcp/127.0.0.1:7447 20 \
            dimos-domains/$R/lidar/sensor_msgs.PointCloud2=cloud dimos-domains/$R/lidar_imu/sensor_msgs.Imu=imu \
            dimos-domains/$R/imu/sensor_msgs.Imu=imu dimos-domains/$R/slam_odom/nav_msgs.Odometry=odom \
            dimos-domains/$R/coordinator_joint_state/sensor_msgs.JointState=joint \
            dimos-domains/$R/battery_left/sensor_msgs.BatteryState=battery dimos-domains/$R/battery_right/sensor_msgs.BatteryState=battery \
            > $OUT/latency_1.txt 2>&1
        bash /tmp/cpu_snap.sh rustmode > $OUT/cpu.txt 2>&1
    fi
done
log "measure late"
./.venv/bin/python /tmp/latency.py tcp/127.0.0.1:7447 20 \
    dimos-domains/$R/lidar/sensor_msgs.PointCloud2=cloud dimos-domains/$R/lidar_imu/sensor_msgs.Imu=imu \
    dimos-domains/$R/imu/sensor_msgs.Imu=imu dimos-domains/$R/slam_odom/nav_msgs.Odometry=odom \
    dimos-domains/$R/coordinator_joint_state/sensor_msgs.JointState=joint \
    dimos-domains/$R/battery_left/sensor_msgs.BatteryState=battery dimos-domains/$R/battery_right/sensor_msgs.BatteryState=battery \
    > $OUT/latency_2.txt 2>&1
# the cameras: m20-camera.service on AOS, through NOS's router (its 7448 fallback now the bridge is down)
./.venv/bin/python /tmp/latency.py tcp/10.21.31.106:7447 10 dimos/front_camera/foxglove_msgs.CompressedVideo=video dimos/rear_camera/foxglove_msgs.CompressedVideo=video > $OUT/cameras.txt 2>&1
log "camera link: $(on AOS 'ss -tn state established | grep -E "10.21.33.106:744[78]" | awk "{print \$4}" | tr "\n" " "')"
./.venv/bin/python /tmp/verify_run.py $R 20 > $OUT/verify.txt 2>&1
bash /tmp/cpu_snap.sh rustmode-late > $OUT/cpu_late.txt 2>&1
for b in GOS AOS NOS; do dds_procs $b > $OUT/dds_rust_late_$b.txt; log "DDS processes on $b late: $(wc -l < $OUT/dds_rust_late_$b.txt)"; done
cp /var/opt/robot/data/dimos/logs/$R/main.jsonl $OUT/main.jsonl 2>/dev/null
log "measured"
