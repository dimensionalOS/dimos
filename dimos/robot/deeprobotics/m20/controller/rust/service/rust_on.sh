# rust_on.sh [tokens] (GOS, m20_rust_revert/): vendor -> the m20-controller service + `dimos run m20`; dog down; allow-gains -> launch.json.
source /var/opt/robot/data/user_home/m20_rust_revert/common.sh
cd ~/dimos
log "=== RUST ON (service) $*"
dimos_up && { log "a dimos run is up (stop it first: dimos stop), abort"; exit 1; }
if on AOS 'pgrep -x rl_deploy > /dev/null'; then
    status=$(m20tool status 2>&1)
    echo "$status" | grep -qE '^motion: (idle|damping|sit|lying|power-on damping)' || { log "the dog isn't down ($(echo "$status" | grep '^motion')), abort"; exit 1; }
fi
git diff --quiet HEAD || { log "GOS ~/dimos has uncommitted changes (every host must run the same revision), abort"; exit 1; }
gains=""; run_args=()
for t in "$@"; do
    case "$t" in
        --m20controller.allow-gains=true) gains=--allow-gains ;;
        --m20controller.allow-gains=*) ;;
        *) run_args+=("$t") ;;
    esac
done

on AOS 'systemctl stop m20-shadow-guard.service m20-shadow.service m20-camera.service m20-cam.service m20-cmdvel.service m20-split-aos.service 2>/dev/null; true' root
for u in "${UNITS[@]}"; do
    read -r board unit <<< "$u"
    on $board "systemctl stop $unit" root
    if [ "$unit" = drdds-zenoh-bridge.service ]; then
        on NOS 'systemctl stop dimos-host.service; systemctl restart dimos-host-router.service' root
    fi
done
log "vendor units stopped; NOS router: $(on NOS 'systemctl is-active dimos-host-router.service')"
on AOS "cd /var/opt/robot/data/dimos_host && sudo -u user .venv/bin/python -m dimos.robot.deeprobotics.m20.controller.service --policy-dir /var/opt/robot/data/m20_shadow/policies $gains > /var/opt/robot/data/m20_controller/launch.json && systemctl restart m20-controller.service" root
log "m20-controller: $(on AOS 'systemctl is-active m20-controller.service') (gains ${gains:-off})"
on AOS 'systemctl restart dimos-host.service' root
sudo systemctl restart dimos-host.service
rev=$(git rev-parse --short=10 HEAD)
for i in $(seq 1 40); do [ "$(ZENOH_CONNECT=tcp/127.0.0.1:7447 timeout 20 .venv/bin/dimos host ls 2>/dev/null | grep -E '^(aos|gos|nos) ' | grep -c $rev)" = 3 ] && break; sleep 3; done
log "hosts: $(ZENOH_CONNECT=tcp/127.0.0.1:7447 timeout 20 .venv/bin/dimos host ls 2>/dev/null | grep -E '^(aos|gos|nos) ' | awk '{print $1, $3}' | tr '\n' ';')"

# the rust-mode launch with the controller as the service (its policy dir is the service's)
args=$(python3 -c "import json,shlex; a=json.load(open('$D/launch_rust_mode.json'))['args'][1:]; a=[x.replace('=controller','=service') if x.startswith(('M20Joints=','M20Walker=')) else x for x in a if not x.startswith('--m20controller.')]; print(shlex.join(a))")
log "launch: dimos run m20 $args ${run_args[*]:-}"
eval setsid nohup .venv/bin/dimos run m20 $args "${run_args[@]}" > $D/rust_run.out 2>&1 < /dev/null &
for i in $(seq 1 60); do dimos_up && [ -n "$(on AOS 'pgrep -x yesense_imu')" ] && break; sleep 2; done
sleep 3
dimos_up || { log "m20 did not start:"; tail -5 $D/rust_run.out; exit 1; }
log "=== RUST ON (service) done: $(.venv/bin/dimos status | grep 'Run ID')"
