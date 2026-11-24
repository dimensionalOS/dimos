# vendor_on.sh (GOS, as user, from m20_rust_revert/): pure-Rust m20 -> vendor stack. Stopping the m20-controller service
# lies a standing robot down first (its SIGTERM path); this logs the state before and after and stops if it fails.
source /var/opt/robot/data/user_home/m20_rust_revert/common.sh
cd ~/dimos
log "=== VENDOR ON"
dimos_up && { .venv/bin/dimos stop > /dev/null 2>&1; log "dimos run stopped (the service keeps the robot as it is)"; }
if [ "$(on AOS 'systemctl is-active m20-controller.service')" = active ]; then
    log "m20-controller before stop: $(on AOS 'journalctl -u m20-controller -n 400 --no-pager -o cat | grep "controller status" | tail -1')"
    on AOS 'systemctl stop m20-controller.service' root
    log "m20-controller after stop: $(on AOS 'journalctl -u m20-controller -n 400 --no-pager -o cat | grep -E "controller status|policy loop" | tail -2 | tr "\n" " "')"
    [ "$(on AOS 'systemctl is-active m20-controller.service')" = inactive ] || { log "m20-controller did not stop, abort (vendor units left down)"; exit 1; }
fi
for i in $(seq 1 30); do [ -z "$(ctl)" ] && break; sleep 1; done
[ -n "$(ctl)" ] && { log "a controller outside the service is up (dog may drop): SIGKILL"; on AOS "kill -9 $(ctl | tr '\n' ' ')" root; sleep 1; }
for (( i=${#UNITS[@]}-1; i>=0; i-- )); do
    read -r board unit <<< "${UNITS[$i]}"
    [ "$unit" = drdds-zenoh-bridge.service ] && on NOS 'systemctl stop dimos-host-router.service' root
    on $board "systemctl start $unit" root
    [ "$unit" = drdds-zenoh-bridge.service ] && on NOS 'systemctl start dimos-host.service' root
    if [ "$unit" = rl_deploy.service ]; then
        for j in $(seq 1 30); do on AOS 'pgrep -x rl_deploy > /dev/null' && break; sleep 1; done
    fi
done
on AOS 'systemctl start m20-camera.service' root
bad=""; for u in "${UNITS[@]}"; do read -r board unit <<< "$u"; [ "$(on $board "systemctl is-active $unit")" = active ] || bad="$bad $board:$unit"; done
log "=== VENDOR ON done; rl_deploy pid $(on AOS 'pgrep -x rl_deploy'); not active:${bad:- none}"
