#!/bin/bash
# m20_shadow on AOS: the Rust controller in dry-run beside rl_deploy (it never sends; see src/shadow/mod.rs).
# Temporary units in /run/systemd/system (gone after a reboot), CPUs 1-3 at nice 10, a guard that stops it if
# rl_deploy's "send Data update timeout" count rises faster than its background rate.
#
#   install.sh install DIR   copy DIR's m20_shadow, m20_controller.toml, shadow.toml, policies/ into place, add units
#   install.sh start | stop | status | uninstall
#   stop by hand:            sudo systemctl stop m20-shadow-guard m20-shadow
set -euo pipefail
home=/var/opt/robot/data/m20_shadow
units=/run/systemd/system
here="$(cd "$(dirname "$0")" && pwd)"
case "${1:-status}" in
install)
    src="${2:?install DIR}"
    mkdir -p "$home/bin" "$home/policies" "$home/log"
    install -m 755 "$src/m20_shadow" "$home/bin/m20_shadow"
    install -m 644 "$src/m20_controller.toml" "$src/shadow.toml" "$home/"
    cp "$src"/policies/*.onnx "$home/policies/"
    install -m 755 "$here/guard.sh" "$home/bin/guard.sh"
    install -m 644 "$here/m20-shadow.service" "$here/m20-shadow-guard.service" "$units/"
    systemctl daemon-reload
    echo "installed into $home; start with: $0 start"
    ;;
start)
    systemctl start m20-shadow m20-shadow-guard
    sleep 2
    systemctl --no-pager status m20-shadow | head -12
    ;;
stop)
    systemctl stop m20-shadow-guard m20-shadow
    ;;
status)
    systemctl --no-pager status m20-shadow m20-shadow-guard 2>/dev/null | grep -E "●|Active|Main PID" || true
    journalctl -u m20-shadow -u m20-shadow-guard -n 8 --no-pager -o cat 2>/dev/null || true
    du -sh "$home/log" 2>/dev/null || true
    ;;
uninstall)
    systemctl stop m20-shadow-guard m20-shadow 2>/dev/null || true
    rm -f "$units/m20-shadow.service" "$units/m20-shadow-guard.service"
    systemctl daemon-reload
    echo "units removed; logs stay in $home/log"
    ;;
*)
    echo "usage: $0 install DIR | start | stop | status | uninstall" >&2
    exit 2
    ;;
esac
