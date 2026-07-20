#!/bin/bash
# Bootstrap verifier (Final plan Step 0): one-time INSTALLATION verification,
# machine-readable. This is not the runtime checker - runtime readiness calls
# MulticastConfiguratorLinux.check() / BufferConfiguratorLinux.check() inside
# the container. Nothing here repairs anything; the systemd units repair.
#
# Output: one KEY=OK|FAIL line per check on stdout. Exit 0 iff all OK.

fail=0
report() {
    if [ "$2" = "0" ]; then echo "$1=OK"; else echo "$1=FAIL"; fail=1; fi
}

ip link show lo | grep -q MULTICAST
report LO_MULTICAST $?

ip route show | grep -q '^224.0.0.0/4 dev lo'
report LO_MCAST_ROUTE $?

[ "$(sysctl -n net.core.rmem_max 2>/dev/null)" -ge 67108864 ] 2>/dev/null
report RMEM_MAX $?

[ "$(sysctl -n net.core.rmem_default 2>/dev/null)" -ge 67108864 ] 2>/dev/null
report RMEM_DEFAULT $?

command -v docker >/dev/null && docker info >/dev/null 2>&1
report DOCKER_USABLE $?

systemctl is-enabled dimos-hostnet.service >/dev/null 2>&1
report HOSTNET_UNIT_ENABLED $?

systemctl is-active dimos-hostnet.service >/dev/null 2>&1
report HOSTNET_UNIT_ACTIVE $?

systemctl is-enabled dimos-r1lite.service >/dev/null 2>&1
report R1LITE_UNIT_ENABLED $?

exit $fail
