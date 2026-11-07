#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Make this board (GOS) the M20's LAN time master: chrony serves the wired 10.21.31.0/24 net, at a usable stratum even
# when the upstream internet NTP pools are unreachable, so the other boards never free-run. GOS tracks real UTC from
# its own pools when it can. Restarting chrony resets the step counter so the stock `makestep` steps GOS's own initial
# drift once (the boards had drifted a second or two and the stock counter was spent, so it only slewed); then slews.
# Idempotent (re-running replaces the block). Run with sudo.
set -euo pipefail
conf=/etc/chrony/chrony.conf
sed -i '/# dimos-m20-clock-master BEGIN/,/# dimos-m20-clock-master END/d' "$conf"
cat >>"$conf" <<'BLOCK'
# dimos-m20-clock-master BEGIN
allow 10.21.31.0/24
local stratum 10
# dimos-m20-clock-master END
BLOCK
systemctl restart chrony 2>/dev/null || systemctl restart chronyd
# Converge now: arm a step on each of the next several updates so GOS's own initial drift is stepped out at once
# (the stock step counter is spent by boot, so otherwise it would only slew); once converged these are no-ops.
for _ in $(seq 1 8); do chronyc -a makestep >/dev/null 2>&1 || chronyc makestep >/dev/null 2>&1 || true; sleep 1; done
chronyc tracking | grep -E 'Reference ID|System time|Last offset' || true
