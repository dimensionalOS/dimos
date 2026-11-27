#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Point this board's chrony at the M20's LAN time master (GOS) and track ONLY it. The robot's own internet path (office
# wifi behind the vendor board's NAT) is too jittery to be a good clock -- the public pools disagree by ~1 s and a
# reachable one out-ranks the master by stratum -- so the stock `pool`/`server` lines are commented out and this board
# follows the master over the clean wired LAN. A chrony restart resets the step counter, so the initial multi-second
# gap is stepped once and then only slewed (stable, no hunting). Idempotent; re-running replaces our block and leaves
# the stock lines commented (prefixed `#dimos-disabled`, so a revert is a search-and-replace away). Run with sudo.
#   sync-clock.sh <master-ip>
set -euo pipefail
master="${1:?usage: sync-clock.sh <master-ip>}"
conf=/etc/chrony/chrony.conf
# drop our previous block, then comment any stock internet time sources we haven't already (keep them for easy revert)
sed -i '/# dimos-m20-clock BEGIN/,/# dimos-m20-clock END/d' "$conf"
sed -i -E '/^#dimos-disabled /! s/^[[:space:]]*(pool|server|peer)[[:space:]]+/#dimos-disabled &/' "$conf"
cat >>"$conf" <<EOF2
# dimos-m20-clock BEGIN
server ${master} iburst prefer
# dimos-m20-clock END
EOF2
systemctl restart chrony 2>/dev/null || systemctl restart chronyd
# Converge now: arm a step on each of the next several updates so the initial (possibly multi-second) offset is
# stepped out at once rather than slewed away over hours -- and so an offset that only appears once the master settles
# is still caught. iburst makes those updates arrive within a couple of seconds; once converged these are no-ops.
for _ in $(seq 1 8); do chronyc -a makestep >/dev/null 2>&1 || chronyc makestep >/dev/null 2>&1 || true; sleep 1; done
chronyc tracking | grep -E 'Reference ID|System time|Last offset' || true
