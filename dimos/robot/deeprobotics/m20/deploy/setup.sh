#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

deploy_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
navigation_host="${M20_NAVIGATION_HOST:-user@10.21.31.106}"

if ! systemctl cat rsdriver.service >/dev/null 2>&1; then
  echo "Run this script on the M20 computer at 10.21.31.104." >&2
  exit 1
fi

echo "Configuring lidar forwarding on ${navigation_host}..."
tar -C "$deploy_dir" -cf - \
  dimos-m20-multicast-relay-supervisor \
  multicast-relay.service.d/10-dimos-network-readiness.conf | \
  ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 "$navigation_host" '
    set -eu
    setup_dir=$(mktemp -d /tmp/dimos-m20-setup.XXXXXX)
    trap '\''rm -f "$setup_dir/dimos-m20-multicast-relay-supervisor" "$setup_dir/multicast-relay.service.d/10-dimos-network-readiness.conf"; rmdir "$setup_dir/multicast-relay.service.d" "$setup_dir" 2>/dev/null || true'\'' EXIT
    tar -xf - -C "$setup_dir"
    sudo install -Dm755 "$setup_dir/dimos-m20-multicast-relay-supervisor" /usr/local/libexec/dimos-m20-multicast-relay-supervisor
    sudo install -Dm644 "$setup_dir/multicast-relay.service.d/10-dimos-network-readiness.conf" /etc/systemd/system/multicast-relay.service.d/10-dimos-network-readiness.conf
    sudo rm -f /etc/systemd/system/planner.service.d/10-dimos-command-ownership.conf
    sudo systemctl daemon-reload
    sudo systemctl enable multicast-relay.service
    sudo systemctl restart multicast-relay.service
    sudo systemctl disable --now planner.service
    systemctl is-active --quiet multicast-relay.service
    ! systemctl is-active --quiet planner.service
  '

echo "Configuring lidar access on 10.21.31.104..."
sudo install -Dm755 \
  "$deploy_dir/dimos-m20-rsdriver-shm-permissions" \
  /usr/local/libexec/dimos-m20-rsdriver-shm-permissions
sudo install -Dm644 \
  "$deploy_dir/rsdriver.service.d/10-dimos-shm-permissions.conf" \
  /etc/systemd/system/rsdriver.service.d/10-dimos-shm-permissions.conf

if sudo test -e /etc/systemd/system/dimos-m20-fastdds-permissions.path; then
  sudo systemctl disable --now dimos-m20-fastdds-permissions.path
fi
sudo rm -f \
  /etc/systemd/system/dimos-m20-fastdds-permissions.path \
  /etc/systemd/system/dimos-m20-fastdds-permissions.service
sudo systemctl daemon-reload
sudo systemctl enable rsdriver.service
sudo systemctl restart rsdriver.service
systemctl is-active --quiet rsdriver.service

echo "M20 setup complete."
