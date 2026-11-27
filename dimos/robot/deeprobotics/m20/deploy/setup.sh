#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

deploy_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bridge_dir="$deploy_dir/../onboard/drdds-zenoh-bridge/cpp"
navigation_host="${M20_NAVIGATION_HOST:-user@10.21.31.106}"

if ! systemctl cat rsdriver.service >/dev/null 2>&1; then
  echo "Run this from the DimOS checkout on GOS (10.21.31.104)." >&2
  exit 1
fi

if [[ -z "${DIMOS_LCM_DIR:-}" && -d /var/opt/robot/data/m20-deps/dimos-lcm ]]; then
  export DIMOS_LCM_DIR=/var/opt/robot/data/m20-deps/dimos-lcm
fi

echo "Making nix reachable over ssh so a closure can be copied onto GOS..."
# `nix copy --to ssh://user@10.21.31.104` runs `nix-store` on the far side through a non-interactive
# shell. Two things stop that working on a fresh GOS:
#
# 1. Ubuntu's ~/.bashrc returns before it sets anything when the shell is not interactive, and
#    /etc/profile.d/nix.sh is never read at all for `ssh host cmd`, so nix is not on PATH and the copy
#    fails with "bash: nix-store: command not found". The PATH therefore has to be set ABOVE that guard.
# 2. The daemon rejects store paths from a user that is not trusted:
#    "cannot add path '...' because it lacks a signature by a trusted key". Signing would work too, but
#    it needs a key on every machine that builds for the robot.
#
# Both are idempotent; a second run of this script changes nothing.
nix_path_marker="dimos: nix on PATH for non-interactive"
if ! grep -q "$nix_path_marker" "$HOME/.bashrc" 2>/dev/null; then
  nix_block="$(mktemp)"
  cat >"$nix_block" <<'NIXPATH'
# dimos: nix on PATH for non-interactive shells too. `nix copy --to ssh://...` runs `nix-store` over ssh,
# and this file returns a few lines below when the shell is not interactive, before /etc/profile.d/nix.sh
# would have been read, so the PATH has to be set above that guard.
for dimos_nix_bin in "$HOME/.nix-profile/bin" /nix/var/nix/profiles/default/bin; do
    if [ -d "$dimos_nix_bin" ] && [[ ":$PATH:" != *":$dimos_nix_bin:"* ]]; then
        PATH="$dimos_nix_bin:$PATH"
    fi
done
unset dimos_nix_bin
export PATH

NIXPATH
  cat "$HOME/.bashrc" >>"$nix_block"
  cp "$HOME/.bashrc" "$HOME/.bashrc.before-dimos-nix-path"
  mv "$nix_block" "$HOME/.bashrc"
  chmod 644 "$HOME/.bashrc"
fi
if [[ -f /etc/nix/nix.conf ]] && ! grep -qE "^trusted-users\b.*\b$USER\b" /etc/nix/nix.conf; then
  printf 'trusted-users = root %s\n' "$USER" | sudo tee -a /etc/nix/nix.conf >/dev/null
  sudo systemctl restart nix-daemon 2>/dev/null || true
fi
# Proves it from the outside, the way `nix copy` will see it, rather than from this shell's PATH.
ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 "$USER@localhost" \
  'command -v nix-store >/dev/null' \
  || echo "warning: nix-store is still not on the PATH of a non-interactive ssh shell" >&2

echo "Building the M20 DrDDS/Zenoh bridge on GOS..."
"$bridge_dir/build.sh"

stage_dir="$(mktemp -d /tmp/dimos-m20-deploy.XXXXXX)"
trap 'rm -r "$stage_dir"' EXIT
install -Dm755 "$bridge_dir/build/m20_drdds_zenoh_bridge" \
  "$stage_dir/m20_drdds_zenoh_bridge"
install -Dm644 "$deploy_dir/drdds-zenoh-bridge.service" \
  "$stage_dir/drdds-zenoh-bridge.service"
install -Dm644 "$deploy_dir/localization.service.d/10-dimos-lio.conf" \
  "$stage_dir/10-dimos-lio.conf"

echo "Masking the obsolete GOS lidar driver..."
sudo systemctl mask --now rsdriver.service
sudo systemctl disable --now dimos-m20-fastdds-permissions.path 2>/dev/null || true
sudo systemctl disable --now dimos-m20-fastdds-permissions.service 2>/dev/null || true
sudo rm -f \
  /etc/systemd/system/rsdriver.service.d/10-dimos-shm-permissions.conf \
  /usr/local/libexec/dimos-m20-rsdriver-shm-permissions \
  /etc/systemd/system/dimos-m20-fastdds-permissions.path \
  /etc/systemd/system/dimos-m20-fastdds-permissions.service
sudo systemctl daemon-reload
! systemctl is-active --quiet rsdriver.service
! systemctl is-enabled --quiet rsdriver.service

echo "Installing vendor LIO and the bridge on NOS (${navigation_host})..."
tar -C "$stage_dir" -cf - . | ssh \
  -o StrictHostKeyChecking=accept-new \
  -o ConnectTimeout=10 \
  "$navigation_host" '
    set -eu
    setup_dir=$(mktemp -d /tmp/dimos-m20-setup.XXXXXX)
    trap '\''rm -r "$setup_dir"'\'' EXIT
    tar -xf - -C "$setup_dir"

    sudo install -Dm755 "$setup_dir/m20_drdds_zenoh_bridge" \
      /usr/local/libexec/m20_drdds_zenoh_bridge
    sudo install -Dm644 "$setup_dir/drdds-zenoh-bridge.service" \
      /etc/systemd/system/drdds-zenoh-bridge.service
    sudo install -Dm644 "$setup_dir/10-dimos-lio.conf" \
      /etc/systemd/system/localization.service.d/10-dimos-lio.conf

    sudo rm -f \
      /etc/systemd/system/drdds-zenoh-bridge.service.d/connect.conf \
      /etc/systemd/system/localization.service.d/lio.conf \
      /etc/systemd/system/planner.service.d/10-dimos-command-ownership.conf \
      /etc/systemd/system/multicast-relay.service.d/10-dimos-network-readiness.conf \
      /usr/local/libexec/dimos-m20-multicast-relay-supervisor
    sudo systemctl daemon-reload

    sudo systemctl stop drdds-zenoh-bridge.service localization.service
    sudo systemctl mask --now multicast-relay.service
    sudo systemctl mask --now hsLidar.service
    sudo systemctl mask --now planner.service

    sudo systemctl unmask rsdriver.service localization.service \
      drdds-zenoh-bridge.service
    sudo systemctl enable rsdriver.service localization.service \
      drdds-zenoh-bridge.service
    sudo systemctl restart rsdriver.service
    sudo systemctl restart localization.service
    sudo systemctl restart drdds-zenoh-bridge.service

    systemctl is-active --quiet rsdriver.service
    systemctl is-active --quiet localization.service
    systemctl is-active --quiet drdds-zenoh-bridge.service
    systemctl is-enabled --quiet rsdriver.service
    systemctl is-enabled --quiet localization.service
    systemctl is-enabled --quiet drdds-zenoh-bridge.service
    test -e /run/dimos-m20-lio-ready
    ! systemctl is-active --quiet hsLidar.service
    ! systemctl is-active --quiet multicast-relay.service
    ! systemctl is-active --quiet planner.service
    test "$(systemctl is-enabled hsLidar.service)" = masked
    test "$(systemctl is-enabled multicast-relay.service)" = masked
    test "$(systemctl is-enabled planner.service)" = masked
  '

echo "M20 setup complete."
