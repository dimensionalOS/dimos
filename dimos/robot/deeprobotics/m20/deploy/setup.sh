#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

deploy_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bridge_dir="$deploy_dir/../onboard/drdds-zenoh-bridge/cpp"
navigation_host="${M20_NAVIGATION_HOST:-user@10.21.31.106}"
camera_host="${M20_CAMERA_HOST:-user@10.21.31.103}"
camera_rust_dir="$deploy_dir/../camera/rust"
# The native camera relay is a cross-compiled aarch64 binary (AOS has no Rust toolchain). Point
# M20_CAMERA_BINARY at it, or drop it at camera/rust/result/bin/m20_camera (e.g. a nix build on a dev box).
camera_binary="${M20_CAMERA_BINARY:-$camera_rust_dir/result/bin/m20_camera}"

if ! systemctl cat rsdriver.service >/dev/null 2>&1; then
  echo "Run this from the DimOS checkout on GOS (10.21.31.104)." >&2
  exit 1
fi

if [[ -z "${DIMOS_LCM_DIR:-}" && -d /var/opt/robot/data/m20-deps/dimos-lcm ]]; then
  export DIMOS_LCM_DIR=/var/opt/robot/data/m20-deps/dimos-lcm
fi

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

echo "Installing the direct camera relay on AOS (${camera_host})..."
if [[ ! -x "$camera_binary" ]]; then
  echo "Native camera relay binary not found at $camera_binary." >&2
  echo "Build it for aarch64 (cargo zigbuild --target aarch64-unknown-linux-gnu.2.17 in $camera_rust_dir)" >&2
  echo "and set M20_CAMERA_BINARY, or place it at camera/rust/result/bin/m20_camera." >&2
  exit 1
fi
cam_stage="$(mktemp -d /tmp/dimos-m20-camera.XXXXXX)"
trap 'rm -r "$stage_dir" "$cam_stage"' EXIT
install -Dm755 "$camera_binary" "$cam_stage/m20_camera"
install -Dm644 "$deploy_dir/m20-camera.service" "$cam_stage/m20-camera.service"
install -Dm644 "$deploy_dir/m20-camera-config.json" "$cam_stage/config.json"
tar -C "$cam_stage" -cf - . | ssh \
  -o StrictHostKeyChecking=accept-new \
  -o ConnectTimeout=10 \
  "$camera_host" '
    set -eu
    setup_dir=$(mktemp -d /tmp/dimos-m20-camera-setup.XXXXXX)
    trap '\''rm -r "$setup_dir"'\'' EXIT
    tar -xf - -C "$setup_dir"

    sudo install -Dm755 "$setup_dir/m20_camera" /usr/local/bin/m20_camera
    sudo install -Dm644 "$setup_dir/config.json" /etc/dimos-m20-camera/config.json
    sudo install -Dm644 "$setup_dir/m20-camera.service" /etc/systemd/system/m20-camera.service
    sudo systemctl daemon-reload

    # stop the vendor RTSP camera pipelines (they hold /dev/video0,2); the service also kills them on every start
    sudo pkill -f gst-launch-1.0.*rtspclientsink || true

    sudo systemctl enable m20-camera.service
    sudo systemctl restart m20-camera.service
    sleep 6
    systemctl is-active --quiet m20-camera.service
    systemctl is-enabled --quiet m20-camera.service
  '

echo "M20 setup complete."
