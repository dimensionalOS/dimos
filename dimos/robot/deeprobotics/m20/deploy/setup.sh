#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Provision an M20 for DimOS from GOS: sync the three boards' clocks, install the DrDDS/Zenoh bridge and vendor LIO on
# NOS, mask the obsolete GOS lidar driver, and install the direct camera relay on AOS. Idempotent -- safe to re-run.
# A preflight checks every board is reachable and every privileged step can run before anything is changed, so it
# fails up front with one clear message instead of part way through on a fresh dog.
#
#   M20_NAVIGATION_HOST   NOS ssh target        (default user@10.21.31.106)
#   M20_CAMERA_HOST       AOS ssh target        (default user@10.21.31.103)
#   M20_CAMERA_BINARY     the aarch64 relay     (default camera/rust/result/bin/m20_camera)
#   M20_CAMERA_SUDO_PASS  AOS sudo password     (the vendor board isn't passwordless; NOS and GOS are)
set -euo pipefail

deploy_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bridge_dir="$deploy_dir/../onboard/drdds-zenoh-bridge/cpp"
navigation_host="${M20_NAVIGATION_HOST:-user@10.21.31.106}"
camera_host="${M20_CAMERA_HOST:-user@10.21.31.103}"
camera_rust_dir="$deploy_dir/../camera/rust"
# The native camera relay is a cross-compiled aarch64 binary (AOS has no Rust toolchain). Point
# M20_CAMERA_BINARY at it, or drop it at camera/rust/result/bin/m20_camera (e.g. a nix build on a dev box).
camera_binary="${M20_CAMERA_BINARY:-$camera_rust_dir/result/bin/m20_camera}"
# GOS is the DimOS compute board and the LAN time master the other boards sync their clocks to.
master_ip="10.21.31.104"
# AOS's sudo wants a password (the vendor board); NOS and GOS are passwordless. Fed to `sudo -A` through a one-shot
# askpass staged on AOS (base64 so a quote in the password survives), so the password never sits in a command line.
camera_sudo_pass="${M20_CAMERA_SUDO_PASS-}"
camera_sudo_pass_b64="$(printf '%s' "$camera_sudo_pass" | base64 | tr -d '\n')"
ssh_opts=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
aos_askpass=/tmp/dimos-m20-askpass

die() { echo "setup: $*" >&2; exit 1; }

# `sudo -A` on AOS, reading the password from the staged askpass; a no-op askpass path is harmless when AOS turns out
# passwordless (sudo never calls it). Used verbatim at the top of every AOS remote script.
aos_sudo_preamble="export SUDO_ASKPASS=$aos_askpass; sudo() { command sudo -A \"\$@\"; }"

stage_dir=""; cam_stage=""; aos_askpass_staged=""
cleanup() {
  [[ -n "$stage_dir" ]] && rm -rf "$stage_dir"
  [[ -n "$cam_stage" ]] && rm -rf "$cam_stage"
  [[ -n "$aos_askpass_staged" ]] && ssh -n "${ssh_opts[@]}" "$camera_host" "rm -f $aos_askpass" 2>/dev/null || true
}
trap cleanup EXIT

# --- preflight -------------------------------------------------------------
echo "Preflight: checking this is GOS, the boards are reachable, and every step can run..."

systemctl cat rsdriver.service >/dev/null 2>&1 \
  || die "run this from the DimOS checkout on GOS ($master_ip): rsdriver.service isn't here."
sudo -n true 2>/dev/null || die "GOS sudo should be passwordless but isn't."

[[ -x "$camera_binary" ]] || die "camera relay binary not found at $camera_binary.
  Build it for aarch64 (cargo zigbuild --target aarch64-unknown-linux-gnu.2.17 in $camera_rust_dir) and set
  M20_CAMERA_BINARY, or place it at camera/rust/result/bin/m20_camera."

for pair in "NOS:$navigation_host" "AOS:$camera_host"; do
  name="${pair%%:*}"; host="${pair#*:}"
  ssh -n "${ssh_opts[@]}" "$host" true 2>/dev/null || die "$name ($host) is not reachable over ssh."
done

# NOS must be passwordless (its install heredoc uses plain sudo).
ssh -n "${ssh_opts[@]}" "$navigation_host" 'sudo -n true 2>/dev/null' \
  || die "NOS ($navigation_host) sudo must be passwordless."

# AOS: stage the askpass if it needs a password, and prove sudo works before touching anything.
if ssh -n "${ssh_opts[@]}" "$camera_host" 'sudo -n true 2>/dev/null'; then
  echo "  AOS sudo is passwordless."
else
  # Ask once (env var, or a single interactive prompt), then reuse the staged askpass for every AOS step.
  if [[ -z "$camera_sudo_pass" ]]; then
    [[ -t 0 ]] || die "AOS ($camera_host) sudo needs a password. Set M20_CAMERA_SUDO_PASS and re-run (no terminal to prompt)."
    read -rsp "AOS ($camera_host) sudo password: " camera_sudo_pass; echo
    [[ -n "$camera_sudo_pass" ]] || die "no AOS password entered."
    camera_sudo_pass_b64="$(printf '%s' "$camera_sudo_pass" | base64 | tr -d '\n')"
  fi
  ask_local="$(mktemp)"
  printf '#!/bin/sh\nprintf %%s %s | base64 -d\n' "'$camera_sudo_pass_b64'" >"$ask_local"
  scp -q "${ssh_opts[@]}" "$ask_local" "$camera_host:$aos_askpass"
  rm -f "$ask_local"
  aos_askpass_staged=1
  ssh -n "${ssh_opts[@]}" "$camera_host" "chmod 700 $aos_askpass; SUDO_ASKPASS=$aos_askpass sudo -A -k true 2>/dev/null" \
    || die "AOS sudo rejected the M20_CAMERA_SUDO_PASS password."
  echo "  AOS sudo via staged askpass: OK."
fi

if [[ -z "${DIMOS_LCM_DIR:-}" && -d /var/opt/robot/data/m20-deps/dimos-lcm ]]; then
  export DIMOS_LCM_DIR=/var/opt/robot/data/m20-deps/dimos-lcm
fi

# --- clock sync ------------------------------------------------------------
# Otherwise the boards free-run: NOS can't reach the public NTP pools at all (no source) and ran seconds ahead; AOS
# reaches them but drifted past chrony's step window so it only slews. Cross-board timestamps then diverge by seconds
# -- the Controller reads wrong source-to-screen video latency and misaligned lidar-vs-camera timing. GOS becomes the
# LAN time master (it keeps the pools, for rough UTC); NOS and AOS track GOS ONLY (sync-clock.sh comments their stock
# pools -- the robot's internet, office wifi behind the vendor NAT, is too jittery to trust and a reachable pool would
# out-rank the master). A chrony restart resets the step counter, so the initial multi-second gap is stepped once and
# then only slewed (stable, internally consistent at ms-level; the NICs can do PTP if we ever need tighter).
echo "Clock: making GOS the LAN NTP master..."
sudo bash "$deploy_dir/clock-master.sh"

echo "Clock: syncing NOS to GOS..."
scp -q "${ssh_opts[@]}" "$deploy_dir/sync-clock.sh" "$navigation_host:/tmp/dimos-m20-sync-clock.sh"
ssh -n "${ssh_opts[@]}" "$navigation_host" \
  "sudo bash /tmp/dimos-m20-sync-clock.sh $master_ip; rm -f /tmp/dimos-m20-sync-clock.sh"

echo "Clock: syncing AOS to GOS..."
scp -q "${ssh_opts[@]}" "$deploy_dir/sync-clock.sh" "$camera_host:/tmp/dimos-m20-sync-clock.sh"
ssh -n "${ssh_opts[@]}" "$camera_host" \
  "$aos_sudo_preamble; sudo bash /tmp/dimos-m20-sync-clock.sh $master_ip; rm -f /tmp/dimos-m20-sync-clock.sh"

# --- GOS: mask the obsolete lidar driver -----------------------------------
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
! systemctl is-active --quiet rsdriver.service || die "GOS rsdriver.service is still active after masking."
! systemctl is-enabled --quiet rsdriver.service || die "GOS rsdriver.service is still enabled after masking."

# --- NOS: vendor LIO + the bridge ------------------------------------------
echo "Building the M20 DrDDS/Zenoh bridge on GOS..."
"$bridge_dir/build.sh"

stage_dir="$(mktemp -d /tmp/dimos-m20-deploy.XXXXXX)"
install -Dm755 "$bridge_dir/build/m20_drdds_zenoh_bridge" "$stage_dir/m20_drdds_zenoh_bridge"
install -Dm644 "$deploy_dir/drdds-zenoh-bridge.service" "$stage_dir/drdds-zenoh-bridge.service"
install -Dm644 "$deploy_dir/localization.service.d/10-dimos-lio.conf" "$stage_dir/10-dimos-lio.conf"

echo "Installing vendor LIO and the bridge on NOS ($navigation_host)..."
tar -C "$stage_dir" -cf - . | ssh "${ssh_opts[@]}" "$navigation_host" '
    set -eu
    setup_dir=$(mktemp -d /tmp/dimos-m20-setup.XXXXXX)
    trap '\''rm -r "$setup_dir"'\'' EXIT
    tar -xf - -C "$setup_dir"

    sudo install -Dm755 "$setup_dir/m20_drdds_zenoh_bridge" /usr/local/libexec/m20_drdds_zenoh_bridge
    sudo install -Dm644 "$setup_dir/drdds-zenoh-bridge.service" /etc/systemd/system/drdds-zenoh-bridge.service
    sudo install -Dm644 "$setup_dir/10-dimos-lio.conf" /etc/systemd/system/localization.service.d/10-dimos-lio.conf

    sudo rm -f \
      /etc/systemd/system/drdds-zenoh-bridge.service.d/connect.conf \
      /etc/systemd/system/localization.service.d/lio.conf \
      /etc/systemd/system/planner.service.d/10-dimos-command-ownership.conf \
      /etc/systemd/system/multicast-relay.service.d/10-dimos-network-readiness.conf \
      /usr/local/libexec/dimos-m20-multicast-relay-supervisor
    sudo systemctl daemon-reload

    sudo systemctl stop drdds-zenoh-bridge.service localization.service 2>/dev/null || true
    sudo systemctl mask --now multicast-relay.service
    sudo systemctl mask --now hsLidar.service
    sudo systemctl mask --now planner.service

    sudo systemctl unmask rsdriver.service localization.service drdds-zenoh-bridge.service
    sudo systemctl enable rsdriver.service localization.service drdds-zenoh-bridge.service
    sudo systemctl restart rsdriver.service
    sudo systemctl restart localization.service
    sudo systemctl restart drdds-zenoh-bridge.service

    fail() { echo "NOS setup: $*" >&2; exit 1; }
    for s in rsdriver localization drdds-zenoh-bridge; do
      systemctl is-active  --quiet "$s.service" || fail "$s.service did not come up."
      systemctl is-enabled --quiet "$s.service" || fail "$s.service is not enabled."
    done
    test -e /run/dimos-m20-lio-ready || fail "LIO never signalled ready (/run/dimos-m20-lio-ready)."
    for s in hsLidar multicast-relay planner; do
      ! systemctl is-active --quiet "$s.service" || fail "$s.service is still active (should be masked)."
      test "$(systemctl is-enabled "$s.service")" = masked || fail "$s.service is not masked."
    done
  '

# --- AOS: the direct camera relay ------------------------------------------
echo "Installing the direct camera relay on AOS ($camera_host)..."
cam_stage="$(mktemp -d /tmp/dimos-m20-camera.XXXXXX)"
install -Dm755 "$camera_binary" "$cam_stage/m20_camera"
install -Dm644 "$deploy_dir/m20-camera.service" "$cam_stage/m20-camera.service"
install -Dm644 "$deploy_dir/m20-camera-config.json" "$cam_stage/config.json"
tar -C "$cam_stage" -cf - . | ssh "${ssh_opts[@]}" "$camera_host" "
    set -eu
    $aos_sudo_preamble
    setup_dir=\$(mktemp -d /tmp/dimos-m20-camera-setup.XXXXXX)
    trap 'rm -r \"\$setup_dir\"' EXIT
    tar -xf - -C \"\$setup_dir\"

    sudo install -Dm755 \"\$setup_dir/m20_camera\" /usr/local/bin/m20_camera
    sudo install -Dm644 \"\$setup_dir/config.json\" /etc/dimos-m20-camera/config.json
    sudo install -Dm644 \"\$setup_dir/m20-camera.service\" /etc/systemd/system/m20-camera.service
    sudo systemctl daemon-reload

    # stop the vendor RTSP camera pipelines (they hold /dev/video0,2); the service also kills them on every start
    sudo pkill -f 'gst-launch-1.0.*rtspclientsink' || true

    sudo systemctl enable m20-camera.service
    sudo systemctl restart m20-camera.service
    sleep 6
    systemctl is-active  --quiet m20-camera.service || { echo 'AOS setup: m20-camera.service did not come up.' >&2; exit 1; }
    systemctl is-enabled --quiet m20-camera.service || { echo 'AOS setup: m20-camera.service is not enabled.' >&2; exit 1; }
  "

# --- verify ----------------------------------------------------------------
echo "Verifying clocks reference GOS ($master_ip)..."
for pair in "NOS:$navigation_host" "AOS:$camera_host"; do
  name="${pair%%:*}"; host="${pair#*:}"
  ref="$(ssh -n "${ssh_opts[@]}" "$host" 'chronyc tracking 2>/dev/null | sed -n "s/^Reference ID *: //p"')"
  echo "  $name clock reference: ${ref:-unknown}"
  case "$ref" in
    *"$master_ip"*) ;;
    *) echo "  note: $name is not yet referencing GOS (it may take a minute to prefer it over a reachable public pool)." >&2 ;;
  esac
done

echo "M20 setup complete."
