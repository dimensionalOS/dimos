#!/bin/bash
#
# M20 NOS deployment helper
#
# Syncs code, sets up NOS prerequisites, and manages the nav stack.
# Everything runs natively via nix — no Docker needed.
#
# Prerequisites:
#   SSH keys copied to NOS and AOS (ssh-copy-id)
#   NOS user has passwordless sudo (or set SUDO_PASS env var)
#
# Usage:
#   ./deploy.sh bootstrap [--host <hostname>]            # NEW dog post-OTA: provision + sync + install-binaries + restore-rsdriver-config
#   ./deploy.sh provision [--host <hostname>]            # systemd units, fstab, sudoers, sysctl (idempotent)
#   ./deploy.sh sync [--host <hostname>]                 # Sync dimos source to NOS
#   ./deploy.sh install-binaries [--host <hostname>]     # Build + install drdds_recv from source on NOS
#   ./deploy.sh restore-rsdriver-config [--host <hostname>]  # Restore our rsdriver config edits (post-OTA)
#   ./deploy.sh backup-customizations [--host <hostname>]    # Snapshot pre-OTA NOS state to ~/m20_backup_<ts>/
#   ./deploy.sh start [--host <hostname>]                # Start smartnav on NOS
#   ./deploy.sh stop [--host <hostname>]                 # Stop smartnav
#   ./deploy.sh restart [--host <hostname>]              # Stop + start
#   ./deploy.sh bridge-start [--host <hostname>]         # Restart drdds SHM bridge
#   ./deploy.sh bridge-stop [--host <hostname>]          # Stop bridge
#   ./deploy.sh status [--host <hostname>]               # Show status
#   ./deploy.sh viewer [--host <hostname>]               # Start tunnels + dimos-viewer
#
# First time on a new M20 post-OTA (interactive sudo, one-time):
#   ./deploy.sh bootstrap --host m20-XXX-gogo
#
# After reboot (no-op — provision makes everything persistent):
#   ./deploy.sh start --host m20-770-gogo
#   ./deploy.sh viewer --host m20-770-gogo
#
# Quick deploy after code changes:
#   ./deploy.sh sync --host m20-770-gogo && ./deploy.sh restart --host m20-770-gogo
#
# Before a system OTA on a dog you've already customized:
#   ./deploy.sh backup-customizations --host m20-XXX-gogo
# After the OTA finishes:
#   ./deploy.sh bootstrap --host m20-XXX-gogo
#
# Options:
#   --host <hostname>   Access robot via Tailscale (e.g., m20-770-gogo)
#                       Without --host: uses AOS WiFi (10.21.41.1)

set -e

CMD="${1:-help}"
shift 2>/dev/null || true

REMOTE_HOST=""
positional=()
while [ $# -gt 0 ]; do
    case "$1" in
        --host) REMOTE_HOST="$2"; shift 2 ;;
        *) positional+=("$1"); shift ;;
    esac
done
set -- "${positional[@]}"

NOS_HOST="${1:-10.21.31.106}"
NOS_USER="${2:-user}"
AOS_INTERNAL="10.21.31.103"
DEPLOY_DIR="/var/opt/robot/data/dimos"
VENV="/home/user/dimos-venv"
SMARTNAV_MODULE="dimos.robot.deeprobotics.m20.blueprints.nav.m20_smartnav_native"
SMARTNAV_LOG="/tmp/smartnav_native.log"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

if [ -n "${REMOTE_HOST}" ]; then
    JUMP_HOST="${NOS_USER}@${REMOTE_HOST}"
else
    JUMP_HOST="${JUMP_HOST:-user@10.21.41.1}"
fi

SSH_OPTS="-o ProxyJump=${JUMP_HOST}"

remote_ssh() {
    ssh ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "$@"
}

remote_sudo() {
    if [ -n "${SUDO_PASS}" ]; then
        remote_ssh "echo '${SUDO_PASS}' | sudo -S $*" 2>&1 | grep -v '^\[sudo\]' || true
    else
        remote_ssh "sudo $*"
    fi
}

RSYNC_EXCLUDES=(
    --exclude '__pycache__'
    --exclude '*.pyc'
    --exclude '.git'
    --exclude 'venv'
    --exclude '.venv'
    --exclude '.pytest_cache'
    --exclude '.mypy_cache'
    --exclude '*.egg-info'
    --exclude '.beads'
    --exclude '.runtime'
    --exclude 'node_modules'
    --exclude 'data'
    --exclude 'logs'
    --exclude '.claude'
    # CMake build artifacts live on NOS only — mustn't be wiped by --delete.
    # Exclude without trailing / so symlinks named `result` (nix output links)
    # are also preserved — trailing / would only match directories.
    --exclude 'build'
    --exclude 'CMakeFiles'
    --exclude 'CMakeCache.txt'
    --exclude 'cmake_install.cmake'
    --exclude '_deps'
    --exclude 'result'
)

case "${CMD}" in
    sync)
        echo "=== Syncing dimos source to NOS ==="
        rsync -avz --delete \
            -e "ssh ${SSH_OPTS}" \
            "${RSYNC_EXCLUDES[@]}" \
            "${DIMOS_ROOT}/" "${NOS_USER}@${NOS_HOST}:${DEPLOY_DIR}/"
        # Ensure nix binary symlinks exist. Pick the NEWEST store directory
        # by mtime (not alphabetical — the old `for store_path in ... do
        # ln -sf; done` loop silently reverted to whichever hash sorted
        # last, which routinely pointed at stale binaries after a rebuild).
        # `ls -dt` sorts directories by mtime descending; head -1 picks
        # the most recently created nix store output.
        remote_ssh "
            mkdir -p ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/arise_slam/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/terrain_analysis/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/local_planner/result/bin
            mkdir -p ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/path_follower/result/bin

            link_newest() {
                # \$1 = store glob (directory pattern), \$2 = bin name, \$3 = dest symlink
                local store=\$(ls -dt \$1 2>/dev/null | head -1)
                [ -n \"\$store\" ] && [ -f \"\${store%/}/bin/\$2\" ] && \
                    ln -sfn \"\${store%/}/bin/\$2\" \"\$3\"
            }

            link_newest '/nix/store/*-drdds_lidar_bridge-*/' drdds_lidar_bridge \
                ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin/drdds_lidar_bridge
            # airy_imu_bridge ships from the same flake as drdds_lidar_bridge,
            # so both resolve from the same store path.
            link_newest '/nix/store/*-drdds_lidar_bridge-*/' airy_imu_bridge \
                ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin/airy_imu_bridge
            link_newest '/nix/store/*-smartnav-arise-slam-*/' arise_slam \
                ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/arise_slam/result/bin/arise_slam
            link_newest '/nix/store/*-smartnav-terrain-analysis-*/' terrain_analysis \
                ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/terrain_analysis/result/bin/terrain_analysis
            link_newest '/nix/store/*-smartnav-local-planner-*/' local_planner \
                ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/local_planner/result/bin/local_planner
            link_newest '/nix/store/*-smartnav-path-follower-*/' path_follower \
                ${DEPLOY_DIR}/dimos/navigation/smart_nav/modules/path_follower/result/bin/path_follower

            # nav_cmd_pub (built via cmake, not nix — needs /usr/local/lib/libdrdds.so)
            # Build dir lives in the synced source tree so it survives reboots (ext4).
            nav_cmd_pub_bin=${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/build/nav_cmd_pub
            [ -f \"\$nav_cmd_pub_bin\" ] && \
                ln -sfn \"\$nav_cmd_pub_bin\" ${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin/nav_cmd_pub
        "
        echo "=== Sync Complete ==="
        ;;

    provision)
        echo "=== Provisioning NOS for persistent M20 nav ==="
        echo "You will be prompted for sudo password on the NOS (once)."
        echo ""

        # Generate the provisioning script locally, then run it on NOS under sudo.
        # Everything is idempotent and survives reboot.
        PROVISION_SCRIPT=$(cat <<'PROVISION_EOF'
#!/bin/bash
# M20 NOS persistent provisioning — idempotent
set -e

echo "[provision] Installing /nix bind mount in /etc/fstab..."
FSTAB_LINE="/var/opt/robot/data/nix /nix none bind 0 0"
if ! grep -qF "/var/opt/robot/data/nix /nix" /etc/fstab; then
    echo "${FSTAB_LINE}" >> /etc/fstab
    echo "  added"
else
    echo "  already present"
fi
mkdir -p /nix
if ! mountpoint -q /nix; then
    mount /nix
    echo "  /nix mounted"
else
    echo "  /nix already mounted"
fi

echo "[provision] Disabling legacy drdds-bridge.service (if present)..."
# Older rig provisioning installed drdds-bridge.service, which runs the
# same drdds_recv binary as our drdds-recv.service. Leaving both enabled
# produces two drdds_recv processes fighting over the FastDDS SHM
# subscription (load avg spike, imu_vs_frame_end growing to -0.7s,
# catastrophic SLAM drift — see FASTLIO2_LOG Finding #23). Disable and
# stop the legacy unit if it exists.
if systemctl list-unit-files drdds-bridge.service >/dev/null 2>&1 \
   && systemctl list-unit-files drdds-bridge.service | grep -q drdds-bridge; then
    systemctl stop drdds-bridge.service 2>/dev/null || true
    systemctl disable drdds-bridge.service 2>/dev/null || true
    echo "  legacy drdds-bridge.service stopped + disabled"
else
    echo "  not present"
fi

echo "[provision] Installing drdds-recv.service..."
cat > /etc/systemd/system/drdds-recv.service <<'UNIT'
[Unit]
Description=drdds bridge receiver (drdds SHM -> POSIX SHM for dimos)
Documentation=file:///var/opt/robot/data/dimos/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/drdds_recv.cpp
After=network-online.target m20-nav-net.service
Wants=network-online.target
Before=rsdriver.service yesense.service

[Service]
Type=simple
# Wait for both physical interfaces to be UP before starting. At cold boot
# network-online.target can fire before eth0/eth1 have IPs; FastDDS would
# then bind only to lo, breaking discovery for cross-interface peers.
# Bounded by 30s in case an interface is genuinely missing.
ExecStartPre=/bin/sh -c 't=0; until ip -br addr show eth0 2>/dev/null | grep -q UP && ip -br addr show eth1 2>/dev/null | grep -q UP; do t=$((t+1)); [ $t -ge 30 ] && exit 0; sleep 1; done'
ExecStart=/opt/drdds_bridge/lib/drdds_bridge/drdds_recv
Restart=on-failure
RestartSec=3
StandardOutput=append:/var/log/drdds_recv.log
StandardError=append:/var/log/drdds_recv.log

[Install]
WantedBy=multi-user.target
UNIT

echo "[provision] Installing rsdriver ordering dropin..."
mkdir -p /etc/systemd/system/rsdriver.service.d
cat > /etc/systemd/system/rsdriver.service.d/10-drdds-order.conf <<'DROPIN'
[Unit]
# drdds_recv must open SHM before rsdriver starts publishing — otherwise
# rsdriver's drdds SHM segments never get discovered by the reader.
After=drdds-recv.service
Requires=drdds-recv.service
DROPIN

echo "[provision] Installing yesense ordering dropin..."
mkdir -p /etc/systemd/system/yesense.service.d
cat > /etc/systemd/system/yesense.service.d/10-drdds-order.conf <<'DROPIN'
[Unit]
# Same SHM-discovery race as rsdriver. Empirically, post-cold-boot
# yesense without this dropin ends up matched=0 against drdds_recv
# even though discovery on rsdriver succeeds (Phase B 2026-04-25).
After=drdds-recv.service
Requires=drdds-recv.service
DROPIN

echo "[provision] Installing sshd OOM protection dropin..."
mkdir -p /etc/systemd/system/ssh.service.d
cat > /etc/systemd/system/ssh.service.d/oom.conf <<'DROPIN'
[Service]
# Pin sshd OOM score to the kernel minimum so it survives memory pressure
# from the dimos stack. Without this, OOM killer takes sshd first under
# heavy SLAM load and we lose remote access to the dog.
OOMScoreAdjust=-1000
DROPIN

echo "[provision] Installing m20-nav-net.service (loopback multicast + DDS routes)..."
cat > /etc/systemd/system/m20-nav-net.service <<'UNIT'
[Unit]
Description=M20 nav networking (loopback multicast + DDS routes)
After=network-online.target
Wants=network-online.target
Before=rsdriver.service drdds-recv.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c '/sbin/ip link set lo multicast on'
ExecStart=/bin/sh -c '/sbin/ip route add 224.0.0.0/4 dev lo 2>/dev/null || true'
ExecStart=/bin/sh -c '/sbin/ip route add 10.21.33.103/32 via 10.21.31.103 2>/dev/null || true'

[Install]
WantedBy=multi-user.target
UNIT

echo "[provision] Setting up 4GB swap (NOS has only ~4GB RAM, dimos peaks above that)..."
SWAPFILE=/var/opt/robot/data/swapfile
if ! swapon --show=NAME --noheadings | grep -qF "$SWAPFILE"; then
    if [ ! -f "$SWAPFILE" ]; then
        # Use fallocate; mkswap fails on holes, so prefer dd if fallocate-backed FS doesn't allow swap.
        if ! fallocate -l 4G "$SWAPFILE" 2>/dev/null; then
            dd if=/dev/zero of="$SWAPFILE" bs=1M count=4096 status=progress
        fi
        chmod 600 "$SWAPFILE"
        mkswap "$SWAPFILE" >/dev/null
        echo "  created $SWAPFILE"
    fi
    if ! grep -qF "$SWAPFILE" /etc/fstab; then
        echo "$SWAPFILE none swap sw 0 0" >> /etc/fstab
        echo "  added to /etc/fstab"
    fi
    swapon "$SWAPFILE" || true
    echo "  swap on"
else
    echo "  already active"
fi

echo "[provision] Checking kernel cmdline for isolcpus=4,5,6,7..."
if grep -q 'isolcpus=4,5,6,7' /proc/cmdline; then
    echo "  present"
else
    echo "  NOT PRESENT — SLAM cpu_affinity will work without exclusivity"
    echo "  To enable, add 'isolcpus=4,5,6,7' to the kernel cmdline (bootloader-specific)"
    echo "  and reboot. Skipping auto-modification — bootloader edits are dog-specific."
fi

echo "[provision] Installing sysctl tuning for LCM (large UDP packets)..."
cat > /etc/sysctl.d/99-m20-nav.conf <<'SYSCTL'
# LCM carries 2.8MB lidar PointCloud2 fragments over UDP multicast.
# Default 208KB recv buffer drops fragments → incomplete scans → SLAM fails.
# Recommended by https://lcm-proj.github.io/lcm/content/multicast-setup.html
net.core.rmem_max=33554432
net.core.rmem_default=33554432
net.core.wmem_max=33554432
net.core.wmem_default=33554432
SYSCTL
sysctl --system | grep -E 'rmem|wmem' | head -4 || true

echo "[provision] Installing /usr/local/sbin/m20-clean-shm..."
cat > /usr/local/sbin/m20-clean-shm <<'SCRIPT'
#!/bin/bash
# Clean stale FastRTPS + drdds SHM segments. Safe to run any time bridge is stopped.
rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* /dev/shm/sem.fastrtps_* \
      /dev/shm/drdds_bridge_* /dev/shm/sem.drdds_bridge_*
SCRIPT
chmod 755 /usr/local/sbin/m20-clean-shm

echo "[provision] Installing sudoers rule for deploy.sh..."
cat > /etc/sudoers.d/m20-nav <<'SUDOERS'
# Allow the 'user' account to manage the nav services from deploy.sh without a password.
user ALL=(root) NOPASSWD: /bin/systemctl stop rsdriver, /bin/systemctl start rsdriver, /bin/systemctl restart rsdriver
user ALL=(root) NOPASSWD: /bin/systemctl stop drdds-recv, /bin/systemctl start drdds-recv, /bin/systemctl restart drdds-recv
user ALL=(root) NOPASSWD: /bin/systemctl status rsdriver, /bin/systemctl status drdds-recv, /bin/systemctl is-active rsdriver, /bin/systemctl is-active drdds-recv
user ALL=(root) NOPASSWD: /usr/local/sbin/m20-clean-shm
SUDOERS
chmod 440 /etc/sudoers.d/m20-nav
# Validate sudoers; if invalid, remove it rather than lock out sudo.
if ! visudo -c -f /etc/sudoers.d/m20-nav >/dev/null; then
    echo "[provision] ERROR: sudoers file invalid — removing"
    rm /etc/sudoers.d/m20-nav
    exit 1
fi

echo "[provision] Reloading systemd..."
systemctl daemon-reload

echo "[provision] Enabling + starting services..."
systemctl enable m20-nav-net.service drdds-recv.service
systemctl start m20-nav-net.service
# drdds-recv will be started by rsdriver dependency on next boot; start now too
if ! systemctl is-active --quiet drdds-recv.service; then
    # Kill any manually-started drdds_recv first
    pkill -f '/opt/drdds_bridge/lib/drdds_bridge/drdds_recv' 2>/dev/null || true
    sleep 1
    systemctl start drdds-recv.service
fi

echo "[provision] Done. Summary:"
systemctl is-enabled drdds-recv.service m20-nav-net.service
systemctl is-active drdds-recv.service m20-nav-net.service rsdriver.service
echo ""
echo "Persistent config installed:"
echo "  /etc/fstab:                              /nix bind mount + swap"
echo "  /etc/systemd/system/drdds-recv.service:  drdds SHM receiver (with interface-up wait)"
echo "  /etc/systemd/system/rsdriver.service.d/: ordering after drdds-recv"
echo "  /etc/systemd/system/yesense.service.d/:  ordering after drdds-recv"
echo "  /etc/systemd/system/ssh.service.d/:      sshd OOM protection"
echo "  /etc/systemd/system/m20-nav-net.service: multicast + routes"
echo "  /etc/sysctl.d/99-m20-nav.conf:           LCM large UDP buffers"
echo "  /etc/sudoers.d/m20-nav:                  NOPASSWD for service control"
echo "  /usr/local/sbin/m20-clean-shm:           SHM cleanup helper"
echo "  /var/opt/robot/data/swapfile:            4GB swap"
PROVISION_EOF
)

        # Copy script to NOS, then run under sudo with a TTY so password prompts work.
        echo "  Copying provision script to NOS..."
        remote_ssh "cat > /tmp/m20-provision.sh" <<< "${PROVISION_SCRIPT}"
        remote_ssh "chmod +x /tmp/m20-provision.sh"
        echo "  Running provision script (sudo password required, one time)..."
        if [ -n "${SUDO_PASS}" ]; then
            # Non-interactive path. Base64-encode the password locally so any
            # quoting characters (this session's literal `'` for instance)
            # survive ssh argument passing without escape gymnastics.
            PW64=$(printf '%s' "${SUDO_PASS}" | base64)
            remote_ssh "echo ${PW64} | base64 -d | sudo -S bash /tmp/m20-provision.sh" 2>&1 | grep -v '^\[sudo\] password for'
        else
            ssh -t ${SSH_OPTS} "${NOS_USER}@${NOS_HOST}" "sudo bash /tmp/m20-provision.sh"
        fi
        remote_ssh "rm -f /tmp/m20-provision.sh"
        echo "=== Provision Complete — reboots will now Just Work ==="
        echo ""
        echo "Next steps for a fresh post-OTA dog:"
        echo "  $0 sync --host \${REMOTE_HOST:-<host>}                  # push source"
        echo "  $0 install-binaries --host \${REMOTE_HOST:-<host>}      # build drdds_recv"
        echo "  $0 restore-rsdriver-config --host \${REMOTE_HOST:-<host>}  # re-apply our config edits"
        echo "(or run \`bootstrap\` to do all four in one go.)"
        ;;

    install-binaries)
        echo "=== Building + installing drdds_recv on NOS ==="
        # Verify source synced.
        SRC_DIR="${DEPLOY_DIR}/dimos/robot/deeprobotics/m20/drdds_bridge/cpp"
        if ! remote_ssh "test -f ${SRC_DIR}/drdds_recv.cpp"; then
            echo "ERROR: source not on NOS at ${SRC_DIR}. Run \`$0 sync\` first." >&2
            exit 1
        fi
        # Build under a fresh build dir owned by the user. CMakeLists is at
        # ../, so we configure once and rebuild as needed. We build:
        #   - drdds_recv (the runtime daemon, installed to /opt)
        #   - nav_cmd_pub (smartnav binary, picked up via deploy.sh sync's
        #     symlink at result/bin/nav_cmd_pub)
        echo "  cmake configure (idempotent) + build drdds_recv + nav_cmd_pub..."
        remote_ssh "
            set -e
            cd ${SRC_DIR}
            mkdir -p build
            cd build
            if [ ! -f Makefile ] && [ ! -f build.ninja ]; then
                cmake .. >/tmp/drdds_recv_cmake.log 2>&1
            fi
            make drdds_recv nav_cmd_pub -j2 2>&1 | tail -20
            ls -la drdds_recv nav_cmd_pub
        "
        echo "  installing drdds_recv to /opt/drdds_bridge/lib/drdds_bridge/..."
        remote_sudo "mkdir -p /opt/drdds_bridge/lib/drdds_bridge"
        remote_sudo "systemctl stop drdds-recv 2>/dev/null || true"
        remote_sudo "install -o root -g root -m 0755 ${SRC_DIR}/build/drdds_recv /opt/drdds_bridge/lib/drdds_bridge/drdds_recv"
        remote_sudo "systemctl start drdds-recv"
        sleep 3
        echo "  drdds-recv status:"
        remote_ssh "systemctl is-active drdds-recv && tail -3 /var/log/drdds_recv.log 2>/dev/null | strings | head -3"
        echo "  nav_cmd_pub built at ${SRC_DIR}/build/nav_cmd_pub (symlinked via \`sync\`)"
        echo "=== Binary install complete ==="
        ;;

    restore-rsdriver-config)
        echo "=== Restoring rsdriver config edits (post-OTA) ==="
        # OTA wipes our edits to /opt/robot/share/node_driver/config/config.yaml.
        # Two settings must be flipped back:
        #   - send_separately: false → true   (we want per-lidar topics, not merged)
        #   - ts_first_point: true → false    (FAST-LIO assumes scan_END timestamps; per-lidar applies)
        # Calibration values (per-lidar pitch/yaw/roll) are NOT touched — they're
        # factory-set per dog and the OTA preserves them inside config.yaml.
        CONFIG_PATH=/opt/robot/share/node_driver/config/config.yaml
        if ! remote_ssh "test -f ${CONFIG_PATH}"; then
            echo "ERROR: ${CONFIG_PATH} not found on NOS." >&2
            exit 1
        fi
        # Snapshot before edit so a botched sed leaves a recovery copy.
        remote_sudo "cp -n ${CONFIG_PATH} ${CONFIG_PATH}.pre-restore || true"
        echo "  flipping send_separately to true..."
        remote_sudo "sed -i 's/^\\(\\s*send_separately:\\s*\\)false/\\1true/' ${CONFIG_PATH}"
        echo "  flipping ts_first_point to false (both lidars)..."
        remote_sudo "sed -i 's/^\\(\\s*ts_first_point:\\s*\\)true/\\1false/g' ${CONFIG_PATH}"
        echo "  diff vs .pre-restore:"
        remote_ssh "diff ${CONFIG_PATH}.pre-restore ${CONFIG_PATH} 2>&1 | head -20" || true
        echo "  restarting rsdriver..."
        remote_sudo "systemctl restart rsdriver"
        sleep 5
        echo "  rsdriver status: $(remote_ssh 'systemctl is-active rsdriver')"
        echo "=== rsdriver config restored ==="
        ;;

    bootstrap)
        # End-to-end fresh-dog setup post-OTA:
        # provision (system) → sync (source) → install-binaries → restore-rsdriver-config
        echo "=== Bootstrapping fresh dog post-OTA ==="
        if [ -n "${REMOTE_HOST}" ]; then
            HOST_ARGS="--host ${REMOTE_HOST}"
        else
            HOST_ARGS=""
        fi
        "$0" provision ${HOST_ARGS}
        "$0" sync ${HOST_ARGS}
        "$0" install-binaries ${HOST_ARGS}
        "$0" restore-rsdriver-config ${HOST_ARGS}
        echo ""
        echo "=== Bootstrap complete ==="
        echo "Verify with:  $0 status ${HOST_ARGS}"
        echo "Then:         $0 start ${HOST_ARGS}"
        ;;

    backup-customizations)
        # Wraps scripts/backup_m20_customizations.sh, exposing it via deploy.sh
        # so the OTA-prep workflow lives next to the rest of the deploy commands.
        echo "=== Snapshotting NOS customizations to ~/m20_backup_<ts>/ ==="
        if [ -n "${REMOTE_HOST}" ]; then
            NOS_HOST="${REMOTE_HOST}" "${SCRIPT_DIR}/scripts/backup_m20_customizations.sh"
        else
            "${SCRIPT_DIR}/scripts/backup_m20_customizations.sh"
        fi
        ;;

    start)
        echo "=== Starting smartnav ==="
        remote_ssh "
            # Clear old log first so the readiness poll only sees this boot.
            : > ${SMARTNAV_LOG}
            cd ${DEPLOY_DIR} && ${VENV}/bin/python -m ${SMARTNAV_MODULE} > ${SMARTNAV_LOG} 2>&1 &
        "
        # Poll for readiness signals instead of a blind 30s sleep.
        # We require: (a) nav_cmd_pub matched=1 (FastDDS path to AOS alive)
        #             (b) first lidar frame flowing through drdds_bridge
        echo -n "  Waiting for startup"
        for i in $(seq 1 40); do
            echo -n "."
            sleep 0.5
            ready=$(remote_ssh "grep -ac 'matched=1' ${SMARTNAV_LOG} 2>/dev/null" || echo 0)
            lidar=$(remote_ssh "grep -ac 'drdds_bridge.*lidar #' ${SMARTNAV_LOG} 2>/dev/null" || echo 0)
            if [ "${ready}" -ge 1 ] && [ "${lidar}" -ge 1 ]; then
                echo " ready (${i}x 0.5s)"
                break
            fi
        done
        echo "  Status:"
        remote_ssh "grep -a -E 'matched|keyframe|Running' ${SMARTNAV_LOG} | grep -v sec_nsec | grep -v wireframe | head -5" || true
        echo "=== Started ==="
        ;;

    stop)
        echo "=== Stopping smartnav ==="
        # Graceful shutdown first (sends sit-down command to robot).
        remote_ssh "pkill -f m20_smartnav 2>/dev/null" || true
        # Poll for process exit (100ms), cap 5s.
        for i in $(seq 1 50); do
            sleep 0.1
            alive=$(remote_ssh "pgrep -f m20_smartnav 2>/dev/null | wc -l" || echo 0)
            [ "${alive}" = "0" ] && break
        done
        # Force-kill if still alive, then release ports.
        remote_ssh "pkill -9 -f m20_smartnav 2>/dev/null; pkill -9 -f dimos-venv 2>/dev/null; fuser -k 9877/tcp 9877/tcp 3030/tcp 7779/tcp 2>/dev/null" || true
        echo "=== Stopped ==="
        ;;

    restart)
        if [ -n "${REMOTE_HOST}" ]; then
            "$0" stop --host "${REMOTE_HOST}"
            "$0" start --host "${REMOTE_HOST}"
        else
            "$0" stop
            "$0" start
        fi
        ;;

    viewer)
        echo "=== Starting dimos-viewer ==="
        pkill -f "rerun" 2>/dev/null || true
        pkill -f "ssh.*-L.*9877.*${JUMP_HOST}" 2>/dev/null || true
        sleep 1
        # Set up SSH tunnels
        ssh -f -N -o ConnectTimeout=10 ${SSH_OPTS} \
            -L 9877:${NOS_HOST}:9877 \
            -L 7779:${NOS_HOST}:7779 \
            -L 3030:${NOS_HOST}:3030 \
            "${NOS_USER}@${NOS_HOST}" 2>/dev/null
        sleep 1
        # Launch viewer
        DIMOS_DEBUG=1 "${DIMOS_ROOT}/.venv/bin/rerun" \
            --connect "rerun+http://127.0.0.1:9877/proxy" \
            --ws-url "ws://127.0.0.1:3030/ws" &
        echo "  dimos-viewer PID: $!"
        echo "=== Viewer started ==="
        ;;

    bridge-start)
        echo "=== Restarting drdds bridge (with rsdriver ordering) ==="

        echo "  Stopping rsdriver..."
        remote_ssh "sudo systemctl stop rsdriver" || true
        sleep 2

        echo "  Restarting drdds-recv (and cleaning stale SHM)..."
        remote_ssh "sudo systemctl stop drdds-recv" || true
        remote_ssh "sudo /usr/local/sbin/m20-clean-shm" || true
        remote_ssh "sudo systemctl start drdds-recv"
        sleep 3

        echo "  Starting rsdriver (30s init delay)..."
        remote_ssh "sudo systemctl start rsdriver"
        sleep 35

        echo "  Verifying bridge data flow..."
        remote_ssh "tail -3 /var/log/drdds_recv.log 2>/dev/null" || true

        echo "=== Bridge running ==="
        ;;

    bridge-stop)
        echo "Stopping drdds bridge..."
        remote_ssh "sudo systemctl stop drdds-recv" || true
        echo "Bridge stopped."
        ;;

    status)
        echo "=== systemd services ==="
        remote_ssh "for s in drdds-recv rsdriver m20-nav-net; do printf '  %-20s %s\n' \$s \$(systemctl is-active \$s); done"
        echo ""
        echo "=== /nix bind mount ==="
        remote_ssh "mountpoint -q /nix && echo '  /nix mounted OK' || echo '  /nix NOT mounted'"
        echo ""
        echo "=== dimos processes ==="
        remote_ssh "pgrep -a -f m20_smartnav 2>/dev/null" || echo "  smartnav not running"
        remote_ssh "pgrep -a nav_cmd_pub 2>/dev/null" || echo "  nav_cmd_pub not running"
        remote_ssh "pgrep -a drdds_lidar 2>/dev/null" || echo "  lidar bridge not running"
        remote_ssh "pgrep -a arise_slam 2>/dev/null" || echo "  arise_slam not running"
        echo ""
        echo "=== Ports ==="
        remote_ssh "ss -tlnp 2>/dev/null | grep -E '9877|7779|3030'" || echo "  no ports listening"
        echo ""
        echo "=== AOS services ==="
        remote_ssh "ssh user@${AOS_INTERNAL} 'for s in basic_server ctrlmcu rl_deploy rsdriver; do printf \"  %-15s %s\n\" \$s \$(systemctl is-active \$s); done'" 2>/dev/null || echo "  (could not reach AOS)"
        ;;

    *)
        echo "Usage: $0 <command> [--host <hostname>]"
        echo ""
        echo "Per-OTA / new-dog commands:"
        echo "  bootstrap                - provision + sync + install-binaries + restore-rsdriver-config"
        echo "  provision                - install systemd units, fstab, sudoers, sysctl, swap"
        echo "  sync                     - rsync dimos source to NOS + fix nix symlinks"
        echo "  install-binaries         - build drdds_recv from source on NOS, install to /opt"
        echo "  restore-rsdriver-config  - re-apply send_separately:true + ts_first_point:false"
        echo "  backup-customizations    - snapshot NOS state to ~/m20_backup_<ts>/ (run pre-OTA)"
        echo ""
        echo "Runtime commands:"
        echo "  start                    - start smartnav on NOS"
        echo "  stop                     - stop smartnav"
        echo "  restart                  - stop + start"
        echo "  viewer                   - start SSH tunnels + dimos-viewer locally"
        echo "  bridge-start             - restart drdds SHM bridge (with rsdriver ordering)"
        echo "  bridge-stop              - stop drdds bridge"
        echo "  status                   - show status"
        echo ""
        echo "Typical flows:"
        echo "  Fresh post-OTA dog:    $0 bootstrap --host m20-XXX-gogo"
        echo "  Quick code update:     $0 sync --host m20-770-gogo && $0 restart --host m20-770-gogo"
        echo "  After reboot:          $0 start --host m20-770-gogo && $0 viewer --host m20-770-gogo"
        echo "  Pre-OTA snapshot:      $0 backup-customizations --host m20-770-gogo"
        echo ""
        echo "Options:"
        echo "  --host <hostname>  Access robot via Tailscale (e.g., m20-770-gogo)"
        exit 1
        ;;
esac
