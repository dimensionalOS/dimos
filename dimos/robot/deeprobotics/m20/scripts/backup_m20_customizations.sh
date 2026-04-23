#!/bin/bash
# Snapshot all M20 NOS customizations to a tarball on the Mac.
# Run BEFORE upgrading system software to V1.1.8.5.
# After upgrade, inspect the tarball to see what needs restoring.

set -e

NOS_HOST="${NOS_HOST:-m20-770-gogo}"
NOS_USER="${NOS_USER:-user}"
NOS_INTERNAL="10.21.31.106"
SSH_OPTS="-o ProxyJump=${NOS_USER}@${NOS_HOST}"

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT_DIR="${HOME}/m20_backup_${STAMP}"
mkdir -p "$OUT_DIR"

echo "[backup] snapshot → ${OUT_DIR}"

echo "[backup] 1/8 kernel cmdline + /proc info"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'cat /proc/cmdline' > "${OUT_DIR}/proc_cmdline.txt"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'uname -a; cat /etc/os-release' > "${OUT_DIR}/system_info.txt"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'lscpu' > "${OUT_DIR}/lscpu.txt"

echo "[backup] 2/8 /etc/fstab + /etc/ systemd units + cmdline"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'cat /etc/fstab' > "${OUT_DIR}/etc_fstab.txt"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'ls -la /etc/systemd/system/ /etc/systemd/system/*.d/ 2>/dev/null' > "${OUT_DIR}/etc_systemd_ls.txt"

for unit in drdds-recv.service drdds-bridge.service m20-nav-net.service; do
    ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" "cat /etc/systemd/system/${unit} 2>/dev/null" > "${OUT_DIR}/systemd_${unit}.txt" || true
done
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" "cat /etc/systemd/system/rsdriver.service.d/10-drdds-order.conf 2>/dev/null" > "${OUT_DIR}/systemd_rsdriver_dropin.txt" || true
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" "cat /etc/systemd/system/ssh.service.d/oom.conf 2>/dev/null" > "${OUT_DIR}/systemd_ssh_oom.txt" || true

echo "[backup] 3/8 rsdriver config (lives in rootfs — will definitely be overwritten)"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'cat /opt/robot/share/node_driver/config/config.yaml' > "${OUT_DIR}/rsdriver_config.yaml"

echo "[backup] 4/8 nix GC roots (our pinned store paths)"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'ls -la /nix/var/nix/gcroots/custom/ 2>/dev/null; for f in /nix/var/nix/gcroots/custom/*; do echo "--- $f ---"; readlink -f "$f"; done' > "${OUT_DIR}/nix_gcroots.txt"

echo "[backup] 5/8 nix store summary (sizes/counts — full store too big to tar)"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'df -h /var/opt/robot/data; echo ---count---; ls /nix/store | wc -l; echo ---drdds/slam builds---; ls -d /nix/store/*-drdds_lidar_bridge-* /nix/store/*-smartnav-* /nix/store/*-fastlio2* 2>/dev/null' > "${OUT_DIR}/nix_store_summary.txt"

echo "[backup] 6/8 launch scripts in /tmp"
for s in start_smartnav.sh start_smartnav_nonav.sh start_airy.sh wrap_build.sh; do
    ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" "cat /tmp/${s} 2>/dev/null" > "${OUT_DIR}/tmp_${s}" || true
done

echo "[backup] 7/8 procs + network state (for post-upgrade comparison)"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'pgrep -fa drdds_recv rsdriver yesense handler; echo; ip -4 addr show; echo; ip route' > "${OUT_DIR}/procs_and_network.txt"

echo "[backup] 8/8 current rsdriver drdds topics + DIFOP info for reference"
ssh $SSH_OPTS "${NOS_USER}@${NOS_INTERNAL}" 'ls /dev/shm/drdds_bridge_* /dev/shm/fastrtps_* 2>/dev/null; echo; ls -la /var/opt/robot/data/dimos/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/result/bin/ 2>/dev/null' > "${OUT_DIR}/shm_and_binlinks.txt"

echo
echo "[backup] done. snapshot at: ${OUT_DIR}"
echo
echo "Files:"
ls -la "$OUT_DIR"
