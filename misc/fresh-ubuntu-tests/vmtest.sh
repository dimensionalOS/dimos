#!/usr/bin/env bash
#
# vmtest.sh -- run the documented dimos install + tests on a fresh, official,
# unmodified Ubuntu Desktop 24.04 VM (VirtualBox).
#
# Why: CI tests dimos from a maintainer's angle (pre-built images, cached deps).
# It does not verify that a brand-new user following docs/installation/ubuntu.md
# on a clean machine can actually install and run dimos. This tool does, by
# replaying that flow (the suite/ next to this script) in a fresh VM.
#
# Run by hand, occasionally. It is slow; that's fine.
#
#   ./vmtest.sh build    download + verify the ISO, install the OS, snapshot "golden" (once)
#   ./vmtest.sh run      clone golden, run the doc flow, report PASS/FAIL
#   ./vmtest.sh clean    delete leftover run clones and logs (keeps the ISO + golden VM)
#
# Everything lives in a gitignored ./cache subdir next to this script. To change
# what is tested or the VM size, edit the constants below -- there are no flags.

set -euo pipefail

# VBoxHeadless otherwise drops a <timestamp>-VBoxHeadless-<pid>.log in the cwd on
# every VM start; send that default process log to nowhere. The VM's own
# VBox.log (in the VM folder) is unaffected.
export VBOX_RELEASE_LOG_DEST=nofile

# --- constants ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; readonly SCRIPT_DIR
readonly RELEASE="24.04"
readonly VM="dimos-vmtest-base"
readonly MEM=12288        # MiB
readonly CPUS=8
readonly DISK=40960       # MiB (dynamic; only grows as used)
readonly SSH_PORT=2222
readonly VM_USER="tester"
readonly VM_PASS="tester"

readonly CACHE="$SCRIPT_DIR/cache"
readonly ISO_DIR="$CACHE/iso"
readonly VMS_DIR="$CACHE/vms"
readonly SSH_DIR="$CACHE/ssh"
readonly LOG_DIR="$CACHE/logs"
readonly SSH_KEY="$SSH_DIR/id_ed25519"

# --- helpers ---
log()  { printf '\033[1;34m[vmtest]\033[0m %s\n' "$*" >&2; }
warn() { printf '\033[1;33m[vmtest]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[vmtest] error:\033[0m %s\n' "$*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"; }

SSH_OPTS=(-i "$SSH_KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
          -o ConnectTimeout=5 -o LogLevel=ERROR)
ssh_vm() { ssh "${SSH_OPTS[@]}" -p "$SSH_PORT" "$VM_USER@127.0.0.1" "$@"; }
scp_to() { scp "${SSH_OPTS[@]}" -P "$SSH_PORT" "$1" "$VM_USER@127.0.0.1:$2"; }

vm_exists()  { VBoxManage showvminfo "$1" >/dev/null 2>&1; }
vm_state()   { VBoxManage showvminfo "$1" --machinereadable 2>/dev/null | sed -n 's/^VMState="\(.*\)"/\1/p'; }
run_clones() { VBoxManage list vms 2>/dev/null | sed -n 's/^"\(dimos-vmtest-run-[^"]*\)".*/\1/p'; }

golden_exists() {
  vm_exists "$VM" && VBoxManage snapshot "$VM" list --machinereadable 2>/dev/null \
    | grep -q 'SnapshotName[^=]*="golden"'
}

wait_for_ssh() {  # $1 = timeout seconds
  local deadline=$(( SECONDS + $1 ))
  while (( SECONDS < deadline )); do
    ssh_vm true >/dev/null 2>&1 && return 0
    sleep 5
  done
  return 1
}

wait_poweroff() {  # $1 = vm, $2 = timeout
  local deadline=$(( SECONDS + $2 )) s
  while (( SECONDS < deadline )); do
    s="$(vm_state "$1")"
    [[ "$s" == "poweroff" || "$s" == "aborted" || -z "$s" ]] && return 0
    sleep 3
  done
  return 1
}

power_off() {  # graceful, then hard
  local vm="$1"
  [[ "$(vm_state "$vm")" == "running" ]] || return 0
  VBoxManage controlvm "$vm" acpipowerbutton >/dev/null 2>&1 || true
  wait_poweroff "$vm" 60 && return 0
  VBoxManage controlvm "$vm" poweroff >/dev/null 2>&1 || true
  wait_poweroff "$vm" 30 || true
}

remove_vm() {  # power off + delete
  vm_exists "$1" || return 0
  power_off "$1"
  VBoxManage unregistervm "$1" --delete >/dev/null 2>&1 || true
}

ensure_iso() {
  need wget
  mkdir -p "$ISO_DIR"
  local fname="ubuntu-24.04.4-desktop-amd64.iso"
  ISO_PATH="$ISO_DIR/$fname"

  if [[ -f "$ISO_PATH" ]]; then
    return
  fi

  wget --tries=3 --continue -O "$ISO_PATH" "https://releases.ubuntu.com/$RELEASE/$fname" \
    || die "download failed"
}

ensure_sshkey() {
  need ssh-keygen
  mkdir -p "$SSH_DIR"; chmod 700 "$SSH_DIR"
  [[ -f "$SSH_KEY" ]] || ssh-keygen -t ed25519 -N '' -C 'dimos-vmtest' -f "$SSH_KEY" >/dev/null
}

# Bootstrap run as root inside the installed system by VBox's autoinstall late-
# command. Adds only what we need to drive the VM headlessly (openssh + our key +
# passwordless sudo) -- none are dimos deps, so they don't mask a dep gap. Passed
# base64-encoded: the only quoting-safe way through VBox's template substitution.
post_install_command() {
  local script
  script="$(cat <<EOF
set -e
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y openssh-server
install -d -m 700 -o $VM_USER -g $VM_USER /home/$VM_USER/.ssh
cat > /home/$VM_USER/.ssh/authorized_keys <<KEY
$(cat "$SSH_KEY.pub")
KEY
chown $VM_USER:$VM_USER /home/$VM_USER/.ssh/authorized_keys
chmod 600 /home/$VM_USER/.ssh/authorized_keys
systemctl enable ssh || true
echo "$VM_USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-$VM_USER
chmod 440 /etc/sudoers.d/90-$VM_USER
touch /etc/dimos-vmtest-ready
EOF
)"
  printf 'bash -c "echo %s | base64 -d | bash"' "$(printf '%s' "$script" | base64 -w0)"
}

# --- commands ---
cmd_build() {
  need VBoxManage
  golden_exists && die "golden image already exists; delete VM '$VM' to rebuild"
  vm_exists "$VM" && die "VM '$VM' exists without a golden snapshot; delete it to rebuild"

  ensure_iso
  ensure_sshkey
  mkdir -p "$VMS_DIR"

  log "creating VM '$VM'"
  VBoxManage createvm --name "$VM" --basefolder "$VMS_DIR" --ostype Ubuntu_64 --register
  VBoxManage modifyvm "$VM" --memory "$MEM" --cpus "$CPUS" --ioapic on --rtcuseutc on \
    --graphicscontroller vmsvga --vram 64 --audio-driver none --firmware bios
  VBoxManage modifyvm "$VM" --nic1 nat --natpf1 "ssh,tcp,127.0.0.1,$SSH_PORT,,22"
  VBoxManage createmedium disk --filename "$VMS_DIR/$VM/$VM.vdi" --size "$DISK" --format VDI
  VBoxManage storagectl "$VM" --name SATA --add sata --controller IntelAhci --portcount 2
  VBoxManage storageattach "$VM" --storagectl SATA --port 0 --device 0 --type hdd \
    --medium "$VMS_DIR/$VM/$VM.vdi"
  VBoxManage storagectl "$VM" --name IDE --add ide

  log "starting unattended install (slow: ~15-30 min)"
  VBoxManage unattended install "$VM" \
    --iso="$ISO_PATH" \
    --user="$VM_USER" --user-password="$VM_PASS" --full-user-name="Dimos Tester" \
    --hostname="dimos-vmtest.local" \
    --no-install-additions \
    --post-install-command="$(post_install_command)" \
    --start-vm=headless

  log "waiting for the installed system over SSH (up to 60 min)"
  wait_for_ssh 3600 || die "VM never became reachable over SSH"
  ssh_vm 'test -f /etc/dimos-vmtest-ready' || die "post-install marker missing; install incomplete"

  log "shutting down to snapshot a clean base"
  power_off "$VM"
  wait_poweroff "$VM" 120 || die "VM did not power off"
  VBoxManage snapshot "$VM" take golden --description "fresh Ubuntu $RELEASE desktop install"
}

CLONE=""
cleanup_clone() { [[ -n "$CLONE" ]] && remove_vm "$CLONE"; }

cmd_run() {
  need VBoxManage
  golden_exists || die "no golden image; run '$0 build' first"
  mkdir -p "$LOG_DIR"

  local stamp; stamp="$(date +%Y%m%d-%H%M%S)-$$"
  CLONE="dimos-vmtest-run-$stamp"
  local rundir="$LOG_DIR/$stamp"
  mkdir -p "$rundir"
  trap cleanup_clone EXIT

  log "cloning golden -> $CLONE"
  VBoxManage clonevm "$VM" --snapshot golden --options link \
    --name "$CLONE" --basefolder "$VMS_DIR" --register >/dev/null
  VBoxManage startvm "$CLONE" --type headless >/dev/null

  log "waiting for SSH"
  wait_for_ssh 300 || die "clone never became reachable over SSH"

  log "running suite (per-script logs -> $rundir/logs)"
  scp -r "${SSH_OPTS[@]}" -P "$SSH_PORT" "$SCRIPT_DIR/suite" "$VM_USER@127.0.0.1:suite"
  local rc=0
  ssh_vm "bash suite/run.sh" 2>&1 | tee "$rundir/run.log" || rc=${PIPESTATUS[0]}
  # pull the individual per-script logs back to the host before the clone is deleted
  scp -r "${SSH_OPTS[@]}" -P "$SSH_PORT" "$VM_USER@127.0.0.1:suite/logs" "$rundir/" 2>/dev/null || true

  if [[ "$rc" -eq 0 ]]; then
    printf '\033[1;32m[vmtest] PASS\033[0m  logs=%s\n' "$rundir" >&2
  else
    printf '\033[1;31m[vmtest] FAIL (exit %s)\033[0m  logs=%s\n' "$rc" "$rundir" >&2
  fi
  return "$rc"
}

# Remove leftover run clones and logs; keep the ISO and golden VM (the slow-to-
# rebuild caches). The SSH key is kept too -- it is paired with the golden VM.
cmd_clean() {
  need VBoxManage
  local name
  while read -r name; do
    [[ -n "$name" ]] || continue
    log "removing clone $name"
    remove_vm "$name"
  done < <(run_clones)
  rm -rf "$LOG_DIR"
  log "clean done (kept ISO + golden VM)"
}

case "${1:-}" in
  build) cmd_build ;;
  run)   cmd_run ;;
  clean) cmd_clean ;;
  *)     die "usage: $0 {build|run|clean}" ;;
esac
