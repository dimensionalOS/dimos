#!/usr/bin/env bash
set -euo pipefail

# Copy the C-prime checkout and the calibration inputs over SSH. This does not
# install software or command motors on either machine.

usage() {
  cat <<'EOF'
Usage: scripts/openyam/transfer_cprime.sh [USER@HOST] [--remote-root ABS_PATH] [--calibration-root ABS_PATH] [--dry-run]

Copies files into ~/openyam-cprime on the destination by default. Existing
files with the same names are updated; other destination files are left alone.
If USER@HOST is omitted, the script asks for it. Use an SSH config alias for
nonstandard ports. See sanity_reproduce.md for the later installation steps.
EOF
}

die() {
  printf 'transfer_cprime: %s\n' "$*" >&2
  exit 1
}

ssh_target=""
if [[ $# -gt 0 && "$1" != -* ]]; then
  ssh_target=$1
  shift
fi
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
calibration_dir="$HOME/calibration/openyam_camera_calibration_20261027_i3h7w6ox"
remote_root=""
dry_run=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --remote-root|--calibration-root)
      [[ $# -ge 2 ]] || die "$1 needs a path"
      if [[ "$1" == --remote-root ]]; then remote_root=$2; else calibration_dir=$2; fi
      shift 2
      ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
if [[ -z "$ssh_target" ]]; then
  read -r -p 'Destination SSH target (USER@HOST): ' ssh_target
fi
[[ "$ssh_target" =~ ^[[:alnum:]_.@:-]+$ && "$ssh_target" != -* ]] || die "invalid SSH target"

session_name=openyam_camera_calibration_20261027_i3h7w6ox
model_dir="$repo_dir/data/dual_openyam_abc_box_v2"
experiment_dir="$calibration_dir/gravity_friction_identification"
checker="$experiment_dir/bounded_integral_stability/check_workspace.py"
for path in \
  "$repo_dir/scripts/openyam/demo_dimos_joint_offset.py" \
  "$repo_dir/scripts/openyam/demo_test_dimos_joint_offset.py" \
  "$repo_dir/sanity_reproduce.md" \
  "$model_dir/assets/base.stl" \
  "$checker" \
  "$calibration_dir/robot_model.urdf" \
  "$calibration_dir/results/calibration_bundle.json"; do
  [[ -f "$path" ]] || die "required file missing: $path"
done

if [[ $dry_run == 1 ]]; then
  printf 'SSH target: %s\n' "$ssh_target"
  printf 'DimOS source: %s\n' "$repo_dir"
  printf 'Calibration source: %s\n' "$calibration_dir"
  printf 'Destination root: %s\n' "${remote_root:-<remote home>/openyam-cprime}"
  printf 'Copies: DimOS files (except .venv, logs and caches), workspace checker, URDF, calibration bundle, model meshes.\n'
  printf 'Installs packages: no; runs motors: no\n'
  exit 0
fi

for tool in ssh rsync mktemp; do command -v "$tool" >/dev/null || die "$tool is required"; done
temporary=$(mktemp -d)
control_path="$temporary/ssh-control"
ssh_cmd=(ssh -o ControlMaster=auto -o ControlPath="$control_path" -o ControlPersist=10m)
rsync_ssh="ssh -o ControlMaster=auto -o ControlPath=$control_path -o ControlPersist=10m"
cleanup() {
  "${ssh_cmd[@]}" -O exit "$ssh_target" >/dev/null 2>&1 || true
  rm -rf "$temporary"
}
trap cleanup EXIT

printf 'Connecting to %s (enter the destination password if prompted)...\n' "$ssh_target"
"${ssh_cmd[@]}" -MNf "$ssh_target"
"${ssh_cmd[@]}" "$ssh_target" 'command -v rsync >/dev/null' \
  || die "destination needs rsync"
remote_home=$("${ssh_cmd[@]}" "$ssh_target" 'printf %s "$HOME"')
[[ "$remote_home" == /* ]] || die "could not determine destination home directory"
remote_root=${remote_root:-$remote_home/openyam-cprime}
remote_root=${remote_root%/}
[[ "$remote_root" =~ ^/[[:alnum:]_./-]+$ && "$remote_root" != / ]] \
  || die "--remote-root must be an absolute path using letters, numbers, _, -, . and /"

remote_calibration="$remote_root/calibration/$session_name"
remote_experiment="$remote_calibration/gravity_friction_identification"
"${ssh_cmd[@]}" "$ssh_target" "mkdir -p \
  '$remote_root/dimos' \
  '$remote_calibration/results' \
  '$remote_experiment/bounded_integral_stability' \
  '$remote_experiment/geometry/dual_openyam_abc_box'"

printf 'Copying DimOS files...\n'
rsync -a --partial --info=progress2 -e "$rsync_ssh" \
  --exclude='/.venv/' --exclude='/logs/' \
  --exclude='/.pytest_cache/' --exclude='/.ruff_cache/' \
  --exclude='__pycache__/' --exclude='*.pyc' \
  "$repo_dir/" "$ssh_target:$remote_root/dimos/"

printf 'Copying calibration inputs...\n'
rsync -a -e "$rsync_ssh" "$calibration_dir/robot_model.urdf" \
  "$ssh_target:$remote_calibration/"
rsync -a -e "$rsync_ssh" "$calibration_dir/results/calibration_bundle.json" \
  "$ssh_target:$remote_calibration/results/"
rsync -a -e "$rsync_ssh" "$checker" \
  "$ssh_target:$remote_experiment/bounded_integral_stability/"
rsync -a -e "$rsync_ssh" \
  "$repo_dir/scripts/openyam/demo_dimos_joint_offset.py" \
  "$repo_dir/scripts/openyam/demo_test_dimos_joint_offset.py" \
  "$ssh_target:$remote_experiment/"
rsync -a -e "$rsync_ssh" "$model_dir/" \
  "$ssh_target:$remote_experiment/geometry/dual_openyam_abc_box/"

"${ssh_cmd[@]}" "$ssh_target" "test -f '$remote_root/dimos/sanity_reproduce.md' && \
  test -f '$remote_experiment/demo_dimos_joint_offset.py' && \
  test -f '$remote_experiment/geometry/dual_openyam_abc_box/assets/base.stl' && \
  test -f '$remote_calibration/results/calibration_bundle.json'" \
  || die "copy finished but a required destination file is missing"
printf 'Copy complete: %s\nNext steps: %s/dimos/sanity_reproduce.md\n' \
  "$remote_root" "$remote_root"
