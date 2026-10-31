#!/usr/bin/env bash
set -euo pipefail

# Copy the tested OpenYAM sources and calibration inputs. Installation is done
# later on the destination using sanity_reproduce.md.

usage() {
  cat <<'EOF'
Usage: scripts/openyam/transfer_cprime.sh USER@HOST [--remote-root ABS_PATH] [--calibration-root ABS_PATH] [--dry-run]

Creates a new ~/openyam-cprime directory on the destination by default.
The destination needs SSH, Git, and rsync. The script does not install packages
or send CAN commands. Use an SSH config alias if the port is not 22.
EOF
}

die() {
  printf 'transfer_cprime: %s\n' "$*" >&2
  exit 1
}

[[ $# -gt 0 ]] || { usage >&2; exit 2; }
case "$1" in
  -h|--help) usage; exit 0 ;;
esac
ssh_target=$1
shift
[[ "$ssh_target" =~ ^[[:alnum:]_.@:-]+$ && "$ssh_target" != -* ]] || die "invalid SSH target"

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

branch=codex/feat/openyam-full-tracking
tested_commit=c7ec34c94
can_branch=fix/openyam-anvil-parity
can_commit=20572f3a41dd98d692da44556de0a0d10c2c8d9d
session_name=openyam_camera_calibration_20261027_i3h7w6ox
model_dir="$repo_dir/data/dual_openyam_abc_box_v2"
checker="$calibration_dir/gravity_friction_identification/bounded_integral_stability/check_workspace.py"

[[ -d "$repo_dir/.git" ]] || die "DimOS Git checkout missing: $repo_dir"
[[ $(git -C "$repo_dir" branch --show-current) == "$branch" ]] || die "run from the C-prime branch ($branch)"
git -C "$repo_dir" merge-base --is-ancestor "$tested_commit" HEAD || die "tested commit $tested_commit is missing"
[[ -z $(git -C "$repo_dir" status --porcelain) ]] || die "commit or remove uncommitted files before transfer"
for path in \
  "$model_dir/assets/base.stl" \
  "$checker" \
  "$calibration_dir/robot_model.urdf" \
  "$calibration_dir/results/calibration_bundle.json"; do
  [[ -f "$path" ]] || die "required file missing: $path"
done

if [[ $dry_run == 1 ]]; then
  printf 'SSH target: %s\n' "$ssh_target"
  printf 'DimOS branch: %s (%s)\n' "$branch" "$(git -C "$repo_dir" rev-parse --short HEAD)"
  printf 'Calibration source: %s\n' "$calibration_dir"
  printf 'Destination root: %s\n' "${remote_root:-<remote home>/openyam-cprime}"
  printf 'Copies: DimOS Git history, pinned CAN fork source, OpenYAM model assets, workspace checker, URDF, calibration bundle.\n'
  printf 'Installs packages: no; runs motors: no\n'
  exit 0
fi

for tool in git ssh rsync mktemp; do command -v "$tool" >/dev/null || die "$tool is required"; done
ssh "$ssh_target" 'command -v bash >/dev/null && command -v git >/dev/null && command -v rsync >/dev/null' \
  || die "destination needs Bash, Git, and rsync"
remote_home=$(ssh "$ssh_target" 'printf %s "$HOME"')
[[ "$remote_home" == /* ]] || die "could not determine destination home directory"
remote_root=${remote_root:-$remote_home/openyam-cprime}
remote_root=${remote_root%/}
[[ "$remote_root" =~ ^/[[:alnum:]_./-]+$ && "$remote_root" != / ]] \
  || die "--remote-root must be an absolute path using letters, numbers, _, -, . and /"
ssh "$ssh_target" "test ! -e '$remote_root'" || die "destination already exists: $remote_root"

temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT
source_head=$(git -C "$repo_dir" rev-parse HEAD)
printf 'Packing DimOS branch %s...\n' "$branch"
git -C "$repo_dir" bundle create "$temporary/dimos.bundle" "$branch"
git -C "$repo_dir" bundle verify "$temporary/dimos.bundle" >/dev/null

printf 'Packing CAN fork commit %s...\n' "$can_commit"
git clone --quiet --single-branch --branch "$can_branch" \
  https://github.com/Kaweees/can-motor-control.git "$temporary/can-source"
git -C "$temporary/can-source" checkout --quiet --detach "$can_commit"
git -C "$temporary/can-source" branch -f "$can_branch" "$can_commit" >/dev/null
git -C "$temporary/can-source" bundle create "$temporary/can.bundle" "$can_branch"
git -C "$temporary/can-source" bundle verify "$temporary/can.bundle" >/dev/null

ssh "$ssh_target" "mkdir -p '$remote_root/bundles'"
rsync -a --partial "$temporary/dimos.bundle" "$temporary/can.bundle" \
  "$ssh_target:$remote_root/bundles/"

ssh "$ssh_target" "bash -s -- '$remote_root' '$branch' '$source_head' '$can_branch' '$can_commit' '$session_name'" <<'REMOTE'
set -euo pipefail
root=$1
branch=$2
dimos_commit=$3
can_branch=$4
can_commit=$5
session_name=$6
GIT_LFS_SKIP_SMUDGE=1 git clone --quiet --branch "$branch" "$root/bundles/dimos.bundle" "$root/dimos"
git -C "$root/dimos" remote set-url origin https://github.com/dimensionalOS/dimos.git
git clone --quiet --branch "$can_branch" "$root/bundles/can.bundle" "$root/can-motor-control"
git -C "$root/can-motor-control" remote set-url origin https://github.com/Kaweees/can-motor-control.git
[[ $(git -C "$root/dimos" rev-parse HEAD) == "$dimos_commit" ]]
[[ $(git -C "$root/can-motor-control" rev-parse HEAD) == "$can_commit" ]]
mkdir -p \
  "$root/dimos/data/dual_openyam_abc_box_v2" \
  "$root/calibration/$session_name/results" \
  "$root/calibration/$session_name/gravity_friction_identification/bounded_integral_stability" \
  "$root/calibration/$session_name/gravity_friction_identification/geometry"
REMOTE

remote_calibration="$remote_root/calibration/$session_name"
rsync -a "$model_dir/" "$ssh_target:$remote_root/dimos/data/dual_openyam_abc_box_v2/"
rsync -a "$calibration_dir/robot_model.urdf" "$ssh_target:$remote_calibration/"
rsync -a "$calibration_dir/results/calibration_bundle.json" "$ssh_target:$remote_calibration/results/"
rsync -a "$checker" \
  "$ssh_target:$remote_calibration/gravity_friction_identification/bounded_integral_stability/"

ssh "$ssh_target" "bash -s -- '$remote_root' '$session_name'" <<'REMOTE'
set -euo pipefail
root=$1
session_name=$2
calibration="$root/calibration/$session_name"
experiment="$calibration/gravity_friction_identification"
ln -s "$root/dimos/scripts/openyam/demo_dimos_joint_offset.py" "$experiment/demo_dimos_joint_offset.py"
ln -s "$root/dimos/scripts/openyam/demo_test_dimos_joint_offset.py" "$experiment/test_dimos_joint_offset.py"
ln -s "$root/dimos/data/dual_openyam_abc_box_v2" "$experiment/geometry/dual_openyam_abc_box"
test -f "$root/dimos/sanity_reproduce.md"
test -f "$experiment/bounded_integral_stability/check_workspace.py"
test -f "$experiment/geometry/dual_openyam_abc_box/assets/base.stl"
test -f "$calibration/results/calibration_bundle.json"
printf 'Transfer complete: %s\nNext steps: %s\n' "$root" "$root/dimos/sanity_reproduce.md"
REMOTE
