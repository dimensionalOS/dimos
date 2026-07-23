#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)
DOCKERFILE="$REPO_ROOT/dimos/manipulation/grasping/docker_context/Dockerfile"

die() {
    printf 'build.sh: %s\n' "$1" >&2
    exit 2
}

# Keep the Dockerfile and context fixed. Values for known Docker options are
# accepted, but a bare non-option is never treated as a second build context.
docker_args=()
needs_value=0
for arg in "$@"; do
    if (( needs_value )); then
        docker_args+=("$arg")
        needs_value=0
        continue
    fi

    case "$arg" in
        --)
            die "-- is not allowed; the Dockerfile and context are fixed"
            ;;
        -f|--file|--file=*|-f?*|--file?*)
            die "overriding the Dockerfile is not allowed"
            ;;
        --add-host|--build-arg|--build-context|--cache-from|--cache-to|\
        --cgroup-parent|--cpu-period|--cpu-quota|--cpu-shares|--cpuset-cpus|\
        --cpuset-mems|--iidfile|--label|--memory|--memory-swap|\
        --memory-swappiness|--network|--no-cache-filter|--output|\
        --platform|--progress|--secret|--shm-size|--ssh|--tag|--target|\
        --ulimit|-t|-o)
            docker_args+=("$arg")
            needs_value=1
            ;;
        --*=*)
            docker_args+=("$arg")
            ;;
        -*)
            docker_args+=("$arg")
            ;;
        *)
            die "positional Docker build arguments are not allowed: $arg"
            ;;
    esac
done

if (( needs_value )); then
    die "missing value for Docker build option"
fi

cd "$REPO_ROOT"
uv run python - <<'PY'
from pathlib import Path

from dimos.utils.data import get_data

asset = get_data("models_graspgen")
checkpoint = Path(asset) / "checkpoints" / "graspgen_robotiq_2f_140_gen.pth"
if not checkpoint.is_file():
    raise SystemExit(f"Expected GraspGen checkpoint is missing: {checkpoint}")
print(f"Using GraspGen data asset: {asset}")
print(f"Found checkpoint: {checkpoint}")
PY

docker build --file "$DOCKERFILE" "${docker_args[@]}" "$REPO_ROOT"
