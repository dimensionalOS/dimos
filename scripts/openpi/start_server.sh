#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/../.." && pwd)"
openpi_dir="${OPENPI_ROOT:-$repo_dir/../openpi-yam}"
checkpoint_dir="${YAM_CHECKPOINT:-$repo_dir/../robotwin-pi05-checkpoint-eval/pi0.5-yam}"

if [[ ! -x "$openpi_dir/.venv/bin/python" ]]; then
    echo "OpenPI Python environment missing: $openpi_dir/.venv/bin/python" >&2
    exit 1
fi
if [[ ! -d "$checkpoint_dir/params" ]]; then
    echo "YAM checkpoint parameters missing: $checkpoint_dir/params" >&2
    exit 1
fi
export PYTHONPATH="$openpi_dir/src:$openpi_dir/packages/openpi-client/src${PYTHONPATH:+:$PYTHONPATH}"
export XLA_FLAGS="${XLA_FLAGS:---xla_gpu_autotune_level=0}"
if [[ -d /usr/lib/aarch64-linux-gnu/nvidia ]]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/nvidia${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec "$openpi_dir/.venv/bin/python" -u "$script_dir/serve_yam.py" \
    --checkpoint "$checkpoint_dir" "$@"
