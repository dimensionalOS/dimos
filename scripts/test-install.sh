#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
# Real installation acceptance. Run on a disposable host/container:
#   bash scripts/test-install.sh library|dev cpu|cuda
set -euo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
mode=${1:?expected library or dev}
backend=${2:-cpu}
case "$mode" in library|dev) ;; *) exit 2;; esac
case "$backend" in cpu|cuda) ;; *) exit 2;; esac
: "${INSTALL_TEST_ROOT:?set INSTALL_TEST_ROOT to an empty temporary directory}"
mkdir -p "$INSTALL_TEST_ROOT/logs"
project="$INSTALL_TEST_ROOT/$mode"
export GIT_LFS_SKIP_SMUDGE=1
export UV_PYTHON_PREFERENCE=only-managed
unset VIRTUAL_ENV PYTHONPATH

if [[ "$mode" == dev ]]; then
    expected_commit=$(git -C "$repo" rev-parse HEAD)
    git clone --no-hardlinks --no-checkout "$repo" "$project"
    git -C "$project" checkout --detach "$expected_commit"
fi
args=(--non-interactive --no-nix --no-sysctl --skip-tests --mode "$mode" --project-dir "$project")
if [[ "$backend" == cpu ]]; then
    args+=(--no-cuda)
    export CUDA_VISIBLE_DEVICES=""
else
    # CUDA success must be demonstrated by a real GPU, not inferred from a label.
    nvidia-smi
    args+=(--extras "all,cuda")
fi
# Exercise the same streamed input used by the README's curl | bash command.
cat "$repo/scripts/install.sh" | /bin/bash -s -- "${args[@]}" 2>&1 | tee "$INSTALL_TEST_ROOT/logs/install.log"

# Run outside the checkout with no interactive startup files or inherited venv.
cd "$INSTALL_TEST_ROOT"
/bin/bash --noprofile --norc -s -- "$project" "$mode" "$backend" <<'VERIFY' 2>&1 | tee "$INSTALL_TEST_ROOT/logs/verify.log"
set -euo pipefail
source "$1/.venv/bin/activate"
dimos --help
dimos list
python - "$1" "$2" "$3" <<'PY'
import importlib.metadata
import os
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import open3d
from turbojpeg import TurboJPEG
import torch

image = np.zeros((8, 8, 3), dtype=np.uint8)
jpeg = TurboJPEG()
assert jpeg.decode(jpeg.encode(image)).shape == image.shape
assert cv2.resize(image, (4, 4)).shape == (4, 4, 3)
assert np.asarray(open3d.geometry.PointCloud().points).shape == (0, 3)
assert (torch.ones(1, device="cpu") + 1).item() == 2
if sys.argv[3] == "cuda":
    assert torch.cuda.is_available()
    assert (torch.ones(1, device="cuda") + 1).item() == 2
if sys.argv[2] == "dev":
    dist = importlib.metadata.distribution("dimos")
    direct = json.loads(dist.read_text("direct_url.json"))
    assert direct["dir_info"]["editable"] is True
    assert direct["url"] == Path(sys.argv[1]).resolve().as_uri()
    for tool in ("pytest", "ruff", "mypy"):
        importlib.metadata.version(tool)
else:
    assert importlib.metadata.distribution("dimos").read_text("direct_url.json") is None or "UV_CONSTRAINT" in os.environ
print("Installation acceptance passed")
PY
VERIFY

before_python=$("$project/.venv/bin/python" -c 'import sys; print(sys.executable)')
/bin/bash "$repo/scripts/install.sh" "${args[@]}" 2>&1 | tee "$INSTALL_TEST_ROOT/logs/rerun.log"
test "$before_python" = "$("$project/.venv/bin/python" -c 'import sys; print(sys.executable)')"
if [[ "$mode" == dev ]]; then
    test "$(git -C "$project" rev-parse HEAD)" = "$expected_commit"
    git -C "$project" diff --exit-code
fi
