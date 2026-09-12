#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
# Run one real installation on a disposable host/container:
#   INSTALL_TEST_ROOT="$(mktemp -d)" bash scripts/test-install.sh library|dev navigation|manipulation|navigation,manipulation
set -euo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
mode=${1:?expected library or dev}
case "$mode" in library|dev) ;; *) exit 2;; esac
capabilities=${2:?expected navigation, manipulation, or navigation,manipulation}
: "${INSTALL_TEST_ROOT:?set INSTALL_TEST_ROOT to an empty temporary directory}"
mkdir -p "$INSTALL_TEST_ROOT/logs"
project="$INSTALL_TEST_ROOT/$mode"
export GIT_LFS_SKIP_SMUDGE=1
export UV_PYTHON_PREFERENCE=only-managed
export CUDA_VISIBLE_DEVICES=""
unset VIRTUAL_ENV PYTHONPATH

if [[ "$mode" == dev ]]; then
    expected_commit=$(git -C "$repo" rev-parse HEAD)
    git clone --no-hardlinks --no-checkout "$repo" "$project"
    git -C "$project" checkout --detach "$expected_commit"
fi

# The installer performs bounded CLI and capability dependency verification.
# No blueprint is started. pipefail preserves installation errors.
cat "$repo/scripts/install.sh" | /bin/bash -s -- \
    --non-interactive --no-nix --no-cuda --capabilities "$capabilities" \
    --mode "$mode" --project-dir "$project" \
    2>&1 | tee "$INSTALL_TEST_ROOT/logs/install.log"
