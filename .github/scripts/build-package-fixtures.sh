#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

# Runs on one existing Linux/Python 3.12 test leg. Other legs keep their fast suite.
workspace=$(git rev-parse --show-toplevel)
prefix="$RUNNER_TEMP/package-native"
wheels="$RUNNER_TEMP/package-wheels"
mkdir -p "$prefix" "$wheels"

gh release download 1.10.1 --repo eclipse-zenoh/zenoh-c \
  --pattern zenoh-c-1.10.1-x86_64-unknown-linux-gnu-standalone.zip --dir "$prefix"
gh release download 1.10.1 --repo eclipse-zenoh/zenoh-cpp \
  --pattern zenohcpp-1.10.1-standalone.zip --dir "$prefix"
(cd "$prefix" && sha256sum -c <<'SHA256'
9ee0f2d732b0f3042a7e1cd3076042a2bc3ac0415587c40bc3ed7b8b62fbde11  zenoh-c-1.10.1-x86_64-unknown-linux-gnu-standalone.zip
83f50d1b26d708da70da54862708a131cb903b4ce99ba5e12bbe88a383a8ea49  zenohcpp-1.10.1-standalone.zip
SHA256
)
unzip -q "$prefix/zenoh-c-1.10.1-x86_64-unknown-linux-gnu-standalone.zip" -d "$prefix"
unzip -q "$prefix/zenohcpp-1.10.1-standalone.zip" -d "$prefix"
# The relocatable upstream archive retains /usr/local in its pkg-config file.
sed -i "s|^prefix=.*|prefix=$prefix|" "$prefix/lib/pkgconfig/zenohc.pc"
export CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CARGO_TARGET_DIR="$workspace/target"
# SDK sources do not consume the monorepo's recordings or other LFS assets.
export GIT_LFS_SKIP_SMUDGE=1
# Tests may run git-lfs install and restore filter-process. Keep the existing
# CI download guard on its supported smudge path, with its size cap unchanged.
git config --global --unset-all filter.lfs.process || true

uv pip install --python .venv/bin/python 'build>=1,<2' 'scikit-build-core>=0.11,<2' \
  'setuptools>=70' wheel 'pybind11>=2.12'
export DIMOS_ALLOW_MISSING_COCKPIT=1
uv run python -m build --wheel --no-isolation --outdir "$wheels"
for project in rust cpp python; do
  # Python support joins this stacked change after the native recipes.
  if [ -d "examples/packages/$project" ]; then
    uv run python -m build --wheel --no-isolation --outdir "$wheels" "examples/packages/$project"
  fi
done

# Prime standard uv artifacts once, then the acceptance test installs completely
# offline into its own fresh host and child environments. Never alter root pins.
uv venv "$RUNNER_TEMP/package-prime" --python .venv/bin/python
uv pip install --python "$RUNNER_TEMP/package-prime/bin/python" "$wheels"/*.whl 'packaging>=26'
uv pip install --python .venv/bin/python --target "$RUNNER_TEMP/package-child-dependency" 'packaging==25.0'
{
  echo "DIMOS_PACKAGE_WHEELHOUSE=$wheels"
  echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
} >> "$GITHUB_ENV"
