#!/usr/bin/env bash
set -euxo pipefail

# Native test artifacts: install the toolchain bin/build-test-natives needs
# (cargo for the PyO3 Rust modules), build the artifacts, and run the tests
# that silently skip when they are missing. The toolchain is not part of the
# documented install flow, so it is installed here, not in setup.sh.

# Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
export PATH="$HOME/.cargo/bin:$PATH"

bin/build-test-natives

# Exactly the files that skip without the native artifacts; --error-for-skips
# proves they really ran.
uv run pytest --error-for-skips -m 'not (mujoco or self_hosted_large)' \
  dimos/mapping/ray_tracing \
  dimos/navigation/nav_3d/mls_planner
