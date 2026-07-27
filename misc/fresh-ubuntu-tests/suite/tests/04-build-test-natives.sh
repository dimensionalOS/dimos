#!/usr/bin/env bash
set -euxo pipefail

# Native test artifacts: install the toolchains bin/build-test-natives needs
# (cargo for the PyO3 Rust modules, nix for the cmu_nav binaries), build the
# artifacts, and run the tests that silently skip when they are missing.
# Neither toolchain is part of the documented install flow, so they are
# installed here, not in setup.sh.

# Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
export PATH="$HOME/.cargo/bin:$PATH"

# Nix (multi-user), with the Cachix substituter so the cmu_nav binaries are
# downloaded from CI's cache instead of compiled from source (see
# docs/usage/native_modules.md).
sh <(curl -L https://nixos.org/nix/install) --daemon --yes
sudo tee -a /etc/nix/nix.conf >/dev/null <<'EOF'
experimental-features = nix-command flakes
extra-substituters = https://dimensionalos.cachix.org
extra-trusted-public-keys = dimensionalos.cachix.org-1:20ynj6TjpoD3qTxkdNoeHtgs2G2pNvgAq1EQYLTHJXI=
EOF
sudo systemctl restart nix-daemon
export PATH="/nix/var/nix/profiles/default/bin:$PATH"

bin/build-test-natives

# Exactly the files that skip without the native artifacts; --error-for-skips
# proves they really ran. The -m override selects the self_hosted rosbag tests
# like bin/pytest-slow does; their LFS fixtures are pulled on demand.
uv run pytest --error-for-skips -m 'not (mujoco or self_hosted_large)' \
  dimos/mapping/ray_tracing \
  dimos/navigation/nav_3d/mls_planner \
  dimos/navigation/cmu_nav/modules/far_planner \
  dimos/navigation/cmu_nav/modules/local_planner \
  dimos/navigation/cmu_nav/modules/path_follower \
  dimos/navigation/cmu_nav/modules/pgo \
  dimos/navigation/cmu_nav/modules/terrain_analysis
