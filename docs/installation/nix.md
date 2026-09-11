# Nix installation

Use the official installer with `--use-nix` to provision a Nix development shell and Python environment:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --use-nix --mode dev --project-dir ./dimos
cd dimos
nix develop
source .venv/bin/activate
```

The installer offers to install Nix if needed and enables flakes for a new installation. An existing Nix installation must already have flakes enabled. Choose library mode to install the published package instead; the installer downloads the flake files into that project.

Nix is an option for Arch Linux and other distributions whose package managers the installer does not handle. This path is not covered by installation CI. Native Arch dependency installation through pacman is not implemented.

On Ubuntu, prefer the [system-package path](/docs/installation/ubuntu.md), which is tested in CI. Nix libraries can conflict with PyPI wheels on Ubuntu 22.04.

See [installer options](/docs/installation/index.md) and the [DDS guide](/docs/usage/transports/dds.md) for specialized setup.
