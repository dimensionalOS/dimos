# Nix installation

Use the official installer with `--use-nix` to provision a Nix development shell and a virtual environment using the Nix-provided Python:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --use-nix --mode dev --project-dir ./dimos --capabilities navigation,manipulation
cd dimos
nix develop
source .venv/bin/activate
```

The installation summary includes Nix bootstrap when needed; after confirmation, the installer provisions Nix and enables flakes. An existing Nix installation must already have flakes enabled. Choose library mode to install the published package instead; the installer downloads the flake files into that project.

Nix is the default for NixOS. On Arch Linux and other Linux distributions whose package managers the installer does not handle, the interactive menu recommends Nix and also offers manually provisioned native dependencies; unattended runs default to Nix. This path is not covered by installation CI. Native Arch dependency installation through pacman is not implemented.

On Ubuntu, prefer the [system-package path](/docs/installation/ubuntu.md), which is tested in CI. Nix libraries can conflict with PyPI wheels on Ubuntu 22.04.

See [installer options](/docs/installation/index.md) and the [DDS guide](/docs/usage/transports/dds.md) for specialized setup.

## Manual installation

Install Nix and enable `nix-command` and `flakes` first. From a source checkout:

```sh skip
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/dimensionalOS/dimos.git
cd dimos
nix develop
export UV_PYTHON_PREFERENCE=only-system UV_PYTHON_DOWNLOADS=never
uv sync --locked --python "$(command -v python3)" --extra manipulation --extra unitree --extra cpu --group tests --group lint
source .venv/bin/activate
uv run --no-sync dimos --help
```

For a library environment, download the shell definition instead of cloning:

```sh skip
mkdir dimos-app && cd dimos-app
curl -fsSLO https://raw.githubusercontent.com/dimensionalOS/dimos/main/flake.nix
curl -fsSLO https://raw.githubusercontent.com/dimensionalOS/dimos/main/flake.lock
git init
git add flake.nix flake.lock
nix develop
export UV_PYTHON_PREFERENCE=only-system UV_PYTHON_DOWNLOADS=never
uv venv --python "$(command -v python3)"
source .venv/bin/activate
uv pip install --torch-backend cpu 'dimos[base,unitree,sim]'
uv run dimos --help
```

If uv is missing, install it with `curl -LsSf https://astral.sh/uv/install.sh | sh` and add `$HOME/.local/bin` to PATH. Always enter `nix develop` before activating these environments. Nix supplies Python and native libraries; uv manages Python packages.

In a developer checkout, `uv run --no-sync` uses the installed environment. When updating dependencies, repeat the selected `--extra cpu` or `--extra cuda` on `uv sync` so the accelerator choice is preserved.
