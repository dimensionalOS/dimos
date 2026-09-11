# Ubuntu installation

Use the official installer on Ubuntu 22.04/24.04, on x86_64 or ARM64:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

Choose system packages for the CI-tested path. The installer sets up apt dependencies, uv, Python 3.12, and dimOS, then verifies the CLI and native libraries.

Choose **library** for the published package or **dev** for a source checkout. For example:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --mode dev --no-nix --project-dir ./dimos
cd dimos
source .venv/bin/activate
```

An existing checkout is reused without pulling or switching branches. Developer mode includes test and lint dependencies; see [testing](/docs/development/testing.md) for additional groups.

Both modes pass CPU installation CI on Ubuntu 22.04/24.04 and x86_64/ARM64. Linux ARM64 excludes the unsupported `scene` extra. Jetson CUDA setup is not supported.

See [installer options](/docs/installation/index.md).

## Manual installation

Use these steps if you need to install without the guided script.

```sh skip
sudo apt-get update
sudo apt-get install -y ca-certificates curl git g++ portaudio19-dev git-lfs libturbojpeg pre-commit libgl1 libegl1 libglib2.0-0 ffmpeg libsndfile1 pkg-config
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

### Library environment

```sh skip
mkdir dimos-app && cd dimos-app
uv venv --python 3.12
source .venv/bin/activate
uv pip install --torch-backend cpu 'dimos[base,unitree,sim]'
uv run dimos --help
```

### Developer checkout

```sh skip
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/dimensionalOS/dimos.git
cd dimos
uv sync --locked --python 3.12 --extra manipulation --extra unitree --extra cpu --group tests --group lint
source .venv/bin/activate
uv run --no-sync dimos --help
```

These examples select CPU dependencies. On Linux x86_64 with a CUDA-capable GPU, use `--torch-backend cu128` for library mode or replace `--extra cpu` with `--extra cuda` for developer mode. Do not select both accelerator extras.

In a developer checkout, `uv run --no-sync` uses the installed environment. When updating dependencies, repeat the selected `--extra cpu` or `--extra cuda` on `uv sync` so the accelerator choice is preserved.
