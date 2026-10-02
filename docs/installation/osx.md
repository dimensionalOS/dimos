# macOS installation

Use the official installer on macOS 14 or newer:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

macOS 14 is the supported minimum for the current installer. The default developer environment includes ONNX Runtime 1.24.1 and Drake 1.45.0, whose Apple Silicon wheels require macOS 14. Some older library combinations may work on earlier macOS releases, but this installer does not support or validate those combinations.

The installer sets up Homebrew dependencies, uv, Python 3.12, and dimOS. Choose **library** for the published package or **dev** for a source checkout. Follow the printed activation command when it finishes.

Apple Silicon is the target macOS configuration. macOS CI is paused because runner capacity is exhausted; the current installer needs local validation. Package and hardware support can differ from Linux.

See [installer options](/docs/installation/index.md).

## Transport note for macOS

LCM over UDP can be unreliable on macOS for large or high-rate replay workloads. dimOS defaults the global stream transport to **Zenoh** everywhere, so you never need `--transport=zenoh`. Use `--transport=lcm` if you need to force the legacy multicast path.

See the [Zenoh quickstart](/docs/usage/transports/index.md#zenoh-quickstart) for what the localhost-pinned default reaches and how to point it at a robot or the LAN.

## Manual installation

Use these steps if you need to install without the guided script.

```sh skip
# Install Homebrew first if it is not installed.
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
eval "$(/opt/homebrew/bin/brew shellenv)"
brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo pre-commit ffmpeg libsndfile pkg-config
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

Library mode selects CPU PyTorch wheels. On Linux x86_64 with a CUDA-capable GPU, use `--torch-backend cu128` for library mode or replace `--extra cpu` with `--extra cuda` to add GPU inference dependencies in developer mode.

Developer installs use the locked PyTorch build. On Linux x86_64 it includes CUDA libraries and also supports CPU execution without an NVIDIA GPU. Selecting `cpu` skips optional CUDA extras; it does not select a CPU-only PyTorch wheel. Use `uv run --no-sync` to use the installed environment, and repeat your selected extras when running `uv sync`.
