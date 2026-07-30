# Install DimOS on macOS

DimOS supports macOS 12.6 and newer.

```bash
# Install Homebrew.
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit

# Install uv.
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

## Use DimOS as a library

```bash
mkdir myproject
cd myproject

uv venv --python 3.12
source .venv/bin/activate

# Install all optional capabilities. Select fewer extras when appropriate.
uv pip install 'dimos[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu]'
```

## Develop DimOS

```bash
# Download large files only when Git needs them.
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos

# Install every dependency group so mypy and pytest are available.
uv sync --all-groups

uv run mypy dimos
uv run pytest --numprocesses=auto dimos
```

See [Testing](../development/testing.md) for self-hosted test requirements.

## Choose a transport

LCM over UDP can be unreliable on macOS for large or high-rate replay
workloads. DimOS therefore uses Zenoh as the default global stream transport
on macOS. Use `--transport=lcm` only when you need the legacy multicast path.

See the [Zenoh quick start](../usage/transports/index.md#zenoh-quickstart).

```bash
dimos --dtop --replay --replay-db=go2_bigoffice run unitree-go2
```

When you develop inside the repository, sync the full environment from the
checked-in lockfile:

```bash
uv sync --extra all --frozen
```
