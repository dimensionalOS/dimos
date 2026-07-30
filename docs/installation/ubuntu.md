# Install system dependencies on Ubuntu

DimOS supports Ubuntu 22.04 and 24.04.

```bash
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit

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
