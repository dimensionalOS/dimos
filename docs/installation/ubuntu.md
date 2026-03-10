# System Dependencies Install (Ubuntu 22.04 or 24.04)

```sh
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit

# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
```

# Using DimOS as a library

```sh
mkdir myproject && cd myproject

uv venv --python 3.12
source .venv/bin/activate

# install everything (depending on your use case you might not need all extras,
# check your respective platform guides)
uv pip install dimos[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu,dev]
```

# Developing on DimOS

```sh
# this allows getting large files on-demand (and not pulling all immediately)
export GIT_LFS_SKIP_SMUDGE=1
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

uv sync --all-extras --no-extra dds

# type check
uv run mypy dimos

# tests (around a minute to run)
uv run pytest dimos
```
