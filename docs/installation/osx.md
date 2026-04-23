# macOS Install (12.6 or newer)

```sh
# install homebrew
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
# install dependencies
brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit

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
uv pip install 'dimos[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu,dev]'
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

## Transport note for macOS

LCM over UDP can be unreliable on macOS for large or high-rate replay workloads. If you are running heavy replay traffic, prefer Zenoh:

```sh
dimos --transport=zenoh --dtop --replay --replay-dir=unitree_go2_bigoffice run unitree-go2
```

If you are developing on the repository, prefer syncing the full environment with the checked-in lockfile:

```sh
uv sync --all-extras --no-extra dds --no-extra cuda --frozen
```

Do not rely on `uv sync --extra zenoh` in an existing full development environment. That can re-resolve the environment in a way that removes unrelated packages you already had installed.
