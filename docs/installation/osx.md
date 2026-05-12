# macOS Install (12.6 or newer)

```sh skip
# install homebrew
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
# install dependencies
brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit

# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
```

# Using DimOS as a library

```sh skip
mkdir myproject && cd myproject

uv venv --python 3.12
source .venv/bin/activate

# install everything (depending on your use case you might not need all extras,
# check your respective platform guides)
uv pip install 'dimos[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu]'
```

# Developing on DimOS

```sh skip
# this allows getting large files on-demand (and not pulling all immediately)
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos

# `uv run` syncs the project deps + `tests` group on demand — enough for
# the default test suite and mypy. For self-hosted tests, sync the extras
# explicitly (see docs/development/testing.md).

# type check
uv run mypy dimos

# tests (around a minute to run)
uv run pytest --numprocesses=auto dimos
```
