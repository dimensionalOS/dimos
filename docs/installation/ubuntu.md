# System Dependencies Install (Ubuntu 22.04 or 24.04)

```sh
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit

# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
```

# Install Direnv (optional)

[direnv](https://direnv.net) is a convenient helper that allows you to automatically enter your python env once you cd into the project dir.

You can skip this step if you intend to activate manually.

Following [direnv install docs](https://direnv.net/docs/installation.html) is recommended (many distros have native package support)

but a quick oneliner binary install is
```sh
curl -sfL https://direnv.net/install.sh | bash
```

# Using DimOS as a library

```sh
mkdir myproject
cd myproject

uv venv --python "3.12"

# if using direnv (recommended)
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/.envrc.venv -O .envrc
direnv allow
# if not using direnv
# source .venv/bin/activate

# this will just pull everything (big checkout)
# depending on what you are working on you might not need everything,
# check your respective platform guides
uv pip install dimos[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu,dev]
```

# Developing on DimOS

```sh
# this allows getting large files on-demand (and not pulling all immediately)
export GIT_LFS_SKIP_SMUDGE=1
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

# create venv
uv venv --python 3.12

# if using direnv (recommended)
direnv allow

# if not using direnv
# source .venv/bin/activate

uv sync --all-extras

# type check
uv run mypy dimos

# tests (around a minute to run)
uv run pytest dimos
```
