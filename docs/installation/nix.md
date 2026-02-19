# System Dependancies Install

## Nix install (all linux)
You need to have [nix](https://nixos.org/) installed and [flakes](https://nixos.wiki/wiki/Flakes) enabled,

[official install docs](https://nixos.org/download/) recomended, but here is a quickstart:

```sh
# Install Nix https://nixos.org/download/
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh

# make sure nix-flakes are enabled
mkdir -p "$HOME/.config/nix"; echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"
```

## Ubuntu 22.04 or 24.04
```sh
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit

# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
```

## macOS 12.6 or newer
```sh
# install homebrew
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
# install dependencies
brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit

# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
```

# DimOS as a library

```sh
mkdir myproject
cd myproject

uv venv --python "3.12"
source .venv/bin/activate

# if on nixos you can pull our flake
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/flake.nix
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/flake.lock

# this will just pull everything (big checkout)
# depending on what you are working on you might not need everything,
# check your respectative platform guides
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

source .env/bin/activate

uv sync --all-extras

# type check
uv run mypy dimos

# tests (around a minute to run)
uv run pytest dimos
```

# direnv

[Direnv](https://direnv.net/) is super convinient for auto-entering your nix env of uv venv
