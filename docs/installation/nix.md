# Nix install (required for nix managed dimos)

You need to have [nix](https://nixos.org/) installed and [flakes](https://nixos.wiki/wiki/Flakes) enabled,

[official install docs](https://nixos.org/download/) recommended, but here is a quickstart:

```sh
# Install Nix https://nixos.org/download/
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh

# make sure nix-flakes are enabled
mkdir -p "$HOME/.config/nix"; echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"
```

# Install Direnv (optional)

[direnv](https://direnv.net) is a convenient helper that allows you to automatically enter your nix and python env once you cd into the project dir.

You can skip this step if you intend to type `nix develop` by hand.

Following [direnv install docs](https://direnv.net/docs/installation.html) is recommended (many distros have native package support)

but a quick oneliner binary install is
```sh
curl -sfL https://direnv.net/install.sh | bash
```

# Using DimOS as a library

```sh
mkdir myproject
cd myproject

# if on nixos you can pull our flake
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/flake.nix
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/flake.lock

# if using direnv (recommended)
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/.envrc.nix -O .envrc
direnv allow

# if using nix develop directly instead of direnv,
# nix develop

uv venv --python "3.12"
source .venv/bin/activate

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

# if using direnv (recommended)
cp .envrc.nix .envrc
direnv allow

# if using nix develop directly instead of direnv,
# nix develop

# create venv
uv venv --python 3.12

source .venv/bin/activate

uv sync --all-extras

# type check
uv run mypy dimos

# tests (around a minute to run)
uv run pytest dimos
```
