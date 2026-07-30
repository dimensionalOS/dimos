# Install DimOS with Nix

Install [Nix](https://nixos.org/download/) and enable
[flakes](https://wiki.nixos.org/wiki/Flakes). The official Nix installer is
recommended; the following commands use the Determinate Systems installer as a
quick start:

```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh

mkdir -p "$HOME/.config/nix"
echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"
```

## Use DimOS as a library

```bash
mkdir myproject
cd myproject

# Fetch the flake required to run nix develop outside the repository.
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/flake.nix
wget https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/flake.lock

# Enter the development shell that provides the system dependencies.
nix develop

python3 -m venv .venv
source .venv/bin/activate

# Install all optional capabilities. Select fewer extras when appropriate.
pip install "dimos[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu]"
```

## Develop DimOS

```bash
# Download large files only when Git needs them.
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos

nix develop

python3 -m venv .venv
source .venv/bin/activate

pip install -e ".[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu]"

mypy dimos
pytest --numprocesses=auto dimos
```
