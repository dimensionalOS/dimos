install nix,

https://nixos.wiki/wiki/Nix_Installation_Guide
```sh
sudo install -d -m755 -o $(id -u) -g $(id -g) /nix
curl -L https://nixos.org/nix/install | sh
```

install direnv
https://direnv.net/
```sh
apt-get install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
```

allow direnv in dimos will take a bit to pull the packages,
from that point on your env is standardized
```sh
cd dimos
direnv allow
```
