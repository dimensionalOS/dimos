#!/bin/bash
# Build a wrapped upstream module flake that applies the kernel 5.10
# unpackPhase/fixupPhase workaround. Usage:
#   ./wrap_build.sh <upstream-ref> <out-name>
# Example:
#   ./wrap_build.sh "github:dimensionalOS/dimos-module-local-planner/v0.1.1" local_planner
set -e
UPSTREAM="$1"
NAME="$2"
WORKDIR="/tmp/wrap-${NAME}"
rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

cat > flake.nix <<EOF
{
  description = "Wrapper with kernel 5.10 unpackPhase workaround";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    mod.url = "${UPSTREAM}";
  };
  outputs = { self, nixpkgs, mod, ... }: let
    system = "aarch64-linux";
    pkgs = import nixpkgs { inherit system; };
    base = mod.packages.\${system}.default;
  in {
    packages.\${system}.default = base.overrideAttrs (old: {
      unpackPhase = ''
        /usr/bin/cp -r \$src source
        /usr/bin/chmod -R u+w source
        cd source
      '';
      fixupPhase = ''
        /usr/bin/find \$out -type f -executable -exec /usr/bin/chmod 0755 {} \;
        /usr/bin/find \$out -type d -exec /usr/bin/chmod 0755 {} \;
      '';
    });
  };
}
EOF

echo "[wrap_build] flake.nix at $WORKDIR:"
cat flake.nix

/nix/store/l3gszbzvrwsj98yijsn87r5sc8kwd83b-nix-2.20.9/bin/nix \
  --extra-experimental-features "nix-command flakes" \
  --option sandbox false \
  build . \
  --no-write-lock-file \
  -o "/tmp/${NAME}-result" 2>&1 | tail -15

echo "[wrap_build] output:"
ls -la "/tmp/${NAME}-result/bin/" 2>&1 | head -5
