{
  description = "micromamba for the dimos Habitat native module";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      # habitat-sim is published only as python 3.9 conda packages on the
      # aihabitat channel, so this shell provides the installer rather than the
      # simulator: install.sh builds the env underneath it. The GPU driver stays
      # impure -- headless rendering uses the host's libEGL_nvidia, which is why
      # this is mkShellNoCC and not a derivation.
      devShells = forAll (pkgs: {
        default = pkgs.mkShellNoCC {
          packages = [ pkgs.micromamba pkgs.curl pkgs.cacert ];
        };
      });
    };
}
