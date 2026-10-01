{
  description = "micromamba for the dimos Habitat native module";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      # linux-64 only: the aihabitat conda channel has no aarch64 habitat-sim.
      systems = [ "x86_64-linux" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      # Provides the installer, not the simulator: habitat-sim is conda-only and
      # headless rendering needs the host's EGL driver, so this cannot be a derivation.
      devShells = forAll (pkgs: {
        default = pkgs.mkShellNoCC {
          packages = [ pkgs.micromamba pkgs.curl pkgs.cacert ];
        };
      });
    };
}
