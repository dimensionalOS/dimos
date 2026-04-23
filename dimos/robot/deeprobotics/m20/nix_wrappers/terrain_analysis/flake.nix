{
  description = "Wrapper flake for dimensionalOS/dimos-module-terrain-analysis with M20 NOS kernel 5.10 unpackPhase workaround";

  # See sibling local_planner/flake.nix for full workaround rationale.

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    mod.url = "github:dimensionalOS/dimos-module-terrain-analysis/v0.1.1";
  };

  outputs = { self, nixpkgs, mod, ... }:
    let
      system = "aarch64-linux";
      pkgs = import nixpkgs { inherit system; };
      base = mod.packages.${system}.default;
    in {
      packages.${system}.default = base.overrideAttrs (old: {
        unpackPhase = ''
          /usr/bin/cp -r $src source
          /usr/bin/chmod -R u+w source
          cd source
        '';
        fixupPhase = ''
          /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
          /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
        '';
      });
    };
}
