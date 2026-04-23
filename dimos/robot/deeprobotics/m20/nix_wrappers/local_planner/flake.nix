{
  description = "Wrapper flake for dimensionalOS/dimos-module-local-planner with M20 NOS kernel 5.10 unpackPhase workaround";

  # Background: Nix's bundled coreutils (glibc 2.42) uses the fchmodat2
  # syscall which kernel 5.10 (M20 NOS) doesn't have. Default unpackPhase
  # and fixupPhase fail with "cp: setting permissions ... No such file or
  # directory". This wrapper overrides those phases to use host binaries
  # (/usr/bin/cp, /usr/bin/chmod) that link against the host's glibc 2.31.
  #
  # See: dimos/robot/deeprobotics/m20/nix_wrappers/README.md
  # See: plans/m20-rosnav-migration/06-simplify-remove-container/
  #      nix-arm64-kernel510-workaround.md

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    mod.url = "github:dimensionalOS/dimos-module-local-planner/v0.1.1";
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
