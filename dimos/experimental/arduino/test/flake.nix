{
  description = "Arduino LCM wire format compatibility test";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      # Pin to jeff/feat/arduino until that branch merges to main.
      url = "github:dimensionalOS/dimos-lcm/jeff/feat/arduino";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, lcm-extended }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        lcmFull = (lcm-extended.packages.${system}.default).overrideAttrs (old: {
          outputs = [ "out" ];
          postInstall = "";
        });
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "test_wire_compat";
          version = "0.1.0";
          # Source is the parent arduino/ directory so we can reach common/
          src = ./..;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ lcmFull pkgs.glib ];

          cmakeFlags = [
            "-DDIMOS_LCM_DIR=${dimos-lcm}"
          ];

          # CMakeLists.txt is in test/ subdirectory
          cmakeDir = "../test";

          installPhase = ''
            mkdir -p $out/bin
            cp test_wire_compat $out/bin/
          '';
        };

        devShells.default = pkgs.mkShell {
          inputsFrom = [ self.packages.${system}.default ];
        };
      });
}
