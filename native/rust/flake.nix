{
  description = "The shared dimos crates: consumed as a git dependency, built here only to lint and test them";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};

        src = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };

        crateOf = name:
          let
            generated = crate2nix.tools.${system}.generatedCargoNix {
              inherit name src;
              cargoToml = "${name}/Cargo.toml";
            };
            callWith = mode: import generated {
              inherit pkgs;
              buildRustCrateForPkgs = cratePkgs: crate:
                cratePkgs.buildRustCrate (crate // pkgs.lib.optionalAttrs
                  (mode != null && pkgs.lib.hasPrefix "dimos-module" crate.crateName)
                  ({
                    release = false;
                    extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-C" "debuginfo=0" ];
                  } // pkgs.lib.optionalAttrs (mode == "lint") {
                    useClippy = true;
                    capLints = "forbid";
                    extraRustcOpts =
                      (crate.extraRustcOpts or [ ]) ++ [ "-D" "warnings" "-C" "debuginfo=0" ];
                  }));
            };
            buildOf = called:
              if called ? rootCrate then called.rootCrate.build
              else called.workspaceMembers.${name}.build;
          in {
            lint = (buildOf (callWith "lint")).override {
              runTests = true;
              testCrateFlags = [ "--list" ];
            };
            tests = (buildOf (callWith "test")).override { runTests = true; };
          };

        module = crateOf "dimos-module";
        macros = crateOf "dimos-module-macros";
      in {
        packages.lint = pkgs.linkFarmFromDrvs "dimos-module-lint" [ module.lint macros.lint ];
        packages.tests = pkgs.linkFarmFromDrvs "dimos-module-tests" [ module.tests macros.tests ];
        checks.lint = self.packages.${system}.lint;
        checks.tests = self.packages.${system}.tests;

        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ];
        };
      });
}
