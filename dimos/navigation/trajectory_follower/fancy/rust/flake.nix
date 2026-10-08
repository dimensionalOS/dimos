{
  description = "Trajectory follower native module for dimos";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
    # The sibling local planner crate it builds on (a path dependency in Cargo.toml), pinned like any shared code.
    dimos-local-planner.url = "github:dimensionalOS/dimos?dir=dimos/navigation/local_planner/rust";
    dimos-local-planner.inputs.nixpkgs.follows = "nixpkgs";
    dimos-local-planner.inputs.nix-filter.follows = "nix-filter";
    dimos-local-planner.inputs.flake-utils.follows = "flake-utils";
    dimos-local-planner.inputs.crate2nix.follows = "crate2nix";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix, dimos-local-planner }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "dimos-trajectory-follower";
        here = "navigation/trajectory_follower/fancy/rust";

        own = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        # The two crates at their repo-relative places, so the path dependency resolves.
        src = pkgs.runCommand "${name}-src" { } ''
          mkdir -p $out/${here} $out/navigation/local_planner
          cp -r ${own}/. $out/${here}
          cp -r ${dimos-local-planner} $out/navigation/local_planner/rust
        '';
        generated = crate2nix.tools.${system}.generatedCargoNix {
          inherit name src;
          cargoToml = "${here}/Cargo.toml";
        };

        ours = [ name "dimos-local-planner" "dimos-module" "dimos-module-macros" ];
        callWith = mode: import generated {
          inherit pkgs;
          rootFeatures = [ "module" ];
          buildRustCrateForPkgs = cratePkgs: crate:
            cratePkgs.buildRustCrate (crate // pkgs.lib.optionalAttrs
              (mode != null && builtins.elem crate.crateName ours)
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
        packages.default = buildOf (callWith null);
        packages.${name} = self.packages.${system}.default;

        packages.lint = (buildOf (callWith "lint")).override {
          runTests = true;
          testCrateFlags = [ "--list" ];
        };
        checks.lint = self.packages.${system}.lint;

        packages.tests = (buildOf (callWith "test")).override { runTests = true; };
        checks.tests = self.packages.${system}.tests;

        devShells.default = pkgs.mkShell { packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ]; };
      });
}
