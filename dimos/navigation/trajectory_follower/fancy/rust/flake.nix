{
  description = "Trajectory follower native module for dimos";

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
        name = "dimos-trajectory-follower";

        src = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };

        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };
        # The crate's dev-dependency on itself (to turn features on for its tests) is a cycle to crate2nix.
        testSrc = pkgs.runCommand "${name}-test-src" { } ''
          cp -r ${src} $out && chmod -R u+w $out
          sed -i '/^${name} = { path = "\."/d' $out/Cargo.toml
          sed -i '/^ "${name}",$/d' $out/Cargo.lock
        '';
        generatedTest = crate2nix.tools.${system}.generatedCargoNix { inherit name; src = testSrc; };

        ours = [ name "dimos-module" "dimos-module-macros" ];
        callWith = mode: import (if mode == null then generated else generatedTest) {
          inherit pkgs;
          rootFeatures = if mode == null then [ "module" ] else [ "module" ];
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

        rustTools = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ];
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

        devShells.default = pkgs.mkShell { packages = rustTools; };
      });
}
