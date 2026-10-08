{
  description = "Go2 DDS native module for dimos";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "dimos-go2-dds";

        src = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        # The vendored cyclonedds-sys binds nixpkgs' cyclonedds (bindgen) instead of cloning and building its own.
        cyclonedds = attrs: {
          nativeBuildInputs = (attrs.nativeBuildInputs or [ ]) ++ [ pkgs.rustPlatform.bindgenHook ];
          buildInputs = (attrs.buildInputs or [ ]) ++ [ pkgs.cyclonedds ];
          preConfigure = ''
            export CYCLONEDDS_LIB_DIR=${pkgs.cyclonedds}/lib
            export CYCLONEDDS_INCLUDE_DIR=${pkgs.cyclonedds}/include
          '';
        };

        ours = [ name ];
        callWith = mode: import generated {
          inherit pkgs;
          buildRustCrateForPkgs = cratePkgs: crate:
            (cratePkgs.buildRustCrate.override {
              defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
                # stabby-macros writes the builder's core count (NUM_JOBS) into its code; one value makes every machine's copy match.
                stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; };
                cyclonedds-sys = cyclonedds;
                ${name} = cyclonedds;
              };
            }) (crate // pkgs.lib.optionalAttrs (mode != null && builtins.elem crate.crateName ours) ({
              release = false;
              extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-C" "debuginfo=0" ];
            } // pkgs.lib.optionalAttrs (mode == "lint") {
              useClippy = true;
              capLints = "forbid";
              extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-D" "warnings" "-C" "debuginfo=0" ];
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

        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt pkgs.cyclonedds pkgs.rustPlatform.bindgenHook ];
          CYCLONEDDS_LIB_DIR = "${pkgs.cyclonedds}/lib";
          CYCLONEDDS_INCLUDE_DIR = "${pkgs.cyclonedds}/include";
        };
      });
}
