{
  description = "AV1 camera native module for dimos: V4L2 capture encoded with SVT-AV1, low delay";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  # packages.default builds for the machine you are on. packages.aarch64 (from x86_64-linux) cross-compiles for
  # the robot, which is how this gets onto an M20: build here, `nix copy --to ssh://<board>` the result.
  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "dimos-av1-camera";

        src = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        ours = [ name "dimos-module" "dimos-module-macros" ];
        # `target` is the pkgs set the binary runs on: pkgs itself, or a cross set for the robot.
        callWith = target: mode: import generated {
          pkgs = target;
          buildRustCrateForPkgs = cratePkgs: crate:
            (cratePkgs.buildRustCrate.override {
              defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
                # stabby-macros writes the builder's core count (NUM_JOBS) into its code; one value makes every machine's copy match.
                stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; };
                # Builds libjpeg-turbo from source (the `cmake` feature).
                turbojpeg-sys = attrs: {
                  nativeBuildInputs = (attrs.nativeBuildInputs or [ ]) ++ [ pkgs.cmake pkgs.nasm ];
                };
                ${name} = attrs: {
                  # build.rs reads this instead of asking pkg-config, so the headers and the library are the
                  # ones for the machine the binary will run on, not the builder's.
                  preConfigure = ''
                    export SVT_AV1_DIR=${target.svt-av1}
                  '';
                  buildInputs = (attrs.buildInputs or [ ]) ++ [ target.svt-av1 ];
                  # libSvtAv1Enc lives in the nix store, and the store path travels with the closure.
                  extraRustcOpts = (attrs.extraRustcOpts or [ ])
                    ++ [ "-C" "link-arg=-Wl,-rpath,${target.svt-av1}/lib" ];
                };
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
        packages = {
          default = buildOf (callWith pkgs null);
          lint = (buildOf (callWith pkgs "lint")).override { runTests = true; testCrateFlags = [ "--list" ]; };
          tests = (buildOf (callWith pkgs "test")).override { runTests = true; };
        } // pkgs.lib.optionalAttrs (system == "x86_64-linux") {
          # The tests are not run here: they would need an aarch64 emulator, and the point of this output is to
          # produce a binary for the robot from a laptop.
          aarch64 = buildOf (callWith pkgs.pkgsCross.aarch64-multiplatform null);
        };
        checks.lint = self.packages.${system}.lint;
        checks.tests = self.packages.${system}.tests;

        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt pkgs.cmake pkgs.nasm ];
          SVT_AV1_DIR = "${pkgs.svt-av1}";
        };
      });
}
