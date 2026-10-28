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

        sdkSource = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        sharedArchive = pkgs.fetchurl {
          url = "https://codeload.github.com/dimensionalOS/dimos/tar.gz/a9365487b4323151ea85041c6a944b509c0c3268";
          sha256 = "e339487dd2281980fcde7ab03c46dfd2e274491e0f73e66d9c6cbcaeea8902b2";
        };
        sharedSource = pkgs.runCommand "dimos-native-message-inputs" {
          nativeBuildInputs = [ pkgs.gnutar pkgs.gzip ];
        } ''
          mkdir -p $out
          tar -xzf ${sharedArchive} --strip-components=1 -C $out
        '';
        messageSource = (import "${sharedSource}/native/message-source.nix" { inherit pkgs; }).source;
        src = pkgs.runCommand "dimos-native-sdk-source" { } ''
          mkdir -p $out/native/rust $out/build/message-source
          cp -r ${sdkSource}/. $out/native/rust/
          cp ${sdkSource}/crate-hashes.json $out/crate-hashes.json
          cp -r ${messageSource}/rust $out/build/message-source/rust
        '';

        crateOf = name:
          let
            generated = crate2nix.tools.${system}.generatedCargoNix {
              inherit name src;
              cargoToml = "native/rust/${name}/Cargo.toml";
            };
            callWith = mode: import generated {
              inherit pkgs;
              buildRustCrateForPkgs = cratePkgs: crate:
                # stabby-macros writes the builder's core count (NUM_JOBS) into its code; one value makes every machine's copy match.
                (cratePkgs.buildRustCrate.override {
                  defaultCrateOverrides = cratePkgs.defaultCrateOverrides // { stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; }; };
                }) (crate // pkgs.lib.optionalAttrs
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
