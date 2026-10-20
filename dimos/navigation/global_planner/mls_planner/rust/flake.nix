{
  description = "MLS planner native module for dimos";

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
        name = "dimos-mls-planner";

        moduleSource = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        sharedArchive = pkgs.fetchurl {
          url = "https://codeload.github.com/dimensionalOS/dimos/tar.gz/d5f801d7dd0208731bab566d079bae7abe6278f1";
          sha256 = "596db1c5658ac5e2f4b51b6d1ce8bcc4be9b590f718e3d2281faf61b4744c646";
        };
        sharedSource = pkgs.runCommand "dimos-mls-cdr-source-input" {
          nativeBuildInputs = [ pkgs.gnutar pkgs.gzip ];
        } ''
          mkdir -p $out
          tar -xzf ${sharedArchive} --strip-components=1 -C $out
        '';
        messageSource = (import "${sharedSource}/native/message-source.nix" { inherit pkgs; }).source;
        src = pkgs.runCommand "mls-cdr-source" { } ''
          cp -r ${moduleSource} $out
          chmod -R u+w $out
          mkdir -p $out/sdk
          cp -r ${sharedSource}/native/rust/dimos-module ${sharedSource}/native/rust/dimos-module-macros $out/sdk/
          cp -r ${messageSource}/rust $out/messages
          substituteInPlace $out/Cargo.toml \
            --replace-fail '../../../../../native/rust/dimos-module' 'sdk/dimos-module' \
            --replace-fail '../../../../../build/message-source/rust' 'messages' \
            --replace-fail '[workspace]' '[workspace]
          exclude = ["sdk/dimos-module", "sdk/dimos-module-macros", "messages"]'
        '';

        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        ours = [ name "dimos-module" "dimos-module-macros" ];
        callWith = mode: import generated {
          inherit pkgs;
          buildRustCrateForPkgs = cratePkgs: crate:
            # stabby-macros writes the builder's core count (NUM_JOBS) into its code; one value makes every machine's copy match.
            (cratePkgs.buildRustCrate.override {
              defaultCrateOverrides = cratePkgs.defaultCrateOverrides // { stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; }; };
            }) (crate // pkgs.lib.optionalAttrs
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
