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

        own = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        plannerArchive = pkgs.fetchurl {
          url = "https://codeload.github.com/dimensionalOS/dimos/tar.gz/62cfd6281dd985a49b7654af38c9d2d9939c1d65";
          sha256 = "92f116815a1dd0dfc6f63320af34c3a643d4dae1c488319d0f9e093e7f2089f3";
        };
        plannerSource = pkgs.runCommand "dimos-follower-planner-source" {
          nativeBuildInputs = [ pkgs.gnutar pkgs.gzip ];
        } ''
          mkdir -p $out
          tar -xzf ${plannerArchive} --strip-components=1 -C $out
        '';
        messageSource = (import "${plannerSource}/native/message-source.nix" { inherit pkgs; }).source;
        src = pkgs.runCommand "${name}-cdr-source" { } ''
          cp -r ${own} $out
          chmod -R u+w $out
          mkdir -p $out/sdk
          cp -r ${plannerSource}/dimos/navigation/local_planner/rust $out/local-planner
          cp -r ${plannerSource}/native/rust/dimos-module ${plannerSource}/native/rust/dimos-module-macros $out/sdk/
          cp -r ${messageSource}/rust $out/messages
          # The source artifacts own workspace roots; do not make them automatic members.
          substituteInPlace $out/Cargo.toml --replace-fail '[workspace]' '[workspace]
          exclude = ["local-planner", "sdk/dimos-module", "sdk/dimos-module-macros", "messages"]'
          substituteInPlace $out/Cargo.toml \
            --replace-fail '../../../local_planner/rust' 'local-planner' \
            --replace-fail '../../../../../native/rust/dimos-module' 'sdk/dimos-module' \
            --replace-fail '../../../../../build/message-source/rust' 'messages'
          substituteInPlace $out/local-planner/Cargo.toml \
            --replace-fail '../../../../native/rust/dimos-module' '../sdk/dimos-module' \
            --replace-fail '../../../../build/message-source/rust' '../messages'
        '';
        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        ours = [ name "dimos-local-planner" "dimos-module" "dimos-module-macros" ];
        callWith = mode: import generated {
          inherit pkgs;
          rootFeatures = [ "module" ];
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
