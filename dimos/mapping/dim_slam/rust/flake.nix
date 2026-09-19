{
  description = "dimSLAM native module for DimOS: the dim_slam library behind an LCM wrapper";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
    cu-vslam-rs.url = "github:jeff-hykin/cu_vslam_rs";
    cu-vslam-rs.inputs.nixpkgs.follows = "nixpkgs";
    cu-vslam-rs.inputs.flake-utils.follows = "flake-utils";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix, cu-vslam-rs }:
    flake-utils.lib.eachSystem [ "aarch64-linux" "x86_64-linux" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "dim-slam-module";

        src = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };

        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        sdkPackages = nixpkgs.lib.filterAttrs
          (n: _: nixpkgs.lib.hasPrefix "sdk-" n)
          cu-vslam-rs.packages.${system};
        variants = map (nixpkgs.lib.removePrefix "sdk-") (builtins.attrNames sdkPackages);

        ours = [ name "dimos-module" "dimos-module-macros" ];
        callWith = variant: mode:
          let sdkPackage = sdkPackages."sdk-${variant}"; in
          import generated {
            inherit pkgs;
            buildRustCrateForPkgs = cratePkgs:
              let build = cratePkgs.buildRustCrate.override {
                    defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
                      # cu_vslam_rs's build.rs compiles its shim against this SDK.
                      cu_vslam_rs = _: { CUVSLAM_SDK_DIR = sdkPackage; };
                      dim-slam-module = _: { DEP_CUVSLAM_LIB_DIR = "${sdkPackage}/lib"; };
                    };
                  };
              in crate: build (crate // pkgs.lib.optionalAttrs
                (mode != null && builtins.elem crate.crateName ours)
                (pkgs.lib.optionalAttrs (mode == "lint") {
                  useClippy = true;
                  capLints = "forbid";
                  extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-D" "warnings" ];
                }));
          };
        buildOf = called:
          if called ? rootCrate then called.rootCrate.build
          else called.workspaceMembers.${name}.build;

        lintedVariant = builtins.head (builtins.sort builtins.lessThan variants);
      in {
        packages = nixpkgs.lib.genAttrs variants (v: buildOf (callWith v null)) // {
          default = buildOf (callWith lintedVariant null);
          lint = (buildOf (callWith lintedVariant "lint")).override {
            runTests = true;
            testCrateFlags = [ "--list" ];
          };
          tests = (buildOf (callWith lintedVariant "test")).override { runTests = true; };
        };
        checks.lint = self.packages.${system}.lint;
        checks.tests = self.packages.${system}.tests;

        devShells.default = pkgs.mkShellNoCC {
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ];
        };
      });
}
