{
  description = "dimSLAM native module for DimOS: the dim_slam library behind an LCM wrapper";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    cu-vslam-rs.url = "github:jeff-hykin/cu_vslam_rs";
    cu-vslam-rs.inputs.nixpkgs.follows = "nixpkgs";
    cu-vslam-rs.inputs.flake-utils.follows = "flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nixpkgs, flake-utils, cu-vslam-rs, crate2nix }:
    # Not eachDefaultSystem: nixpkgs 26.11 dropped x86_64-darwin, and merely naming
    # it is an eval error.
    flake-utils.lib.eachSystem [ "aarch64-darwin" "aarch64-linux" "x86_64-linux" ] (system:
      let
        isDarwin = nixpkgs.lib.hasSuffix "-darwin" system;
        pkgs = import nixpkgs {
          inherit system;
          config = { allowUnfree = true; cudaSupport = !isDarwin; };
        };

        sdkPackages = nixpkgs.lib.mapAttrs' (name: sdk: { name = nixpkgs.lib.removePrefix "sdk-" name; value = sdk; })
          (nixpkgs.lib.filterAttrs (name: _: nixpkgs.lib.hasPrefix "sdk-" name) cu-vslam-rs.packages.${system});
        cuvslamVariant = cu-vslam-rs.packages.${system}.cuvslam-variant;

        src = pkgs.runCommand "dim-slam-module-src" {} ''
          mkdir -p $out/dimos/mapping/dim_slam/rust
          cp -r ${./src} $out/dimos/mapping/dim_slam/rust/src
          cp ${./Cargo.toml} $out/dimos/mapping/dim_slam/rust/Cargo.toml
          cp ${./Cargo.lock} $out/dimos/mapping/dim_slam/rust/Cargo.lock
          cp ${./build.rs} $out/dimos/mapping/dim_slam/rust/build.rs

          mkdir -p $out/native/rust
          cp -r ${../../../../native/rust/dimos-module} $out/native/rust/dimos-module
          cp -r ${../../../../native/rust/dimos-module-macros} $out/native/rust/dimos-module-macros
        '';

        generatedCargoNix = crate2nix.tools.${system}.generatedCargoNix {
          name = "dim-slam-module";
          inherit src;
          cargoToml = "dimos/mapping/dim_slam/rust/Cargo.toml";
        };

        packageFor = variant: let sdkPackage = sdkPackages.${variant}; in
          (import generatedCargoNix {
            inherit pkgs;
            buildRustCrateForPkgs = cratePkgs: cratePkgs.buildRustCrate.override {
              defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
                # cu_vslam_rs's build.rs compiles its shim against this SDK.
                cu_vslam_rs = _: { CUVSLAM_SDK_DIR = sdkPackage; };
                # buildRustCrate names DEP_ vars after the crate, cargo after the
                # `links` key, so cu_vslam_rs's lib_dir never reaches our build.rs
                # and the binary comes out with no rpath for libcuvslam.
                dim-slam-module = _: { DEP_CUVSLAM_LIB_DIR = "${sdkPackage}/lib"; };
              };
            };
          }).rootCrate.build;

        # JetPack 6 can't load CUDA 13, so one build per variant and a launcher picks.
        variantBuilds = pkgs.linkFarm "dim-slam-variants"
          (nixpkgs.lib.mapAttrsToList (name: _: { inherit name; path = packageFor name; }) cu-vslam-rs.bundledVariants.${system});
        launcher = pkgs.writeShellScriptBin "dim_slam" ''
          variant=$(${cuvslamVariant}/bin/cuvslam-variant) || exit 1
          if [ ! -x "${variantBuilds}/$variant/bin/dim_slam" ]; then
            echo "dim_slam: the default has no $variant build; build .#$variant" >&2
            exit 1
          fi
          echo "dim_slam: running the $variant build" >&2
          exec "${variantBuilds}/$variant/bin/dim_slam" "$@"
        '';
      in {
        packages = nixpkgs.lib.mapAttrs (name: _: packageFor name) sdkPackages // { default = launcher; };

        # script needs to detect cuda/non-cuda to pick the right things to load
        devShells.default = pkgs.mkShellNoCC {
          shellHook = ''
            if [ -z "''${CUVSLAM_SDK_DIR:-}" ]; then
              cuvslam_variant=$(${cuvslamVariant}/bin/cuvslam-variant)
              case "$cuvslam_variant" in
${nixpkgs.lib.concatStringsSep "\n" (nixpkgs.lib.mapAttrsToList (variant: sdk:
  "                ${variant}) cuvslam_sdk_drv=${builtins.unsafeDiscardStringContext sdk.drvPath} ;;"
) sdkPackages)}
                *) cuvslam_sdk_drv= ;;
              esac
              if [ -n "$cuvslam_sdk_drv" ] \
                && CUVSLAM_SDK_DIR=$(nix build --no-link --print-out-paths "$cuvslam_sdk_drv^out"); then
                export CUVSLAM_SDK_DIR
              else
                echo "no cuVSLAM SDK for variant '$cuvslam_variant'; building the stub" >&2
              fi
              unset cuvslam_variant cuvslam_sdk_drv
            fi
          '';
        };
      });
}
