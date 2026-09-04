{
  description = "dimSLAM native module for DimOS: the dim_slam library behind an LCM wrapper";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    cu-vslam-rs.url = "github:jeff-hykin/cu_vslam_rs";
    cu-vslam-rs.inputs.nixpkgs.follows = "nixpkgs";
    cu-vslam-rs.inputs.flake-utils.follows = "flake-utils";
    # Relative git+file: will be deprecated (nix#12281) but there's no
    # viable alternative for reaching local path deps outside the flake dir currently
    # presumably an alternative will be added before this is removed.
    dimos-repo = { url = "git+file:../../../.."; flake = false; };
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nixpkgs, flake-utils, cu-vslam-rs, dimos-repo, crate2nix }:
    # Not eachDefaultSystem: nixpkgs 26.11 dropped x86_64-darwin, and merely naming
    # it is an eval error.
    flake-utils.lib.eachSystem [ "aarch64-darwin" "aarch64-linux" "x86_64-linux" ] (system:
      let
        isDarwin = nixpkgs.lib.hasSuffix "-darwin" system;
        pkgs = import nixpkgs {
          inherit system;
          config = { allowUnfree = true; cudaSupport = !isDarwin; };
        };

        # cuVSLAM SDKs come from the cu_vslam_rs flake: one sdk-<variant> package per build
        # that exists for this system. metal on aarch64-darwin; orin and thor on
        # aarch64-linux; x86_64-cuda12 and x86_64-cuda13 on x86_64-linux. Taken from that
        # flake rather than listed here so the two cannot drift apart.
        sdkPackages = nixpkgs.lib.filterAttrs (name: _: nixpkgs.lib.hasPrefix "sdk-" name)
          cu-vslam-rs.packages.${system};
        variants = map (nixpkgs.lib.removePrefix "sdk-") (builtins.attrNames sdkPackages);

        src = pkgs.runCommand "dim-slam-module-src" {} ''
          mkdir -p $out/dimos/mapping/dim_slam/rust
          cp -r ${./src} $out/dimos/mapping/dim_slam/rust/src
          cp ${./Cargo.toml} $out/dimos/mapping/dim_slam/rust/Cargo.toml
          cp ${./Cargo.lock} $out/dimos/mapping/dim_slam/rust/Cargo.lock
          cp ${./build.rs} $out/dimos/mapping/dim_slam/rust/build.rs

          mkdir -p $out/native/rust
          cp -r ${dimos-repo}/native/rust/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-repo}/native/rust/dimos-module-macros $out/native/rust/dimos-module-macros
        '';

        # One derivation per crate rather than one vendored blob, so a dependency bump
        # only rebuilds what changed and the SDK variants share everything below
        # cu_vslam_rs.
        # src, not the crate dir: the whole tree has to be visible or the crate's
        # ../../../../native path dependency escapes it.
        generatedCargoNix = crate2nix.tools.${system}.generatedCargoNix {
          name = "dim-slam-module";
          inherit src;
          cargoToml = "dimos/mapping/dim_slam/rust/Cargo.toml";
        };

        packageFor = variant: let sdkPackage = sdkPackages."sdk-${variant}"; in
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
      in {
        # No `default`: nix sees neither /proc/device-tree nor the installed driver, so orin
        # vs thor and cuda12 vs cuda13 are not decidable here, and guessing one builds a
        # module that dies at the first CUDA call. dim_slam.py's sdk_variant() detects the
        # hardware and names the variant on the build command.
        packages = nixpkgs.lib.genAttrs variants packageFor;
      });
}
