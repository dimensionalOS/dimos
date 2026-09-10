{
  description = "Point-LIO + Livox Mid-360 native module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    livox-sdk.url = "path:../../livox/cpp";
    livox-sdk.inputs.nixpkgs.follows = "nixpkgs";
    livox-sdk.inputs.flake-utils.follows = "flake-utils";
    livox-sdk.inputs.lcm-extended.follows = "lcm-extended";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    # Standalone Boost.PFR, consumed by the SDK via a FetchContent source override.
    pfr = {
      url = "github:apolukhin/pfr_non_boost/2.3.2";
      flake = false;
    };
    fast-lio = {
      # Point-LIO fork, split out of dimos-module-fastlio2's pointlio branch.
      url = "github:dimensionalOS/dimos-module-pointlio?ref=main";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nixpkgs, flake-utils, livox-sdk, dimos-lcm, pfr, fast-lio, lcm-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        # Point-LIO uses common/filters and includes PCL I/O headers.
        # Avoid VTK's large, uncached Darwin dependency tree entirely.
        pcl = pkgs.pcl.overrideAttrs (old: {
          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ pkgs.eigen pkgs.boost pkgs.flann pkgs.qhull pkgs.zlib pkgs.cjson ]
            ++ pkgs.lib.optionals pkgs.stdenv.cc.isClang [ pkgs.llvmPackages.openmp ];
          propagatedBuildInputs = [ pkgs.boost pkgs.flann ];
          cmakeFlags = (old.cmakeFlags or [ ]) ++ [
            "-DWITH_VTK=OFF"
            "-DWITH_QT=OFF"
            "-DWITH_OPENGL=OFF"
            "-DBUILD_features=OFF"
            "-DBUILD_ml=OFF"
            "-DBUILD_segmentation=OFF"
            "-DBUILD_surface=OFF"
            "-DBUILD_registration=OFF"
            "-DBUILD_keypoints=OFF"
            "-DBUILD_tracking=OFF"
            "-DBUILD_visualization=OFF"
            "-DBUILD_tools=OFF"
            "-DBUILD_apps=OFF"
            "-DBUILD_examples=OFF"
            "-DBUILD_global_tests=OFF"
          ];
        });
        livox-sdk2 = livox-sdk.packages.${system}.livox-sdk2;
        lcm = lcm-extended.packages.${system}.lcm;

        livox-common = ../../common;

        # Patch the Point-LIO fork in place: resize (not reserve) the per-point
        # vectors in run_once, whose reserve+operator[] is out-of-bounds UB that
        # macOS libc++'s hardened operator[] turns into a SIGTRAP. applyPatches
        # gives a writable, patched copy of the read-only flake input.
        fast-lio-patched = pkgs.applyPatches {
          name = "fast-lio-pointlio-patched";
          src = fast-lio;
          patches = [ ./fastlio-resize-darwin.patch ];
        };

        pointlio_native = pkgs.stdenv.mkDerivation {
          pname = "pointlio_native";
          version = "0.2.0";

          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            livox-sdk2
            lcm
            pkgs.glib
            pkgs.eigen
            pcl
            pkgs.glog
            pkgs.boost
            pkgs.llvmPackages.openmp
            pkgs.nlohmann_json
          ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFETCHCONTENT_SOURCE_DIR_PFR=${pfr}"
            "-DFASTLIO_DIR=${fast-lio-patched}"
            "-DLIVOX_COMMON_DIR=${livox-common}"
            # The header-only SDK lives outside this dir. A git-tree flake can
            # reach it as a path literal within the repo tree.
            "-DDIMOS_NATIVE_CPP_DIR=${../../../../../../native/cpp}"
          ];
        };
      in {
        packages = {
          default = pointlio_native;
          inherit pointlio_native;
        };
      });
}
