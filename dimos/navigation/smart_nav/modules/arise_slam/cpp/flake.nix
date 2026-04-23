{
  description = "arise_slam — LiDAR-inertial SLAM with ICP scan matching and factor graph optimization";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    gtsam-extended = {
      url = "github:jeff-hykin/gtsam-extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    sophus-src = {
      url = "github:strasdat/Sophus/1.22.10";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, gtsam-extended, sophus-src, lcm-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.default;

        # Use gtsam-extended's C++ library but:
        # 1. Fix stale source hash (develop branch moves)
        # 2. Enable unstable (needed for iSAM2)
        gtsam-base = gtsam-extended.packages.${system}.gtsam-cpp;
        gtsam = gtsam-base.overrideAttrs (old: {
          src = pkgs.fetchFromGitHub {
            owner = "borglab";
            repo = "gtsam";
            rev = "develop";
            sha256 = "sha256-IoXNMb6xwoxGgjWl/urzLPUvCMG3d8cOfxmvsE0p1bc=";
          };
          # GCC 15 + Eigen 3.4 SSE intrinsics trigger false-positive array-bounds
          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";
          cmakeFlags = (builtins.filter (f: f != "-DGTSAM_BUILD_UNSTABLE=OFF") old.cmakeFlags) ++ [
            "-DGTSAM_BUILD_UNSTABLE=ON"
          ];
        });

        sophus = pkgs.stdenv.mkDerivation {
          pname = "sophus";
          version = "1.22.10";
          src = sophus-src;

          nativeBuildInputs = with pkgs; [ cmake ];
          buildInputs = with pkgs; [ eigen ];

          # GCC 15 + Eigen 3.4 SSE intrinsics trigger false-positive array-bounds
          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DSOPHUS_INSTALL=ON"
            "-DBUILD_TESTS=OFF"
            "-DBUILD_EXAMPLES=OFF"
          ];
        };
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "arise-slam";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = with pkgs; [
            cmake
            pkg-config
          ];

          buildInputs = [
            lcm
            pkgs.eigen
            pkgs.pcl
            pkgs.boost
            pkgs.tbb
            pkgs.glib  # needed for LCM pkg-config resolution
            pkgs.llvmPackages.openmp
            pkgs.ceres-solver
            pkgs.glog
            pkgs.gflags
            gtsam
            sophus
          ];

          cmakeFlags = [
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];

          installPhase = ''
            mkdir -p $out/bin
            cp arise_slam $out/bin/
          '';
        };
      }
    );
}
