{
  description = "SmartNav PGO native module (pose graph optimization with iSAM2 + PCL ICP)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    gtsam-extended = {
      url = "github:jeff-hykin/gtsam-extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, gtsam-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;

        # Use gtsam-extended's C++ library with unstable features (iSAM2)
        gtsam-base = gtsam-extended.packages.${system}.gtsam-cpp;
        gtsam = gtsam-base.overrideAttrs (old: {
          src = pkgs.fetchFromGitHub {
            owner = "borglab";
            repo = "gtsam";
            rev = "develop";
            sha256 = "sha256-IoXNMb6xwoxGgjWl/urzLPUvCMG3d8cOfxmvsE0p1bc=";
          };
          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";
          cmakeFlags = (builtins.filter (f: f != "-DGTSAM_BUILD_UNSTABLE=OFF") old.cmakeFlags) ++ [
            "-DGTSAM_BUILD_UNSTABLE=ON"
          ];
        });
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "smartnav-pgo-native";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            lcm
            pkgs.glib
            pkgs.eigen
            pkgs.boost
            pkgs.pcl
            gtsam
          ];

          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];
        };
      });
}
