{
  description = "FAST-LIO2 + Livox Mid-360 native module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    # Using aphexcx fork with restored Velodyne PointCloud2 input path for
    # non-Livox lidars (RoboSense RSAIRY on M20). Goal: upstream back to
    # leshy/FAST-LIO-NON-ROS once validated.
    fast-lio = {
      url = "github:aphexcx/FAST-LIO-NON-ROS/dimos-integration-velodyne";
      flake = false;
    };
    # livox-sdk2 is inlined below rather than pulled in as a flake input.
    # nix >= 2.20 rejects `path:` flake inputs as mutable locks.  Defining
    # the livox-sdk2 derivation inline here keeps the flake self-contained
    # and avoids the need for the sibling `livox/cpp` flake during builds
    # of fastlio2_native (the Livox SDK is still needed at link time, and
    # is built from its upstream GitHub source).
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, fast-lio, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # Inlined livox-sdk2 derivation.  Kept in sync with
        # dimos/hardware/sensors/lidar/livox/cpp/flake.nix — change both when
        # updating the Livox SDK version.
        #
        # NOTE: uses `pkgs.fetchurl` + host-tar unpack rather than
        # `pkgs.fetchFromGitHub`, because the latter runs `chmod -R` inside
        # its own fixed-output source derivation using nix's coreutils
        # (glibc 2.42, needs fchmodat2) — kernel 5.10 on the M20 NOS doesn't
        # implement that syscall, so the unpack errors out before our
        # `unpackPhase` override ever runs.
        livox-sdk2 = pkgs.stdenv.mkDerivation rec {
          pname = "livox-sdk2";
          version = "1.2.5";

          src = pkgs.fetchurl {
            url = "https://github.com/Livox-SDK/Livox-SDK2/archive/refs/tags/v${version}.tar.gz";
            hash = "sha256-ONZclpeIKF8pgif72VTrJT4oFuSmqTqb/GJM+R+151o=";
          };

          nativeBuildInputs = [ pkgs.cmake ];

          # Custom unpack: use nix's tar (on stdenv PATH) without the usual
          # post-extract `chmod -R u+w`.  That chmod is what triggers the
          # fchmodat2 syscall missing on kernel 5.10 (Rockchip RK3588).
          # GitHub source tarballs come with writable file perms already.
          unpackPhase = ''
            tar xf $src
            cd Livox-SDK2-${version}
          '';
          dontFixup = true;

          cmakeFlags = [
            "-DBUILD_SHARED_LIBS=ON"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
          ];

          preConfigure = ''
            substituteInPlace CMakeLists.txt \
              --replace-fail "add_subdirectory(samples)" ""
            sed -i '1i #include <cstdint>' sdk_core/comm/define.h
            sed -i '1i #include <cstdint>' sdk_core/logger_handler/file_manager.h
          '';
        };

        fastlio2_native = pkgs.stdenv.mkDerivation {
          pname = "fastlio2_native";
          version = "0.1.0";

          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            livox-sdk2
            pkgs.lcm
            pkgs.glib
            pkgs.eigen
            pkgs.pcl
            pkgs.yaml-cpp
            pkgs.boost
            pkgs.llvmPackages.openmp
          ];

          # Don't copy source into the build dir — CMake runs out-of-source
          # and reads directly from $src. This sidesteps the recursive chmod
          # (both nix's default unpackPhase and our own `cp -r` versions hit
          # fchmodat2, which kernel 5.10 on NOS doesn't implement).
          dontUnpack = true;
          dontFixup = true;
          configurePhase = ''
            runHook preConfigure
            mkdir -p build
            cd build
            cmake $src \
              -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_INSTALL_PREFIX=$out \
              -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
              -DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm} \
              -DFASTLIO_DIR=${fast-lio}
            runHook postConfigure
          '';

          # cmakeFlags applied inline in configurePhase above.
          # LIVOX_COMMON_DIR: CMakeLists.txt falls back to livox_common_vendor/
          # (vendored copy of ../../common/) when LIVOX_COMMON_DIR is unset.
        };
      in {
        packages = {
          default = fastlio2_native;
          inherit fastlio2_native;
        };
      });
}
