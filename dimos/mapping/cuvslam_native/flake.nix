{
  description = "dimos cuVSLAM native C++ module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    # Generated LCM message headers, consumed via a FetchContent source override.
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    # Standalone Boost.PFR, consumed by the dimos native SDK the same way.
    pfr = {
      url = "github:apolukhin/pfr_non_boost/2.3.2";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, pfr, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        # cudaSupport pulls the unfree CUDA runtime that libcuvslam.so links.
        pkgs = import nixpkgs {
          inherit system;
          config = { allowUnfree = true; cudaSupport = true; };
        };
        lcm = lcm-extended.packages.${system}.lcm;
        cuda = pkgs.cudaPackages;

        # NVIDIA's official prebuilt C++ SDK. Pinned by hash rather than vendored
        # into the repo: the NVIDIA Community License permits redistribution, but
        # a 111 MB binary does not belong in git.
        #
        # Version note: both are the cuda12.6 build, chosen to match the pycuvslam
        # 17.0.0 wheel already in use (same version, same bundled libcudart
        # 12.6.77) so C++ and Python results are comparable.
        #
        # NVIDIA ships a separate tarball per architecture. The aarch64 one is the
        # "orin" build and is Ubuntu 22.04 based, matching JetPack 6 (L4T R36) --
        # there is no aarch64 24.04 build, so this is not an oversight.
        sdkSource = {
          x86_64-linux = {
            url = "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-cpp-17.0.0-x86_64-cuda12.6.3-ubuntu24.04.tar.gz";
            hash = "sha256-X2iCVMzKTlOuFcLyZJZU3vgQOdoWAU4LuXU0WpdyE9Q=";
          };
          aarch64-linux = {
            url = "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-cpp-17.0.0-orin-cuda12.6.3-ubuntu22.04.tar.gz";
            hash = "sha256-V6e4zKsSZJG0rCqaPkHyw7wSPVCyeN/6Ma/tiY9GDw0=";
          };
        }.${system} or (throw "cuVSLAM ships no SDK for ${system}");

        cuvslam-sdk = pkgs.stdenv.mkDerivation {
          pname = "cuvslam-sdk";
          version = "17.0.0";
          src = pkgs.fetchurl { inherit (sdkSource) url hash; };
          sourceRoot = ".";
          nativeBuildInputs = [ pkgs.autoPatchelfHook ];
          buildInputs = [
            pkgs.stdenv.cc.cc.lib
            cuda.cuda_cudart
            cuda.libcublas
            cuda.libcusolver
            cuda.libcusparse
          ];
          installPhase = ''
            runHook preInstall
            mkdir -p $out/lib $out/include $out/bin $out/share/cuvslam
            cp bin/libcuvslam.so $out/lib/
            cp bin/cuvslam_api_launcher $out/bin/ || true
            cp -r include/cuvslam $out/include/
            # NVIDIA Community License requires the attribution notice to travel
            # with any redistribution.
            cp LICENSE $out/share/cuvslam/
            echo "Licensed by NVIDIA Corporation under the NVIDIA Community License." \
              > $out/share/cuvslam/NOTICE
            runHook postInstall
          '';
          meta.license = pkgs.lib.licenses.unfree;  # NVIDIA Community License
        };
      in {
        packages.cuvslam-sdk = cuvslam-sdk;

        packages.default = pkgs.stdenv.mkDerivation {
          pname = "dimos-cuvslam-native";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config pkgs.autoPatchelfHook ];
          buildInputs = [
            lcm
            pkgs.glib
            pkgs.nlohmann_json
            cuvslam-sdk
            cuda.cuda_cudart
            cuda.libcublas
            cuda.libcusolver
            cuda.libcusparse
          ];

          cmakeFlags = [
            # Without this cmake defaults to no optimisation at all, which showed
            # up as the per-frame speckle mask taking 20x longer than the tracker.
            "-DCMAKE_BUILD_TYPE=Release"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFETCHCONTENT_SOURCE_DIR_PFR=${pfr}"
            # Header-only dimos native SDK lives outside this dir; a git-tree
            # flake can reach it as a path literal within the repo tree.
            "-DDIMOS_NATIVE_CPP_DIR=${../../../native/cpp}"
            "-DCUVSLAM_SDK_DIR=${cuvslam-sdk}"
          ];
        };

        devShells.default = pkgs.mkShell {
          inputsFrom = [ self.packages.${system}.default ];
        };
      });
}
