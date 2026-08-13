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
    # Generated LCM message headers, via a FetchContent source override.
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    # Standalone Boost.PFR, the same way.
    pfr = {
      url = "github:apolukhin/pfr_non_boost/2.3.2";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, pfr, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        isDarwin = nixpkgs.lib.hasSuffix "-darwin" system;
        pkgs = import nixpkgs {
          inherit system;
          config = { allowUnfree = true; cudaSupport = !isDarwin; };
        };
        lcm = lcm-extended.packages.${system}.lcm;

        # Every C++ build NVIDIA ships for this release. The ubuntu flavor does not
        # matter under autoPatchelf. `cuda` is the matching nixpkgs set: a runtime
        # newer than the SDK's fails inside cuSOLVER.
        sdks = {
          x86_64-cuda12 = {
            url = "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-cpp-17.0.0-x86_64-cuda12.6.3-ubuntu24.04.tar.gz";
            hash = "sha256-X2iCVMzKTlOuFcLyZJZU3vgQOdoWAU4LuXU0WpdyE9Q=";
            system = "x86_64-linux";
            cuda = "cudaPackages_12_6";
          };
          x86_64-cuda13 = {
            url = "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-cpp-17.0.0-x86_64-cuda13.2.0-ubuntu24.04.tar.gz";
            hash = "sha256-fEG94wknx6JBrDml3r0Kuy/yjS0HfxgkKUT26UuGbkg=";
            system = "x86_64-linux";
            cuda = "cudaPackages_13_2";
          };
          # Jetson Orin, sm_87, JetPack 6.
          orin = {
            url = "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-cpp-17.0.0-orin-cuda12.6.3-ubuntu22.04.tar.gz";
            hash = "sha256-V6e4zKsSZJG0rCqaPkHyw7wSPVCyeN/6Ma/tiY9GDw0=";
            system = "aarch64-linux";
            cuda = "cudaPackages_12_6";
          };
          # Jetson Thor, sm_110, JetPack 7. A different GPU generation from Orin, so
          # its build is not interchangeable with one.
          thor = {
            url = "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/cuvslam-cpp-17.0.0-thor-cuda13.0.1-ubuntu24.04.tar.gz";
            hash = "sha256-w5b476aY+oS8XVQn9EodgwXf8nrhnD9aioykLSoZTT8=";
            system = "aarch64-linux";
            cuda = "cudaPackages_13_0";
          };
          # Ours, since NVIDIA ships no macOS build: the same v17.0.0 sources compiled for
          # Apple silicon against CuMetal, targeting macOS 13 on apple-m1. It additionally
          # carries libcumetal.dylib and share/cumetal-cache.
          metal = {
            url = "https://github.com/jeff-hykin/cuda-metal/releases/download/cuvslam-v17.0.0-metal.1/cuvslam-cpp-17.0.0-arm64-metal-macos.tar.gz";
            hash = "sha256-rZI7hD05mGbCQWJThzlNjsmCMITNh/fekgTYBRltCRE=";
            system = "aarch64-darwin";
          };
        };

        forThisSystem = pkgs.lib.filterAttrs (_: sdk: sdk.system == system) sdks;
        # What a machine that says nothing about itself gets. CUDA 12 on both linux
        # arches: it is what the drivers in the field are, and a 13 driver runs a 12
        # build.
        defaultVariant = {
          aarch64-darwin = "metal";
          aarch64-linux = "orin";
        }.${system} or "x86_64-cuda12";

        cudaLibs = sdk: pkgs.lib.optionals (sdk ? cuda) (
          with pkgs.${sdk.cuda}; [ cuda_cudart libcublas libcusolver libcusparse ]
        );

        sdkFor = name: sdk: pkgs.stdenv.mkDerivation {
          pname = "cuvslam-sdk-${name}";
          version = "17.0.0";
          src = pkgs.fetchurl { inherit (sdk) url hash; };
          sourceRoot = ".";
          # ELF-only, and none of the CUDA runtime has a darwin build.
          nativeBuildInputs = pkgs.lib.optionals (!isDarwin) [ pkgs.autoPatchelfHook ];
          buildInputs = pkgs.lib.optionals (!isDarwin) [ pkgs.stdenv.cc.cc.lib ]
            ++ cudaLibs sdk;
          installPhase = ''
            runHook preInstall
            mkdir -p $out/lib $out/include $out/bin $out/share/cuvslam
            cp bin/libcuvslam.${if isDarwin then "dylib" else "so"} $out/lib/
            cp bin/cuvslam_api_launcher $out/bin/ || true
            cp -r include/cuvslam $out/include/
            # The NVIDIA Community License requires this to travel with the binary.
            cp LICENSE $out/share/cuvslam/
            echo "Licensed by NVIDIA Corporation under the NVIDIA Community License." \
              > $out/share/cuvslam/NOTICE
            ${pkgs.lib.optionalString isDarwin ''
              cp bin/libcumetal.dylib $out/lib/
              cp -r share/cumetal-cache $out/share/
              # The archive keeps every dylib beside the launcher; splitting them into
              # lib/ and bin/ moves the launcher one directory away from them.
              ${pkgs.darwin.cctools}/bin/install_name_tool \
                -add_rpath "@loader_path/../lib" $out/bin/cuvslam_api_launcher
            ''}
            runHook postInstall
          '';
          meta.license = pkgs.lib.licenses.unfree;  # NVIDIA Community License
        };
        moduleFor = name: sdk: let sdkPackage = sdkFor name sdk; in pkgs.stdenv.mkDerivation {
          pname = "dimos-cuvslam-native";
          version = "0.1.0";
          # cmake inputs only. Editing cuvslam.py must not rebuild the C++.
          src = pkgs.lib.fileset.toSource {
            root = ./.;
            fileset = pkgs.lib.fileset.unions [ ./CMakeLists.txt ./src ];
          };

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ]
            ++ pkgs.lib.optionals isDarwin [ pkgs.makeWrapper ]
            ++ pkgs.lib.optionals (!isDarwin) [ pkgs.autoPatchelfHook ];
          buildInputs = [ lcm pkgs.glib pkgs.nlohmann_json pkgs.eigen sdkPackage ]
            ++ cudaLibs sdk;

          cmakeFlags = [
            # cmake otherwise defaults to no optimisation at all.
            "-DCMAKE_BUILD_TYPE=Release"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFETCHCONTENT_SOURCE_DIR_PFR=${pfr}"
            # Outside this dir, which a git-tree flake reaches as a path literal.
            "-DDIMOS_NATIVE_CPP_DIR=${../../../native/cpp}"
            "-DCUVSLAM_SDK_DIR=${sdkPackage}"
          ];

          # The archive ships every kernel already compiled, but its store path is
          # read-only so CuMetal cannot use it as the normal cache; this is the read-only
          # lookup it consults first.
          postInstall = pkgs.lib.optionalString isDarwin ''
            wrapProgram $out/bin/cuvslam_odometry \
              --set-default CUMETAL_PREBUILT_CACHE_DIR ${sdkPackage}/share/cumetal-cache
          '';
        };
      in {
        # One package per build NVIDIA ships for this arch.
        packages = pkgs.lib.mapAttrs moduleFor forThisSystem
          // pkgs.lib.mapAttrs' (name: sdk: {
               name = "sdk-${name}";
               value = sdkFor name sdk;
             }) forThisSystem
          // { default = moduleFor defaultVariant forThisSystem.${defaultVariant}; };

        devShells.default = pkgs.mkShell {
          inputsFrom = [ self.packages.${system}.default ];
        };
      });
}
