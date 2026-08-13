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
    # dim-slam, our cuVSLAM lineage (odometry state serialization). Built from
    # source with ENFORCE_GPU=OFF so one library carries both the CUDA and CPU
    # paths and the module's use_gpu config is a pure runtime switch.
    cuvslam-fork = {
      url = "github:jeff-hykin/dim-slam/e9d0f7f3133b453b25efcfcc4167006790420a30";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, pfr, cuvslam-fork, ... }:
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

        # Variants built from the fork instead of NVIDIA's prebuilt tarball. Thor and
        # metal stay on tarballs until the fork build is proven on that hardware.
        # cuda12_8 on x86: sm_120 (Blackwell) needs nvcc >= 12.8.
        forkBuilds = {
          x86_64-cuda12 = {
            cuda = "cudaPackages_12_8";
            archs = "89;120";
            cudssPlatform = "linux-x86_64";
            cudssSha256 = "01s7xssfjadz1zfjprwp66j82h04snfpmjxg149m6a2bqq2nlw99";
          };
          orin = {
            cuda = "cudaPackages_12_6";
            archs = "87";
            # cuDSS >= 0.8 ships aarch64 as "linux-sbsa".
            cudssPlatform = "linux-sbsa";
            cudssSha256 = "12xixcrfl9yv2gf7rc0nkn2fhh171m2mnhvpfvgrfs4qbh0jd54l";
          };
        };

        # The fork's FetchContent dependencies, pre-fetched (the sandbox is offline) and
        # handed to cmake as FETCHCONTENT_SOURCE_DIR_* overrides. URL hashes are copied
        # verbatim from the fork's cmake/ext/*.cmake pins.
        depTarball = name: url: sha256: pkgs.runCommand "cuvslam-dep-${name}" { } ''
          mkdir -p $out
          tar xf ${pkgs.fetchurl { inherit url sha256; }} --strip-components=1 -C $out
        '';
        depGithub = name: repo: rev: sha256: pkgs.fetchzip {
          name = "cuvslam-dep-${name}";
          url = "https://github.com/${repo}/archive/${rev}.tar.gz";
          inherit sha256;
        };
        forkDepsFor = fork: {
          eigen = depTarball "eigen"
            "https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz"
            "b93c667d1b69265cdb4d9f30ec21f8facbbe8b307cf34c0b9942834c6d4fdbe2";
          # cuNLS pins /usr/local/cuda and its own arch list with plain set()s ahead of
          # project(), stomping the parent configuration; drop them so the nix toolchain
          # and our CMAKE_CUDA_ARCHITECTURES flow through.
          cunls = pkgs.runCommand "cuvslam-dep-cunls" { } ''
            mkdir -p $out
            tar xf ${pkgs.fetchurl {
              url = "https://github.com/nvidia-isaac/cuNLS/archive/refs/tags/Release_07_13_2026.tar.gz";
              sha256 = "23b2917ae3903e6a688edb1652e40202d314527cd7fa9db68c762f0429375f77";
            }} --strip-components=1 -C $out
            sed -i -e '/set(CMAKE_CUDA_COMPILER/d' -e '/set(CMAKE_CUDA_ARCHITECTURES/d' \
              $out/CMakeLists.txt
          '';
          lmdb = depTarball "lmdb"
            "https://github.com/LMDB/lmdb/archive/refs/tags/LMDB_0.9.31.tar.gz"
            "dd70a8c67807b3b8532b3e987b0a4e998962ecc28643e1af5ec77696b081c9b0";
          gflags = depTarball "gflags"
            "https://github.com/gflags/gflags/archive/v2.3.0.tar.gz"
            "f619a51371f41c0ad6837b2a98af9d4643b3371015d873887f7e8d3237320b2f";
          googletest = depTarball "googletest"
            "https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz"
            "65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c";
          jsoncpp = depTarball "jsoncpp"
            "https://github.com/open-source-parsers/jsoncpp/archive/1.9.6.tar.gz"
            "f93b6dd7ce796b13d02c108bc9f79812245a82e577581c4c9aabe57075c90ea2";
          libjpeg = depTarball "libjpeg"
            "https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/3.1.3/libjpeg-turbo-3.1.3.tar.gz"
            "075920b826834ac4ddf97661cc73491047855859affd671d52079c6867c1c6c0";
          libpng = depTarball "libpng"
            "https://github.com/pnggroup/libpng/archive/refs/tags/v1.6.55.tar.gz"
            "71a2c5b1218f60c4c6d2f1954c7eb20132156cae90bdb90b566c24db002782a6";
          spdlog = depTarball "spdlog"
            "https://github.com/gabime/spdlog/archive/v1.17.0.tar.gz"
            "d8862955c6d74e5846b3f580b1605d2428b11d97a410d86e2fb13e857cd3a744";
          "yaml-cpp" = depTarball "yaml-cpp"
            "https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.9.0.tar.gz"
            "25cb043240f828a8c51beb830569634bc7ac603978e0f69d6b63558dadefd49a";
          zlib = depTarball "zlib"
            "https://github.com/madler/zlib/archive/refs/tags/v1.3.1.tar.gz"
            "17e88863f3600672ab49182f217281b6fc4d3c762bde361935e436a95214d05c";
          cnpy = depGithub "cnpy" "rogersce/cnpy"
            "4e8810b1a8637695171ed346ce68f6984e585ef4"
            "1dgw86l47mwwbs11zqf8sas823qpjfgy0904hy0gmak8wfjw7hrl";
          circularbuffer = depGithub "circularbuffer" "vinitjames/circularbuffer"
            "cef66805cb5424e27300a966becc7c2678117c27"
            "0vi0y131x106v1fvpq3wr0dlyh3q4higkc7lqqrvvvmklvz1mnxn";
          dense_hash_map = depGithub "dense_hash_map" "Jiwan/dense_hash_map"
            "74277fc4813028ae4a9e8d9176788eb8001177a6"
            "0q4z9zvzas2pg566g889j4chy6w3m41bb82zrxs6ihl1arnral6q";
          # Downloaded by cuNLS's own cmake (AddCUDSS.cmake), also via FetchContent.
          cudss = depTarball "cudss"
            "https://developer.download.nvidia.com/compute/cudss/redist/libcudss/${fork.cudssPlatform}/libcudss-${fork.cudssPlatform}-0.8.0.10_cuda12-archive.tar.xz"
            fork.cudssSha256;
        };

        cudaLibs = sdk: pkgs.lib.optionals (sdk ? cuda) (
          with pkgs.${sdk.cuda}; [ cuda_cudart libcublas libcusolver libcusparse ]
            ++ pkgs.lib.optionals (pkgs.${sdk.cuda} ? libnvjitlink) [ libnvjitlink ]
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

        # Same output shape as sdkFor, but compiled from the fork's sources. Only the
        # library target is built: tools, examples, and tests stay out of the closure.
        forkSdkFor = name: fork: let
          cudaSet = pkgs.${fork.cuda};
          deps = forkDepsFor fork;
        in cudaSet.backendStdenv.mkDerivation {
          pname = "cuvslam-fork-${name}";
          version = "17.0.0-odom-state";
          src = cuvslam-fork;
          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config cudaSet.cuda_nvcc ];
          buildInputs = [ cudaSet.cuda_cudart cudaSet.libcublas cudaSet.libcusolver cudaSet.libcusparse ]
            ++ pkgs.lib.optionals (cudaSet ? libnvjitlink) [ cudaSet.libnvjitlink ]
            ++ pkgs.lib.optionals (cudaSet ? cuda_cccl) [ cudaSet.cuda_cccl ];
          cmakeFlags = [
            "-DCMAKE_BUILD_TYPE=Release"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DENFORCE_GPU=OFF"
            "-DCMAKE_CUDA_ARCHITECTURES=${fork.archs}"
            # The fork caches /usr/local/cuda paths; point both at the nix toolkit.
            "-DCUDAToolkit_ROOT=${cudaSet.cudatoolkit}"
            "-DCMAKE_CUDA_COMPILER=${cudaSet.cudatoolkit}/bin/nvcc"
          ];
          # Some dep builds write into their own source tree (zlib renames zconf.h), so
          # hand FetchContent writable copies rather than read-only store paths.
          preConfigure = pkgs.lib.concatStrings (pkgs.lib.mapAttrsToList
            (depName: depSource: ''
              mkdir -p dep-src
              cp -r --no-preserve=mode ${depSource} dep-src/${depName}
              cmakeFlagsArray+=("-DFETCHCONTENT_SOURCE_DIR_${pkgs.lib.toUpper depName}=$PWD/dep-src/${depName}")
            '')
            deps);
          buildFlags = [ "cuvslam" ];
          installPhase = ''
            runHook preInstall
            mkdir -p $out/lib $out/include/cuvslam $out/share/cuvslam
            cp bin/libcuvslam.so $out/lib/
            cp $src/libs/cuvslam/cuvslam2.h $out/include/cuvslam/
            cp $src/LICENSE $out/share/cuvslam/
            echo "Built from github.com/jeff-hykin/cuVSLAM (odom-state-serialization) with ENFORCE_GPU=OFF." \
              > $out/share/cuvslam/NOTICE
            runHook postInstall
          '';
          meta.license = pkgs.lib.licenses.unfree;  # NVIDIA Community License
        };

        forThisSystem = pkgs.lib.filterAttrs (_: sdk: sdk.system == system) sdks;
        # What a machine that says nothing about itself gets. CUDA 12 on both linux
        # arches: it is what the drivers in the field are, and a 13 driver runs a 12
        # build.
        defaultVariant = {
          aarch64-darwin = "metal";
          aarch64-linux = "orin";
        }.${system} or "x86_64-cuda12";

        # Fork-built variants override the tarball, and their newer CUDA set wins for
        # the module's runtime libs.
        sdkPackageFor = name: sdk:
          if forkBuilds ? ${name} then forkSdkFor name forkBuilds.${name} else sdkFor name sdk;
        runtimeCudaFor = name: sdk:
          if forkBuilds ? ${name} then { cuda = forkBuilds.${name}.cuda; } else sdk;

        moduleFor = name: sdk: let sdkPackage = sdkPackageFor name sdk; in pkgs.stdenv.mkDerivation {
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
            ++ cudaLibs (runtimeCudaFor name sdk);

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
               value = sdkPackageFor name sdk;
             }) forThisSystem
          // { default = moduleFor defaultVariant forThisSystem.${defaultVariant}; };

        devShells.default = pkgs.mkShell {
          inputsFrom = [ self.packages.${system}.default ];
        };
      });
}
