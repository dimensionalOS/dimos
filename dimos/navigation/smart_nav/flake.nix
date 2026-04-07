{
  description = "SmartNav native modules - autonomous navigation C++ components";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        commonBuildInputs = [
          pkgs.lcm
          pkgs.glib
          pkgs.eigen
          pkgs.boost
        ];

        commonNativeBuildInputs = [
          pkgs.cmake
          pkgs.pkg-config
        ];

        commonCmakeFlags = [
          "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
          "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
        ];

        # Full build with PCL
        smart_nav_native = pkgs.stdenv.mkDerivation {
          pname = "smart_nav-native";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = commonNativeBuildInputs;
          buildInputs = commonBuildInputs ++ [
            pkgs.pcl
            pkgs.opencv
            pkgs.ceres-solver
          ];

          cmakeFlags = commonCmakeFlags ++ [
            "-DUSE_PCL=ON"
          ];
        };

        # Lightweight build without PCL
        smart_nav_native_lite = pkgs.stdenv.mkDerivation {
          pname = "smart_nav-native-lite";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = commonNativeBuildInputs;
          buildInputs = commonBuildInputs ++ [
            pkgs.opencv
          ];

          cmakeFlags = commonCmakeFlags ++ [
            "-DUSE_PCL=OFF"
          ];
        };

        # Individual module builds
        mkModule = name: extra_inputs: extra_flags:
          pkgs.stdenv.mkDerivation {
            pname = "smart_nav-${name}";
            version = "0.1.0";
            src = ./.;

            nativeBuildInputs = commonNativeBuildInputs;
            buildInputs = commonBuildInputs ++ extra_inputs;

            cmakeFlags = commonCmakeFlags ++ extra_flags;

            # Only build the specific target
            buildPhase = ''
              cmake --build . --target ${name}
            '';

            installPhase = ''
              mkdir -p $out/bin
              cp ${name} $out/bin/
            '';
          };

        terrain_analysis = mkModule "terrain_analysis" [ pkgs.pcl ] [ "-DUSE_PCL=ON" ];
        local_planner = mkModule "local_planner" [ pkgs.pcl ] [ "-DUSE_PCL=ON" ];
        path_follower = mkModule "path_follower" [ pkgs.pcl ] [ "-DUSE_PCL=ON" ];
        far_planner = mkModule "far_planner" [ pkgs.pcl pkgs.opencv ] [ "-DUSE_PCL=ON" ];
        tare_planner = mkModule "tare_planner" [ pkgs.pcl ] [ "-DUSE_PCL=ON" ];
        pgo = mkModule "pgo" [ pkgs.pcl pkgs.gtsam ] [ "-DUSE_PCL=ON" ];
        arise_slam = mkModule "arise_slam" [ pkgs.pcl pkgs.ceres-solver ] [ "-DUSE_PCL=ON" ];
      in {
        packages = {
          default = smart_nav_native;
          inherit smart_nav_native smart_nav_native_lite;
          inherit terrain_analysis local_planner path_follower far_planner tare_planner pgo arise_slam;
        };

        devShells.default = pkgs.mkShell {
          buildInputs = commonBuildInputs ++ commonNativeBuildInputs ++ [
            pkgs.pcl
            pkgs.opencv
            pkgs.gtsam
            pkgs.ceres-solver
          ];
        };
      });
}
