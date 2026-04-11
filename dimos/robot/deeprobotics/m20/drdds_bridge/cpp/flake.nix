{
  description = "DrddsLidarBridge — M20 drdds SHM to LCM bridge";

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

        shm-transport-include = ../../../docker/drdds_bridge/include;
        native-common = ../../../../hardware/sensors/lidar/common;

        drdds_lidar_bridge = pkgs.stdenv.mkDerivation {
          pname = "drdds_lidar_bridge";
          version = "0.1.0";

          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ pkgs.lcm pkgs.glib ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DDRDDS_BRIDGE_INCLUDE=${shm-transport-include}"
            "-DNATIVE_COMMON_DIR=${native-common}"
          ];
        };
      in {
        packages = {
          default = drdds_lidar_bridge;
          inherit drdds_lidar_bridge;
        };
      });
}
