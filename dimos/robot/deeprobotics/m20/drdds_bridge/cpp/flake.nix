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

        drdds_lidar_bridge = pkgs.stdenv.mkDerivation {
          pname = "drdds_lidar_bridge";
          version = "0.1.0";

          src = ./.;

          # Kernel 5.10 on NOS lacks fchmodat2; nix's default unpackPhase
          # and `cp -r` both trip that syscall via glibc 2.42 coreutils,
          # and /usr/bin/cp isn't visible inside the nix build sandbox on
          # this host. Match the FAST-LIO2 flake's workaround: skip the
          # copy-into-build-dir entirely and have cmake read $src directly.
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
              -DBUILD_NAV_CMD_PUB=OFF
            runHook postConfigure
          '';

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ pkgs.lcm pkgs.glib ];
        };
      in {
        packages = {
          default = drdds_lidar_bridge;
          inherit drdds_lidar_bridge;
        };
      });
}
