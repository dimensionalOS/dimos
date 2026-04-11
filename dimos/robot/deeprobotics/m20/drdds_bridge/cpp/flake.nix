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

          # Workaround: nix's coreutils (glibc 2.42) uses fchmodat2 which
          # kernel 5.10 doesn't have. Override phases that use cp/chmod
          # to use host binaries (linked against host glibc 2.31).
          unpackPhase = ''
            /usr/bin/cp -r $src source
            /usr/bin/chmod -R u+w source
            cd source
          '';
          fixupPhase = ''
            /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
            /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
          '';

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ pkgs.lcm pkgs.glib ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];
        };
      in {
        packages = {
          default = drdds_lidar_bridge;
          inherit drdds_lidar_bridge;
        };
      });
}
