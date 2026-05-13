{
  description = "SmartNav RtabMap native module (RTAB-Map SLAM with OctoMap + raycasting)";

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
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "smartnav-rtab-map";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            lcm
            pkgs.glib
            pkgs.eigen
            pkgs.boost
            pkgs.pcl
            pkgs.opencv
            pkgs.rtabmap
            pkgs.octomap  # transitive: rtabmap's global_map/OctoMap.h includes octomap/ColorOcTree.h
          ];

          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];

          # On macOS, librtabmap_core.dylib is referenced via @rpath but the
          # binary has no LC_RPATH entries; add one pointing at the rtabmap
          # lib dir so the loader can find it at runtime. Linux ELF rpath is
          # already handled by nixpkgs' generic rpath wrapper.
          postInstall = pkgs.lib.optionalString pkgs.stdenv.isDarwin ''
            ${pkgs.darwin.cctools}/bin/install_name_tool \
              -add_rpath ${pkgs.rtabmap}/lib $out/bin/rtab_map
          '';
        };
      });
}
