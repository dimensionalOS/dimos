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

        gtsam-base = gtsam-extended.packages.${system}.gtsam-cpp;
        gtsam = gtsam-base.overrideAttrs (_old: {
          src = pkgs.fetchFromGitHub {
            owner = "borglab";
            repo = "gtsam";
            rev = "1a9792a7ede244850a413739557635b606f295c0";
            sha256 = "sha256-zxm5TGVPW1vipFVpw01zcvKRw4mkh+5ZBCR1n6G466o=";
          };
          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";
        });
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "smartnav-pgo";
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

          # On macOS, libgtsam.4.dylib is referenced via @rpath but the binary
          # has no LC_RPATH entries, so it fails to load at runtime. Add one
          # pointing at the gtsam lib dir.
          postInstall = pkgs.lib.optionalString pkgs.stdenv.isDarwin ''
            ${pkgs.darwin.cctools}/bin/install_name_tool -add_rpath ${gtsam}/lib $out/bin/pgo
          '';
        };
      });
}
