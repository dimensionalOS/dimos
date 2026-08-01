{
  description = "dimos C++ native module ping-pong example";

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
    # Standalone Boost.PFR, consumed by the SDK via a FetchContent source override.
    pfr = {
      url = "github:apolukhin/pfr_non_boost/2.3.2";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, pfr, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "dimos-native-ping-pong";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ lcm pkgs.glib pkgs.nlohmann_json ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFETCHCONTENT_SOURCE_DIR_PFR=${pfr}"
            # The header-only SDK lives outside this dir. A git-tree flake can
            # reach it as a path literal within the repo tree.
            "-DDIMOS_NATIVE_CPP_DIR=${../../../native/cpp}"
          ];
        };
      });
}
