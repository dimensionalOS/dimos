{
  description = "dimos C++ native module ping-pong example";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    zenoh.url = "github:jeff-hykin/zenoh_flake";
    zenoh.inputs.nixpkgs.follows = "nixpkgs";
    zenoh.inputs.flake-utils.follows = "flake-utils";
    nixpkgs.follows = "dimos-native-cpp/nixpkgs";
    flake-utils.follows = "dimos-native-cpp/flake-utils";
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
    dimos-native-cpp.url = "github:dimensionalOS/dimos?ref=jeff/fix/native_build_cargo_path&dir=native/cpp";
  };

  outputs = { self, nix-filter, nixpkgs, zenoh, flake-utils, lcm-extended, dimos-lcm, pfr, dimos-native-cpp, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;
        zenohc = zenoh.packages.${system}.zenoh-c;
        zenohcpp = zenoh.packages.${system}.zenoh-cpp;
      in {
        packages.dimos-native-module-examples-cpp = pkgs.stdenv.mkDerivation {
          pname = "dimos-native-ping-pong";
          version = "0.1.0";
          src = nix-filter.lib { root = ./.; exclude = [ "build" "target" "result" "__pycache__" ]; };

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ lcm pkgs.glib pkgs.nlohmann_json zenohc zenohcpp ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFETCHCONTENT_SOURCE_DIR_PFR=${pfr}"
            # The header-only SDK lives outside this dir. A git-tree flake can
            # reach it as a path literal within the repo tree.
            "-DDIMOS_NATIVE_CPP_DIR=${dimos-native-cpp.packages.${system}.default}"
          ];
        };
      });
}
