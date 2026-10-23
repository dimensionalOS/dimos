{
  description = "The shared dimos C++ native module SDK, as a source tree for cmake";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # Only the SDK's own test suite needs these; each module brings its own.
    zenoh.url = "github:jeff-hykin/zenoh_flake";
    zenoh.inputs.nixpkgs.follows = "nixpkgs";
    zenoh.inputs.flake-utils.follows = "flake-utils";
    lcm-extended.url = "github:jeff-hykin/lcm_extended";
    lcm-extended.inputs.nixpkgs.follows = "nixpkgs";
    lcm-extended.inputs.flake-utils.follows = "flake-utils";
    pfr = { url = "github:apolukhin/pfr_non_boost/2.3.2"; flake = false; };
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, zenoh, lcm-extended, pfr }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        src = nix-filter.lib { root = ./.; exclude = [ "build" "target" "result" "__pycache__" ]; };
      in {
        packages.default = pkgs.runCommand "dimos-native-cpp" { } ''
          cp -r ${src} $out
          chmod -R u+w $out
        '';

        # Header-only: nothing to lint yet; declared so the gate sees the flake.
        checks.lint = pkgs.runCommand "dimos-native-cpp-lint" { } "mkdir $out";
        checks.tests = pkgs.stdenv.mkDerivation {
          pname = "dimos-native-cpp-tests";
          version = "0.1.0";
          inherit src;
          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            lcm-extended.packages.${system}.lcm
            pkgs.glib
            pkgs.nlohmann_json
            zenoh.packages.${system}.zenoh-c
            zenoh.packages.${system}.zenoh-cpp
          ];
          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DDIMOS_NATIVE_BUILD_TESTS=ON"
            "-DFETCHCONTENT_SOURCE_DIR_PFR=${pfr}"
            "-DFETCHCONTENT_SOURCE_DIR_DOCTEST=${pkgs.doctest.src}"
          ];
          doCheck = true;
          installPhase = "mkdir $out";
        };

        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cmake pkgs.pkg-config pkgs.nlohmann_json ];
        };
      });
}
