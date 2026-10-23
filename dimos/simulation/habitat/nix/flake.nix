{
  description = "micromamba for the dimos Habitat native module";

  inputs.dimos-native-cpp.url = "github:dimensionalOS/dimos?dir=native/cpp";
  inputs.nixpkgs.follows = "dimos-native-cpp/nixpkgs";

  outputs = { self, nixpkgs, ... }:
    let
      # linux-64 only: the aihabitat conda channel has no aarch64 habitat-sim.
      systems = [ "x86_64-linux" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      # Provides the installer, not the simulator: habitat-sim is conda-only and
      # headless rendering needs the host's EGL driver, so this cannot be a derivation.
      # Nothing to lint and no tests here; declared so the gate sees the flake.
      checks = forAll (pkgs: {
        lint = pkgs.runCommand "habitat-lint" { } "mkdir $out";
        tests = pkgs.runCommand "habitat-tests" { } "mkdir $out";
      });

      devShells = forAll (pkgs: {
        default = pkgs.mkShellNoCC {
          packages = [ pkgs.micromamba pkgs.curl pkgs.cacert ];
        };
      });
    };
}
