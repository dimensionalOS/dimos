{
  description = "The shared dimos C++ native module SDK, as a source tree for cmake";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ] (system:
      let pkgs = nixpkgs.legacyPackages.${system}; in {
        packages.default = pkgs.runCommand "dimos-native-cpp" { } ''
          cp -r ${nix-filter.lib { root = ./.; exclude = [ "build" "target" "result" "__pycache__" ]; }} $out
          chmod -R u+w $out
        '';

        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cmake pkgs.pkg-config pkgs.nlohmann_json ];
        };
      });
}
