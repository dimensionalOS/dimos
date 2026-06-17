{
  description = "Rust native module ping-pong example";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        repoRoot = ../../..;
        src = pkgs.lib.fileset.toSource {
          root = repoRoot;
          fileset = pkgs.lib.fileset.unions [
            (repoRoot + /examples/native-modules/rust)
            (repoRoot + /native/rust)
          ];
        };
      in {
        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "dimos-native-module-examples";
          version = "0.1.0";
          inherit src;
          sourceRoot = "source/examples/native-modules/rust";
          cargoLock = {
            lockFile = ./Cargo.lock;
            outputHashes = {
              "dimos-lcm-0.1.0" = "sha256-4DWFTf7Xqnx6pd2jXA/MVpRmZiFr6HqTSp9Qo9ZjToA=";
              "lcm-msgs-0.1.0" = "sha256-4DWFTf7Xqnx6pd2jXA/MVpRmZiFr6HqTSp9Qo9ZjToA=";
            };
          };
        };
      });
}
