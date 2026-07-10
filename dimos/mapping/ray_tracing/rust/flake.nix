{
  description = "Voxel ray tracing native module for DimOS";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
    dimos-repo = { url = "path:../../../.."; flake = false; };
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay, dimos-repo }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ rust-overlay.overlays.default ];
        };
        rustToolchain = pkgs.rust-bin.stable.latest.default;
        rustPlatform = pkgs.makeRustPlatform {
          cargo = rustToolchain;
          rustc = rustToolchain;
        };

        src = pkgs.runCommand "voxel-ray-tracing-src" {} ''
          mkdir -p $out/dimos/mapping/ray_tracing/rust
          cp -r ${./src} $out/dimos/mapping/ray_tracing/rust/src
          cp ${./Cargo.toml} $out/dimos/mapping/ray_tracing/rust/Cargo.toml
          cp ${./Cargo.lock} $out/dimos/mapping/ray_tracing/rust/Cargo.lock

          mkdir -p $out/native/rust
          cp -r ${dimos-repo}/native/rust/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-repo}/native/rust/dimos-module-macros $out/native/rust/dimos-module-macros
        '';
      in {
        packages.default = rustPlatform.buildRustPackage {
          pname = "voxel-ray-tracing";
          version = "0.1.0";

          inherit src;
          cargoRoot = "dimos/mapping/ray_tracing/rust";
          buildAndTestSubdir = "dimos/mapping/ray_tracing/rust";

          cargoLock = {
            lockFile = ./Cargo.lock;
            outputHashes = {
              "dimos-lcm-0.1.0" = "sha256-4DWFTf7Xqnx6pd2jXA/MVpRmZiFr6HqTSp9Qo9ZjToA=";
              "lcm-msgs-0.1.0" = pkgs.lib.fakeHash;
            };
          };

          meta.mainProgram = "voxel_ray_tracing";
        };
      });
}
