{
  description = "Voxel ray tracing native module for DimOS";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # Relative git+file: will be deprecated (nix#12281) but there's no
    # viable alternative for reaching local path deps outside the flake dir currently
    # presumably an alternative will be added before this is removed.
    dimos-repo = { url = "git+file:../../../..?ref=main"; flake = false; };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-repo }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

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
        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "voxel-ray-tracing";
          version = "0.1.0";

          inherit src;
          cargoRoot = "dimos/mapping/ray_tracing/rust";
          buildAndTestSubdir = "dimos/mapping/ray_tracing/rust";

          cargoHash = "sha256-6a8GHRSKI6mjg9HNbrestCud8xZtF8HaD0bWVMbl7N8=";

          meta.mainProgram = "voxel_ray_tracing";
        };
      });
}
