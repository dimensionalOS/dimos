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
        # Toolchain-only shell for CI fmt/clippy/test, kept independent of the
        # `src` runCommand so `nix develop` stays cheap.
        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ];
        };

        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "voxel-ray-tracing";
          version = "0.1.0";

          inherit src;
          cargoRoot = "dimos/mapping/ray_tracing/rust";
          buildAndTestSubdir = "dimos/mapping/ray_tracing/rust";

          # Vendor straight from Cargo.lock so no crates.io vendor hash needs
          # recomputing when a dependency changes. Only the dimos-lcm git dep
          # needs its fetched tree hash pinned, keyed by any one crate from that
          # rev (nixpkgs maps the key to the commit SHA, covering lcm-msgs too).
          cargoLock = {
            lockFile = ./Cargo.lock;
            outputHashes = {
              "dimos-lcm-0.1.0" = "sha256-4DWFTf7Xqnx6pd2jXA/MVpRmZiFr6HqTSp9Qo9ZjToA=";
            };
          };

          # macOS: pyo3's extension-module leaves CPython symbols undefined in the
          # cdylib; let the loader resolve them at import time.
          env.RUSTFLAGS = nixpkgs.lib.optionalString pkgs.stdenv.isDarwin
            "-C link-args=-Wl,-undefined,dynamic_lookup";

          meta.mainProgram = "voxel_ray_tracing";
        };
      });
}
