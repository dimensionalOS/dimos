{
  description = "Fake Livox Mid-360 pcap replayer (virtual NIC) native module for DimOS";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # Relative git+file: will be deprecated (nix#12281) but there's no
    # viable alternative for reaching local path deps outside the flake dir currently
    # presumably an alternative will be added before this is removed.
    dimos-repo = { url = "git+file:../../../../..?ref=main"; flake = false; };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-repo }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        sub = "dimos/hardware/sensors/lidar/virtual_mid360";

        src = pkgs.runCommand "virtual-mid360-src" {} ''
          mkdir -p $out/${sub}
          cp -r ${./src} $out/${sub}/src
          cp ${./Cargo.toml} $out/${sub}/Cargo.toml
          cp ${./Cargo.lock} $out/${sub}/Cargo.lock

          mkdir -p $out/native/rust
          cp -r ${dimos-repo}/native/rust/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-repo}/native/rust/dimos-module-macros $out/native/rust/dimos-module-macros
        '';
      in {
        # Toolchain-only shell for CI fmt/clippy/test. Deliberately avoids the
        # `src` runCommand (its relative path: input can't resolve once the
        # flake is copied to the store), so `nix develop` stays cheap.
        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ];
        };

        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "virtual-mid360";
          version = "0.1.0";

          inherit src;
          cargoRoot = sub;
          buildAndTestSubdir = sub;

          # Vendor straight from Cargo.lock. nix's fetchurl sends a User-Agent,
          # so crates.io won't 403 it the way nixpkgs' fetchCargoVendor util does.
          # The dimos-lcm git dep needs its fetched tree hash pinned here.
          cargoLock = {
            lockFile = ./Cargo.lock;
            outputHashes = {
              "dimos-lcm-0.1.0" = "sha256-GGkx4Mn6NYP6KZecmoRLKGWIih/+y8OgNn12DeXX6n8=";
            };
          };

          meta.mainProgram = "virtual_mid360";
        };
      });
}
