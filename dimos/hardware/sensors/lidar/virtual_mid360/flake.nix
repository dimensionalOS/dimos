{
  description = "Fake Livox Mid-360 pcap replayer (virtual NIC) native module for DimOS";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # Path input to the in-repo rust crates (mirrors pointlio/cpp's path: inputs).
    # A plain path: (not git+file:) hashes the directory contents, so it carries no
    # git-tree NAR hash — which varies by nix version / clean-vs-dirty checkout and
    # breaks cross-machine builds.
    dimos-rust = { url = "path:../../../../../native/rust"; flake = false; };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-rust }:
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
          cp -r ${dimos-rust}/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-rust}/dimos-module-macros $out/native/rust/dimos-module-macros
        '';
      in {
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
              "dimos-lcm-0.1.0" = "sha256-4DWFTf7Xqnx6pd2jXA/MVpRmZiFr6HqTSp9Qo9ZjToA=";
            };
          };

          meta.mainProgram = "virtual_mid360";
        };
      });
}
