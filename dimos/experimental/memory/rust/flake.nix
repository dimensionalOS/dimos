{
  description = "DimOS Rust native modules";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
  }:
    flake-utils.lib.eachSystem [
      "x86_64-linux"
      "aarch64-linux"
      "aarch64-darwin"
    ] (system: let
      pkgs = nixpkgs.legacyPackages.${system};
      nativeDeps = [pkgs.cmake pkgs.nasm pkgs.pkg-config];
      systemDeps =
        [pkgs.sqlite pkgs.sqlite.dev]
        ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isDarwin [pkgs.libiconv];
      dimos-memory-recorder = pkgs.rustPlatform.buildRustPackage {
        pname = "dimos-memory-recorder";
        version = "0.1.0";
        src = pkgs.lib.fileset.toSource {
          root = ../../../..;
          fileset = pkgs.lib.fileset.unions [
            ../../../../Cargo.lock
            ../../../../Cargo.toml
            ../../../../dimos/experimental/memory/rust
            ../../../../native/rust/dimos-module
            ../../../../native/rust/dimos-lcm-transport
            ../../../../native/rust/dimos-module-macros
            ../../../../dimos/mapping/ray_tracing/rust
            ../../../../dimos/mapping/ray_tracing/rust/py
            ../../../../dimos/navigation/global_planner/mls_planner/rust
            ../../../../dimos/navigation/global_planner/mls_planner/rust/py
            ../../../../dimos/hardware/sensors/lidar/livox/rust
            ../../../../dimos/hardware/sensors/lidar/pointlio/rust
            ../../../../dimos/hardware/sensors/lidar/virtual_mid360
            ../../../../examples/native-modules/rust
          ];
        };

        cargoLock = {
          lockFile = ../../../../Cargo.lock;
          outputHashes = {
            "dimos-lcm-0.1.0" = "sha256-GGkx4Mn6NYP6KZecmoRLKGWIih/+y8OgNn12DeXX6n8=";
            "pointlio-core-0.1.0" = "sha256-iC7nDbEipfi3cViK7fqKiy2hT9ENGi4Ge7L6Wt1W01Q=";
          };
        };

        cargoBuildFlags = ["-p" "dimos-memory-recorder"];
        cargoTestFlags = ["-p" "dimos-memory-recorder"];
        strictDeps = true;

        nativeBuildInputs = nativeDeps;
        buildInputs = systemDeps;

        env.LIBSQLITE3_SYS_USE_PKG_CONFIG = "1";

        meta = {
          description = "Experimental native Memory2 SQLite and MCAP recorder";
          mainProgram = "dimos-memory-recorder";
          platforms = pkgs.lib.platforms.unix;
        };
      };
    in {
      packages = {
        default = dimos-memory-recorder;
        inherit dimos-memory-recorder;
      };

      # Just the system deps and a toolchain. Deliberately not the package's
      # build environment: that vendors the whole workspace lock, which would
      # make `nix develop` (and so the cargo clippy hook) fail on any git
      # dependency added anywhere in the workspace.
      devShells.default = pkgs.mkShell {
        nativeBuildInputs = nativeDeps ++ [pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt];
        buildInputs = systemDeps;
        LIBSQLITE3_SYS_USE_PKG_CONFIG = "1";
      };
    });
}
