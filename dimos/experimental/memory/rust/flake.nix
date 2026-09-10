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
      dimos-memory-recorder = pkgs.rustPlatform.buildRustPackage {
        pname = "dimos-memory-recorder";
        version = "0.1.0";
        src = pkgs.lib.fileset.toSource {
          root = ../../../..;
          fileset = pkgs.lib.fileset.unions [
            ../../../../Cargo.lock
            ../../../../Cargo.toml
            ../../../../dimos/experimental/memory/rust
            ../../../../dimos/hardware/sensors/lidar/virtual_mid360
            ../../../../dimos/mapping/ray_tracing/rust
            ../../../../dimos/navigation/nav_3d/mls_planner/rust
            ../../../../examples/native-modules/rust
            ../../../../native/rust/dimos-module
            ../../../../native/rust/dimos-module-macros
          ];
        };

        cargoLock = {
          lockFile = ../../../../Cargo.lock;
          outputHashes = {
            "dimos-lcm-0.1.0" = "sha256-GGkx4Mn6NYP6KZecmoRLKGWIih/+y8OgNn12DeXX6n8=";
          };
        };

        cargoBuildFlags = ["-p" "dimos-memory-recorder"];
        cargoTestFlags = ["-p" "dimos-memory-recorder"];
        strictDeps = true;

        nativeBuildInputs = [
          pkgs.cmake
          pkgs.nasm
          pkgs.pkg-config
        ];
        buildInputs = [
          pkgs.sqlite
          pkgs.sqlite.dev
        ] ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isDarwin [pkgs.libiconv];

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
    });
}
