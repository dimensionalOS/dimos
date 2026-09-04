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
      # resolve the members straight from the workspace
      workspaceMembers =
        (builtins.fromTOML (builtins.readFile ../../../../Cargo.toml)).workspace.members;
      dimos-memory-recorder = pkgs.rustPlatform.buildRustPackage {
        pname = "dimos-memory-recorder";
        version = "0.1.0";
        src = pkgs.lib.fileset.toSource {
          root = ../../../..;
          fileset = pkgs.lib.fileset.unions (
            [
              ../../../../Cargo.lock
              ../../../../Cargo.toml
            ]
            ++ map (member: ../../../.. + "/${member}") workspaceMembers
          );
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
