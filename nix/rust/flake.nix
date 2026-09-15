{
  description = "Rust toolchain for the dimos cargo workspace";

  # Deliberately a tiny flake of its own rather than the repo-root one: a flake
  # ref copies its entire source tree into the nix store, and a fresh copy every
  # time that tree changes. Pointed at the repo root that means the build dirs,
  # .venv and .git (`path:` refs copy everything) or the tracked data/.lfs blobs
  # (git refs copy tracked files) -- tens of gigabytes per build. This directory
  # is two files, so the snapshot is stable and free.
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      # Locked to the same nixpkgs rev as the repo-root flake, so this is the
      # same toolchain the dev shell hands out rather than a second download.
      devShells = forAll (pkgs: {
        default = pkgs.mkShell {
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt pkgs.pkg-config ];
        };
      });
    };
}
