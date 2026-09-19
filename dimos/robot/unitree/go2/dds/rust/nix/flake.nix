{
  description = "Build shell for the dimos Go2 DDS native module";

  # Its own directory: a `path:` ref copies the whole tree into the store each time,
  # and the crate root holds target/. Locked to the repo-root flake's nixpkgs rev.
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      # bindgenHook points cyclonedds-sys's bindgen at libclang and the include dirs.
      devShells = forAll (pkgs: {
        default = pkgs.mkShell {
          packages = [
            pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt pkgs.pkg-config pkgs.cmake
            pkgs.cyclonedds pkgs.rustPlatform.bindgenHook
          ];
          shellHook = ''
            export CYCLONEDDS_HOME="${pkgs.cyclonedds}"
            export CYCLONEDDS_LIB_DIR="${pkgs.cyclonedds}/lib"
            export CYCLONEDDS_INCLUDE_DIR="${pkgs.cyclonedds}/include"
          '';
        };
      });
    };
}
