{
  description = "librealsense for the dimos RealSense native module";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      # The rust toolchain comes from the enclosing dimos shell; this only adds
      # what realsense-sys links against.
      devShells = forAll (pkgs: {
        default = pkgs.mkShellNoCC { packages = [ pkgs.librealsense pkgs.pkg-config ]; };
      });
    };
}
