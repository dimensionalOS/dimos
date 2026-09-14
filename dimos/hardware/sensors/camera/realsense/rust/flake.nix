{
  description = "librealsense for the dimos RealSense native module";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      # Also works when NativeModule builds from a regular Python environment.
      # Use the Nix compiler/linker so libc matches the camera libraries.
      devShells = forAll (pkgs: {
        default = pkgs.mkShell { packages = [ pkgs.cargo pkgs.rustc pkgs.librealsense pkgs.pkg-config ]; };
      });
    };
}
