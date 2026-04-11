{
  inputs = {
    arise-slam.url = "github:dimensionalOS/dimos-module-arise-slam/v0.1.0";
  };

  outputs = { self, arise-slam, ... }: let
    system = "aarch64-linux";
    pkg = arise-slam.packages.${system}.default;
  in {
    packages.${system}.default = pkg.overrideAttrs (old: {
      # Workaround: nix's coreutils (glibc 2.42) uses fchmodat2 which
      # kernel 5.10 (Rockchip RK3588) doesn't have. Use host binaries.
      unpackPhase = ''
        /usr/bin/cp -r $src source
        /usr/bin/chmod -R u+w source
        cd source
      '';
      fixupPhase = ''
        /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
        /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
      '';
    });
  };
}
