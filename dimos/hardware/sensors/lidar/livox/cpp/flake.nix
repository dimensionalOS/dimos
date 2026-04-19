{
  description = "Livox SDK2 and Mid-360 native module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        livox-sdk2 = pkgs.stdenv.mkDerivation rec {
          pname = "livox-sdk2";
          version = "1.2.5";

          src = pkgs.fetchFromGitHub {
            owner = "Livox-SDK";
            repo = "Livox-SDK2";
            rev = "v${version}";
            hash = "sha256-NGscO/vLiQ17yQJtdPyFzhhMGE89AJ9kTL5cSun/bpU=";
          };

          nativeBuildInputs = [ pkgs.cmake ];

          # Workaround for kernel 5.10 (Rockchip RK3588) missing fchmodat2 syscall.
          # nixpkgs coreutils is built against glibc 2.42 which uses fchmodat2;
          # kernel 5.10 returns ENOSYS for it.  Use host coreutils (glibc 2.31
          # on Ubuntu 22.04) via absolute paths to bypass the syscall.  Mirrors
          # the pattern applied in drdds_bridge and fastlio2 flakes — see
          # plans/m20-rosnav-migration/06-simplify-remove-container/nix-arm64-kernel510-workaround.md
          unpackPhase = ''
            /usr/bin/cp -r $src source
            /usr/bin/chmod -R u+w source
            cd source
          '';
          fixupPhase = ''
            /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
            /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
          '';

          cmakeFlags = [
            "-DBUILD_SHARED_LIBS=ON"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
          ];

          preConfigure = ''
            substituteInPlace CMakeLists.txt \
              --replace-fail "add_subdirectory(samples)" ""
            sed -i '1i #include <cstdint>' sdk_core/comm/define.h
            sed -i '1i #include <cstdint>' sdk_core/logger_handler/file_manager.h
          '';
        };

        livox-common = ../../common;

        mid360_native = pkgs.stdenv.mkDerivation {
          pname = "mid360_native";
          version = "0.1.0";

          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ livox-sdk2 pkgs.lcm pkgs.glib ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DLIVOX_COMMON_DIR=${livox-common}"
          ];
        };
      in {
        packages = {
          default = mid360_native;
          inherit livox-sdk2 mid360_native;
        };

        devShells.default = pkgs.mkShell {
          buildInputs = [ livox-sdk2 ];
        };
      });
}
