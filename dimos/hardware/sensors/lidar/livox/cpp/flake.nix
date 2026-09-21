{
  description = "Livox SDK2 packaging, consumed by the C++ LIO module flakes";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
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

          # macOS socket fixes (SO_RCVBUF too large, broadcast bind fails).
          patches = [ ./livox-sdk2-darwin.patch ];

          nativeBuildInputs = [ pkgs.cmake ];

          cmakeFlags = [
            "-DBUILD_SHARED_LIBS=ON"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
          ];

          preConfigure = ''
            substituteInPlace CMakeLists.txt \
              --replace-fail "add_subdirectory(samples)" ""
            sed -i '1i #include <cstdint>' sdk_core/comm/define.h
            sed -i '1i #include <cstdint>' sdk_core/logger_handler/file_manager.h
            # Livox-SDK2 bundles an old rapidjson whose RAPIDJSON_DIAG_OFF(foo-bar)
            # macros stringify with spaces under newer clang, producing invalid
            # warning-group names.  It also has an unused FastCRC field.  Both
            # explode under -Werror, and passing -DCMAKE_CXX_FLAGS=-Wno-error is
            # overridden by add_compile_options(-Werror) deeper in the sdk_core
            # CMakeLists.  Strip -Werror in-place instead.
            find . -name CMakeLists.txt -exec sed -i 's/-Werror//g' {} +
          '';
        };
      in {
        packages = {
          default = livox-sdk2;
          inherit livox-sdk2;
        };
      });
}
