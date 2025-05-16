{
  description = "Project dev environment as shell + package.";

  inputs = {
    nixpkgs     .url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils .url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        commonEnv = pkgs.mkShell {
          buildInputs = [
            pkgs.stdenv.cc.cc.lib
            pkgs.python312
            pkgs.python312Packages.pip
            pkgs.python312Packages.setuptools
            pkgs.python312Packages.virtualenv
            pkgs.python312Packages.pyaudio
            pkgs.portaudio
            pkgs.ffmpeg_6
            pkgs.ffmpeg_6.dev

            # OpenGL / graphics
            pkgs.libGL
            pkgs.libGLU
            pkgs.mesa
            pkgs.glfw
            pkgs.xorg.libX11
            pkgs.xorg.libXi
            pkgs.xorg.libXext
            pkgs.xorg.libXrandr
            pkgs.xorg.libXinerama
            pkgs.xorg.libXcursor
            pkgs.xorg.libXfixes
            pkgs.xorg.libXrender
            pkgs.xorg.libXdamage
            pkgs.xorg.libXcomposite
            pkgs.xorg.libxcb
            pkgs.xorg.libXScrnSaver
            pkgs.xorg.libXxf86vm

            # Misc runtime deps
            pkgs.udev
            pkgs.SDL2
            pkgs.SDL2.dev
            pkgs.zlib
            
            # GTK/GLib dependencies for OpenCV
            pkgs.glib
            pkgs.gtk3
            pkgs.gdk-pixbuf
            pkgs.gobject-introspection

            # Open3D build-time deps
            pkgs.eigen
            pkgs.cmake
            pkgs.ninja
            pkgs.jsoncpp
            pkgs.libjpeg
            pkgs.libpng
          ];

          shellHook = ''
            export LD_LIBRARY_PATH="${
              pkgs.lib.makeLibraryPath [
                pkgs.stdenv.cc.cc.lib
                pkgs.libGL pkgs.libGLU pkgs.mesa pkgs.glfw
                pkgs.xorg.libX11 pkgs.xorg.libXi pkgs.xorg.libXext
                pkgs.xorg.libXrandr pkgs.xorg.libXinerama pkgs.xorg.libXcursor
                pkgs.xorg.libXfixes pkgs.xorg.libXrender pkgs.xorg.libXdamage
                pkgs.xorg.libXcomposite pkgs.xorg.libXxf86vm pkgs.xorg.libxcb
                pkgs.xorg.libXScrnSaver
                pkgs.udev pkgs.portaudio pkgs.SDL2.dev pkgs.zlib
                pkgs.glib pkgs.gtk3 pkgs.gdk-pixbuf pkgs.gobject-introspection
              ]
            }:$LD_LIBRARY_PATH"

            export DISPLAY=:0

            # project root + optional venv auto-activate
            PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo "$PWD")
            if [ -f "$PROJECT_ROOT/env/bin/activate" ]; then
              source "$PROJECT_ROOT/env/bin/activate"
            fi

            cat $PROJECT_ROOT/motd
          '';
        };
      in {
        devShells.default = commonEnv;
        packages.default = commonEnv;
      });
}

