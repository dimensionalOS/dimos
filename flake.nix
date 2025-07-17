{
  description = "Project dev environment as Nix shell + DockerTools layered image";

  inputs = {
    nixpkgs.url      = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url  = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # ------------------------------------------------------------
        # 1. Shared package list (tool-chain + project deps)
        # ------------------------------------------------------------
        devPackages = with pkgs; [
          ### Core shell & utils
          bashInteractive coreutils gh
          stdenv.cc.cc.lib

          ### Python + static analysis
          python312 python312Packages.pip python312Packages.setuptools
          python312Packages.virtualenv pre-commit

          ### Runtime deps
          python312Packages.pyaudio portaudio ffmpeg_6 ffmpeg_6.dev

          ### Graphics / X11 stack
          libGL libGLU mesa glfw
          xorg.libX11 xorg.libXi xorg.libXext xorg.libXrandr xorg.libXinerama
          xorg.libXcursor xorg.libXfixes xorg.libXrender xorg.libXdamage
          xorg.libXcomposite xorg.libxcb xorg.libXScrnSaver xorg.libXxf86vm

          udev SDL2 SDL2.dev zlib

          ### GTK / OpenCV helpers
          glib gtk3 gdk-pixbuf gobject-introspection

          ### Open3D & build-time
          eigen cmake ninja jsoncpp libjpeg libpng
        ];

        # ------------------------------------------------------------
        # 2. Host interactive shell  →  `nix develop`
        # ------------------------------------------------------------
        devShell = pkgs.mkShell {
          packages = devPackages;
          shellHook = ''
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath [
              pkgs.stdenv.cc.cc.lib pkgs.libGL pkgs.libGLU pkgs.mesa pkgs.glfw
              pkgs.xorg.libX11 pkgs.xorg.libXi pkgs.xorg.libXext pkgs.xorg.libXrandr
              pkgs.xorg.libXinerama pkgs.xorg.libXcursor pkgs.xorg.libXfixes
              pkgs.xorg.libXrender pkgs.xorg.libXdamage pkgs.xorg.libXcomposite
              pkgs.xorg.libxcb pkgs.xorg.libXScrnSaver pkgs.xorg.libXxf86vm
              pkgs.udev pkgs.portaudio pkgs.SDL2.dev pkgs.zlib pkgs.glib pkgs.gtk3
              pkgs.gdk-pixbuf pkgs.gobject-introspection]}:$LD_LIBRARY_PATH"

            export DISPLAY=:0

            PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo "$PWD")
            if [ -f "$PROJECT_ROOT/env/bin/activate" ]; then
              . "$PROJECT_ROOT/env/bin/activate"
            fi

            [ -f "$PROJECT_ROOT/motd" ] && cat "$PROJECT_ROOT/motd"
            [ -f "$PROJECT_ROOT/.pre-commit-config.yaml" ] && pre-commit install --install-hooks
          '';
        };

        # ------------------------------------------------------------
        # 3. Closure copied into the OCI image rootfs
        # ------------------------------------------------------------
        imageRoot = pkgs.buildEnv {
          name = "dimos-image-root";
          paths = devPackages;
          pathsToLink = [ "/bin" ];
        };

      in {
        ## Local dev shell
        devShells.default = devShell;

        ## Layered docker image with DockerTools
        packages.devcontainer = pkgs.dockerTools.buildLayeredImage {
          name      = "dimensionalos/dimos-dev";
          tag       = "latest";
          contents  = [ imageRoot ];
          config = {
            WorkingDir = "/workspace";
            Cmd        = [ "bash" ];
          };
        };
      });
}
