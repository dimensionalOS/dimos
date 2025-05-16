{
  description = "Project dev environment as Nix shell and Dev Container";

  inputs = {
    nixpkgs.url        = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url    = "github:numtide/flake-utils";
    nix2container.url  = "github:nlewo/nix2container";
  };

  outputs = { self, nixpkgs, flake-utils, nix2container, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # ------------------------------------------------------------
        # All runtime / build‑time packages live here so we can share
        # them between the dev‑shell **and** the OCI image.
        # ------------------------------------------------------------
        devPackages = with pkgs; [
          stdenv.cc.cc.lib

          # — Python tool‑chain & unified checks —
          python312
          python312Packages.pip
          python312Packages.setuptools
          python312Packages.virtualenv
          ruff          # ≥0.11: ships its own `ruff server`
          mypy
          pre-commit

          python312Packages.pyaudio  # project runtime deps
          portaudio
          ffmpeg_6 ffmpeg_6.dev

          # — Graphics / X11 & friends —
          libGL libGLU mesa glfw
          xorg.libX11 xorg.libXi xorg.libXext xorg.libXrandr xorg.libXinerama
          xorg.libXcursor xorg.libXfixes xorg.libXrender xorg.libXdamage
          xorg.libXcomposite xorg.libxcb xorg.libXScrnSaver xorg.libXxf86vm

          udev SDL2 SDL2.dev zlib

          # GTK/GLib stack for OpenCV, etc.
          glib gtk3 gdk-pixbuf gobject-introspection

          # Open3D & misc build deps
          eigen cmake ninja jsoncpp libjpeg libpng
        ];

        # ------------------------------------------------------------
        # Host‑side interactive shell  →  `nix develop` / direnv
        # ------------------------------------------------------------
        devShell = pkgs.mkShell {
          packages  = devPackages;

          shellHook = ''
            # Make shared libs discoverable at runtime
            export LD_LIBRARY_PATH="${
              pkgs.lib.makeLibraryPath [
                pkgs.stdenv.cc.cc.lib
                pkgs.libGL pkgs.libGLU pkgs.mesa pkgs.glfw
                pkgs.xorg.libX11 pkgs.xorg.libXi pkgs.xorg.libXext
                pkgs.xorg.libXrandr pkgs.xorg.libXinerama pkgs.xorg.libXcursor
                pkgs.xorg.libXfixes pkgs.xorg.libXrender pkgs.xorg.libXdamage
                pkgs.xorg.libXcomposite pkgs.xorg.libxcb pkgs.xorg.libXScrnSaver
                pkgs.xorg.libXxf86vm
                pkgs.udev pkgs.portaudio pkgs.SDL2.dev pkgs.zlib
                pkgs.glib pkgs.gtk3 pkgs.gdk-pixbuf pkgs.gobject-introspection
              ]
            }:$LD_LIBRARY_PATH"

            export DISPLAY=:0

            # Detect repo root and auto‑activate local venv if present
            PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo "$PWD")
            if [ -f "$PROJECT_ROOT/env/bin/activate" ]; then
              . "$PROJECT_ROOT/env/bin/activate"
            fi

            # Optional MOTD banner
            [ -f "$PROJECT_ROOT/motd" ] && cat "$PROJECT_ROOT/motd"

            # Ensure git hooks match the flake‑pinned tool versions
            if [ -f "$PROJECT_ROOT/.pre-commit-config.yaml" ]; then
              pre-commit install --install-hooks
            fi
          '';
        };

        # ------------------------------------------------------------
        # Turn the *exact same* closure into an OCI image layer
        # ------------------------------------------------------------
        envClosure = pkgs.buildEnv {
          name  = "dimos-env";
          paths = devPackages;
        };

      in {
        # `nix develop` or direnv hooks into this
        devShells.default = devShell;

        # `nix build .#devcontainer` — or built by CI and pushed to GHCR
        packages.devcontainer = nix2container.packages.${system}.nix2container.buildImage {
          name      = "dimensional/dimos-dev";
          tag       = "latest";
          contents  = [ envClosure ];
          config.WorkingDir = "/workspace";  # aligns with Dev‑Container default
          config.Cmd        = [ "bash" ];
        };

        # You can optionally expose the image as the default build output:
        # packages.default = self.packages.${system}.devcontainer;
      });
}
