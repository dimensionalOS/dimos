{
  description = "Project dev environment as Nix shell + DockerTools layered image";

  inputs = {
    nixpkgs.url      = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url  = "github:numtide/flake-utils";
    lib.url          = "github:jeff-hykin/quick-nix-toolkits";
    lib.inputs.flakeUtils.follows = "flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, lib, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        
        # ------------------------------------------------------------
        # 1. Shared package list (tool-chain + project deps)
        # ------------------------------------------------------------
        # we "flag" each package with what we need it for (e.g. LD_LIBRARY_PATH, nativeBuildInputs vs buildInputs, etc)
        aggregation = lib.aggregator [
          ### Core shell & utils
          { vals.pkg=pkgs.bashInteractive;    flags={}; }
          { vals.pkg=pkgs.coreutils;          flags={}; }
          { vals.pkg=pkgs.gh;                 flags={}; }
          { vals.pkg=pkgs.stdenv.cc.cc.lib;   flags.ldLibraryGroup=true; }
          { vals.pkg=pkgs.pcre2;              flags={ ldLibraryGroup=true; flags.packageConfGroup=pkgs.stdenv.isDarwin; }; }
          { vals.pkg=pkgs.libsysprof-capture; flags.packageConfGroup=true; onlyIf=pkgs.stdenv.isDarwin; }
          { vals.pkg=pkgs.git-lfs;            flags={}; }
          { vals.pkg=pkgs.unixtools.ifconfig; flags={}; }
          { vals.pkg=pkgs.unixtools.netstat;  flags={}; }

          ### Python + static analysis
          { vals.pkg=pkgs.python312;                    flags={}; }
          { vals.pkg=pkgs.python312Packages.pip;        flags={}; }
          { vals.pkg=pkgs.python312Packages.setuptools; flags={}; }
          { vals.pkg=pkgs.python312Packages.virtualenv; flags={}; }
          { vals.pkg=pkgs.pre-commit;                   flags={}; }

          ### Runtime deps
          { vals.pkg=pkgs.python312Packages.pyaudio; flags={}; }
          { vals.pkg=pkgs.portaudio;                 flags={}; }
          { vals.pkg=pkgs.ffmpeg_6;                  flags={}; }
          { vals.pkg=pkgs.ffmpeg_6.dev;              flags={}; }
          
          ### Graphics / X11 stack
          { vals.pkg=pkgs.libGL;              flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.libGLU;             flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.mesa;               flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.glfw;               flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libX11;        flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXi;         flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXext;       flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXrandr;     flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXinerama;   flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXcursor;    flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXfixes;     flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXrender;    flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXdamage;    flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXcomposite; flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libxcb;        flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXScrnSaver; flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.xorg.libXxf86vm;    flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.udev;               flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.SDL2;               flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.SDL2.dev;           flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.zlib;               flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }

          ### GTK / OpenCV helpers
          { vals.pkg=pkgs.glib;                  flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.gtk3;                  flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.gdk-pixbuf;            flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.gobject-introspection; flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          
          ### GStreamer
          { vals.pkg=pkgs.gst_all_1.gstreamer;          flags.ldLibraryGroup=true; flags.giTypelibGroup=true; }
          { vals.pkg=pkgs.gst_all_1.gst-plugins-base;   flags.ldLibraryGroup=true; flags.giTypelibGroup=true; }
          { vals.pkg=pkgs.gst_all_1.gst-plugins-good;   flags={}; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.gst_all_1.gst-plugins-bad;    flags={}; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.gst_all_1.gst-plugins-ugly;   flags={}; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.python312Packages.gst-python; flags={}; onlyIf=pkgs.stdenv.isLinux; }

          ### Open3D & build-time
          { vals.pkg=pkgs.eigen;   flags={}; }
          { vals.pkg=pkgs.cmake;   flags={}; }
          { vals.pkg=pkgs.ninja;   flags={}; }
          { vals.pkg=pkgs.jsoncpp; flags={}; }
          { vals.pkg=pkgs.libjpeg; flags={}; }
          { vals.pkg=pkgs.libpng;  flags={}; }
          
          ### LCM (Lightweight Communications and Marshalling)
          { vals.pkg=pkgs.lcm; flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          # lcm works on darwin, but only after two fixes (1. pkg-config, 2. fsync)
          {
            onlyIf=pkgs.stdenv.isDarwin;
            flags.ldLibraryGroup=true;
            vals.pkg=pkgs.lcm.overrideAttrs (old: 
                let 
                    # 1. fix pkg-config on darwin
                    pkgConfPackages = aggregation.getAll { hasAllFlags=[ "packageConfGroup" ]; attrPath=[ "pkg" ]; };
                    packageConfPackagesString = lib.print { prefix="packageConfPackagesString"; } (aggregation.getAll {
                        hasAllFlags=[ "packageConfGroup" ];
                        attrPath=[ "pkg" ];
                        strAppend="/lib/pkgconfig";
                        strJoin=":"; 
                    });
                in
                    {
                        buildInputs = (old.buildInputs or []) ++ pkgConfPackages;
                        nativeBuildInputs = (old.nativeBuildInputs or []) ++ [ pkgs.pkg-config pkgs.python312 ];
                        # 1. fix pkg-config on darwin
                        env.PKG_CONFIG_PATH = packageConfPackagesString;
                        # 2. Fix fsync on darwin
                        patches = [
                            (pkgs.writeText "lcm-darwin-fsync.patch" "--- ./lcm-logger/lcm_logger.c     2025-11-14 09:46:01.000000000 -0600\n+++ ./lcm-logger/lcm_logger.c  2025-11-14 09:47:05.000000000 -0600\n@@ -428,9 +428,13 @@\n         if (needs_flushed) {\n             fflush(logger->log->f);\n #ifndef WIN32\n+#ifdef __APPLE__\n+            fsync(fileno(logger->log->f));\n+#else\n             // Perform a full fsync operation after flush\n             fdatasync(fileno(logger->log->f));\n #endif\n+#endif\n             logger->last_fflush_time = log_event->timestamp;\n         }\n")
                        ];
                    }
            ); 
          }
        ];
        
        # ------------------------------------------------------------
        # 2. group / aggregate our packages
        # ------------------------------------------------------------
        devPackages = aggregation.getAll { attrPath=[ "pkg" ]; };
        ldLibraryPackages = aggregation.getAll { hasAllFlags=[ "ldLibraryGroup" ]; attrPath=[ "pkg" ]; };
        giTypelibPackagesString = aggregation.getAll {
          hasAllFlags=[ "giTypelibGroup" ];
          attrPath=[ "pkg" ];
          strAppend="/lib/girepository-1.0";
          strJoin=":"; 
        };

        # ------------------------------------------------------------
        # 3. Host interactive shell  →  `nix develop`
        # ------------------------------------------------------------
        devShell = pkgs.mkShell {
          packages = devPackages;
          shellHook = ''
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath ldLibraryPackages}:$LD_LIBRARY_PATH"
            export DISPLAY=:0
            export GI_TYPELIB_PATH="${giTypelibPackagesString}:$GI_TYPELIB_PATH" 
            PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo "$PWD")
            if [ -f "$PROJECT_ROOT/env/bin/activate" ]; then
              . "$PROJECT_ROOT/env/bin/activate"
            fi
            
            # without this alias, the pytest uses the non-venv python and fails
            alias pytest="python -m pytest"
            [ -f "$PROJECT_ROOT/motd" ] && cat "$PROJECT_ROOT/motd"
            [ -f "$PROJECT_ROOT/.pre-commit-config.yaml" ] && pre-commit install --install-hooks
          '';
        };

        # ------------------------------------------------------------
        # 4. Closure copied into the OCI image rootfs
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
