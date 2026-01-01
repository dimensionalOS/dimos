{
  description = "Project dev environment as Nix shell + DockerTools layered image";

  inputs = {
    nixpkgs.url      = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url  = "github:numtide/flake-utils";
    lib.url          = "github:jeff-hykin/quick-nix-toolkits";
    lib.inputs.flakeUtils.follows = "flake-utils";
    xome.url         = "github:jeff-hykin/xome";
    xome.inputs.nixpkgs.follows    = "nixpkgs";
    xome.inputs.flake-utils.follows = "flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, lib, xome, ... }:
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
          { vals.pkg=pkgs.stdenv.cc;          flags.ldLibraryGroup=true; }
          { vals.pkg=pkgs.cctools;            flags={}; onlyIf=pkgs.stdenv.isDarwin; } # for pip install opencv-python
          { vals.pkg=pkgs.pcre2;              flags={ ldLibraryGroup=true; packageConfGroup=pkgs.stdenv.isDarwin; }; }
          { vals.pkg=pkgs.libsysprof-capture; flags.packageConfGroup=true; onlyIf=pkgs.stdenv.isDarwin; }
          { vals.pkg=pkgs.xcbuild;            flags={}; }
          { vals.pkg=pkgs.git-lfs;            flags={}; }
          { vals.pkg=pkgs.gnugrep;            flags={}; }
          { vals.pkg=pkgs.gnused;             flags={}; }
          { vals.pkg=pkgs.iproute2;           flags={}; onlyIf=pkgs.stdenv.isLinux; }
          { vals.pkg=pkgs.pkg-config;         flags={}; }
          { vals.pkg=pkgs.git;                flags={}; }
          { vals.pkg=pkgs.opensshWithKerberos;flags={}; }
          { vals.pkg=pkgs.unixtools.ifconfig; flags={}; }
          { vals.pkg=pkgs.unixtools.netstat;  flags={}; }

          # when pip packages call cc with -I/usr/include, that causes problems on some machines, this swaps that out for the nix cc headers
          # this is only necessary for pip packages from venv, pip packages from nixpkgs.python312Packages.* already have "-I/usr/include" patched with the nix equivalent
          {
            vals.pkg=(pkgs.writeShellScriptBin
              "cc-no-usr-include"
              ''
                #!${pkgs.bash}/bin/bash
                set -euo pipefail

                real_cc="${pkgs.stdenv.cc}/bin/gcc"

                args=()
                for a in "$@"; do
                case "$a" in
                    -I/usr/include|-I/usr/local/include)
                    # drop these
                    ;;
                    *)
                    args+=("$a")
                    ;;
                esac
                done

                exec "$real_cc" "''${args[@]}"
              ''
            );
            flags={};
          }

          ### Python + static analysis
          { vals.pkg=pkgs.python312;                    flags={}; vals.pythonMinorVersion="12";}
          { vals.pkg=pkgs.python312Packages.pip;        flags={}; }
          { vals.pkg=pkgs.python312Packages.setuptools; flags={}; }
          { vals.pkg=pkgs.python312Packages.virtualenv; flags={}; }
          { vals.pkg=pkgs.pre-commit;                   flags={}; }

          ### Runtime deps
          { vals.pkg=pkgs.portaudio;                 flags={ldLibraryGroup=true; packageConfGroup=true;}; }
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
          { vals.pkg=pkgs.eigen;         flags={}; }
          { vals.pkg=pkgs.cmake;         flags={}; }
          { vals.pkg=pkgs.ninja;         flags={}; }
          { vals.pkg=pkgs.jsoncpp;       flags={}; }
          { vals.pkg=pkgs.libjpeg;       flags.ldLibraryGroup=true; }
          { vals.pkg=pkgs.libjpeg_turbo; flags.ldLibraryGroup=true; }
          { vals.pkg=pkgs.libpng;        flags={}; }

          ### LCM (Lightweight Communications and Marshalling)
          { vals.pkg=pkgs.lcm; flags.ldLibraryGroup=true; onlyIf=pkgs.stdenv.isLinux; }
          # lcm works on darwin, but only after two fixes (1. pkg-config, 2. fsync)
          {
            onlyIf=pkgs.stdenv.isDarwin;
            flags.ldLibraryGroup=true;
            flags.manualPythonPackages=true;
            vals.pkg=pkgs.lcm.overrideAttrs (old:
                let
                    # 1. fix pkg-config on darwin
                    pkgConfPackages = aggregation.getAll { hasAllFlags=[ "packageConfGroup" ]; attrPath=[ "pkg" ]; };
                    packageConfPackagesString = (aggregation.getAll {
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
        packageConfPackagesString = (aggregation.getAll {
            hasAllFlags=[ "packageConfGroup" ];
            attrPath=[ "pkg" ];
            strAppend="/lib/pkgconfig";
            strJoin=":";
        });
        manualPythonPackages = (aggregation.getAll {
            hasAllFlags=[ "manualPythonPackages" ];
            attrPath=[ "pkg" ];
            strAppend="/lib/python3.${aggregation.mergedVals.pythonMinorVersion}/site-packages";
            strJoin=":";
        });
        groups = {
            inherit ldLibraryPackages giTypelibPackagesString packageConfPackagesString manualPythonPackages;
        };
    
        # ------------------------------------------------------------
        # 3. Host interactive shell  →  `nix develop`
        # ------------------------------------------------------------
        envVarsShellHook = ''
          shopt -s nullglob 2>/dev/null || setopt +o nomatch 2>/dev/null || true # allow globs to be empty without throwing an error
          if [ "$OSTYPE" = "linux-gnu" ]; then
            export CC="cc-no-usr-include" # basically patching for nix
            # Create nvidia-only lib symlinks to avoid glibc conflicts
            NVIDIA_LIBS_DIR="/tmp/nix-nvidia-libs-$$"
            mkdir -p "$NVIDIA_LIBS_DIR"
            for lib in /usr/lib/libcuda.so* /usr/lib/libnvidia*.so* /usr/lib/x86_64-linux-gnu/libnvidia*.so*; do
              [ -e "$lib" ] && ln -sf "$lib" "$NVIDIA_LIBS_DIR/" 2>/dev/null
            done
          fi
          export LD_LIBRARY_PATH="$NVIDIA_LIBS_DIR:${pkgs.lib.makeLibraryPath ldLibraryPackages}:$LD_LIBRARY_PATH"
          export LIBRARY_PATH="$LD_LIBRARY_PATH" # fixes python find_library for pyaudio
          export DISPLAY=:0
          export GI_TYPELIB_PATH="${giTypelibPackagesString}:$GI_TYPELIB_PATH"
          export PKG_CONFIG_PATH=${lib.escapeShellArg packageConfPackagesString}
          export PYTHONPATH="$PYTHONPATH:"${lib.escapeShellArg manualPythonPackages}
          # CC, CFLAGS, and LDFLAGS are bascially all for `pip install pyaudio`
          export CFLAGS="$(pkg-config --cflags portaudio-2.0) $CFLAGS"
          export LDFLAGS="-L$(pkg-config --variable=libdir portaudio-2.0) $LDFLAGS"
        '';
        shellHook = ''
          ${envVarsShellHook}

          # without this alias, the pytest uses the non-venv python and fails
          alias pytest="python -m pytest"

          PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo "$PWD")
          [ -f "$PROJECT_ROOT/motd" ] && cat "$PROJECT_ROOT/motd"
          [ -f "$PROJECT_ROOT/.pre-commit-config.yaml" ] && pre-commit install --install-hooks
          if [ -f "$PROJECT_ROOT/env/bin/activate" ]; then
            . "$PROJECT_ROOT/env/bin/activate"
          fi
          cd "$PROJECT_ROOT"

          #
          # python & setup
          #
          if [ -f "$PROJECT_ROOT/venv/bin/activate" ]; then
            # if there is a venv, load it
            _nix_python_path="$(realpath "$(which python)")"
            . "$PROJECT_ROOT/venv/bin/activate"
            # check the venv to make sure it wasn't created with a different (non nix) python
            if [ "$_nix_python_path" != "$(realpath "$(which python)")" ]
            then
              echo
              echo
              echo "WARNING:"
              echo "     Your venv was created with something other than the current nix python"
              echo "     This could happen if you made the venv before doing `nix develop`"
              echo "     It could also happen if the nix-python was updated but the venv wasn't"
              echo "     WHAT YOU NEED TO DO:"
              echo "     - If you're about to make/test a PR, delete/rename your venv and run `nix develop` again"
              echo "     - If you're just trying to get the code working, you can continue but you might get bugs FYI"
              echo
              echo
              echo "Got it? (press enter)"; read _
              echo
            fi
          else
            #
            # automate the readme
            #
            cyan="$(printf '%b' "\e[0;36m")"
            color_reset="$(printf '%b' "\e[0m")"
            echo
            echo "I don't see a venv directory"
            echo "If you'd like me to setup the project for you, run: $cyan bin/_dev_init $color_reset"
          fi
        '';
        devShells = {
          # basic shell (blends with your current environment)
          default = pkgs.mkShell {
            buildInputs = devPackages;
            shellHook = shellHook;
          };
          # strict shell (creates a fake home, only select exteral commands (e.g. sudo) from your system are available)
          isolated = (xome.simpleMakeHomeFor {
            inherit pkgs;
            pure = true;
            commandPassthrough = [ "sudo" "nvim" "code" "sysctl" "sw_vers" "git" "vim" "emacs" "openssl" "ssh" "osascript" "otool" "hidutil" "logger" "codesign" ]; # e.g. use external nvim instead of nix's
            # commonly needed for MacOS: [ "osascript" "otool" "hidutil" "logger" "codesign" ]
            homeSubpathPassthrough = [ "cache/nix/" ]; # share nix cache between projects
            homeModule = {
              # for home-manager examples, see:
              # https://deepwiki.com/nix-community/home-manager/5-configuration-examples
              # all home-manager options:
              # https://nix-community.github.io/home-manager/options.xhtml
              home.homeDirectory = "/tmp/virtual_homes/dimos";
              home.stateVersion = "25.11";
              home.packages = devPackages;

              programs = {
                home-manager = {
                  enable = true;
                };
                zsh = {
                  enable = true;
                  enableCompletion = true;
                  autosuggestion.enable = true;
                  syntaxHighlighting.enable = true;
                  shellAliases.ll = "ls -la";
                  history.size = 100000;
                  # this is kinda like .zshrc
                  initContent = ''
                    # most people expect comments in their shell to to work
                    setopt interactivecomments
                    # fix emoji prompt offset issues (this shouldn't lock people into English b/c LANG can be non-english)
                    export LC_CTYPE=en_US.UTF-8
                    ${shellHook}
                  '';
                };
                starship = {
                  enable = true;
                  enableZshIntegration = true;
                  settings = {
                    character = {
                      success_symbol = "[▣](bold green)";
                      error_symbol = "[▣](bold red)";
                    };
                  };
                };
              };
            };
          }).default;
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
        # for re-use in other flakes
        vars = {
            inherit devPackages groups aggregation lib;
            # what other flakes will use
            shellHook = envVarsShellHook;
            # what someone would use if they weren't really managing their own project
            fullShell = shellHook;
        };
        ## Local dev shell
        devShells = devShells;

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
