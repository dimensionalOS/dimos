{
  description = "YOUR_DIMOS_PROJECT_DESCRIPTION";
  
  inputs = {
    nixpkgs.url      = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url  = "github:numtide/flake-utils";
    dimos-flake.url  = "github:jeff-hykin/mystery_test_1/59acdf244148b879230aa1dd0dcb1e1191a50b72";
    dimos-flake.inputs.nixpkgs.follows     = "nixpkgs";
    dimos-flake.inputs.flake-utils.follows = "flake-utils";
    xome.url         = "github:jeff-hykin/xome";
    xome.inputs.nixpkgs.follows     = "nixpkgs";
    xome.inputs.flake-utils.follows = "flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, dimos-flake, xome, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        dimos = dimos-flake.vars.${system};

        # ------------------------------------------------------------
        # packages
        # ------------------------------------------------------------
        devPackages = dimos.devPackages ++ [
            # add your nix packages here!
            # ex: pkgs.cowsay
        ];

        # ------------------------------------------------------------
        # shell setup (bashrc like)
        # ------------------------------------------------------------
        shellHook = ''
          ${dimos.shellHook}

          PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo "$PWD")
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
            # automate venv setup
            #
            echo "Setting up virtualenv..."
            python3 -m venv venv
            echo "Activating virtualenv..."
            . venv/bin/activate
            # if uv doesnt exist
            if [ -z "$(command -v "uv")" ]; then
                pip install uv
            fi
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
              home.homeDirectory = "/tmp/virtual_homes/YOUR_DIMOS_PROJECT_NAME";
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
                      success_symbol = "[∫](bold green)";
                      error_symbol = "[∫](bold red)";
                    };
                  };
                };
              };
            };
          }).default;
        };
      in {
        ## Local dev shell
        devShells = devShells;
      });
}
