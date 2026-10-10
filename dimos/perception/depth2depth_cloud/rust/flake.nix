{
  description = "depth2depth_cloud native module for DimOS: the depth2depth crate behind an LCM wrapper";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
    # The tag of the depth2depth release Cargo.toml uses: its flake fetches the model that release embeds (pinned in its model.json).
    depth2depth.url = "github:jeff-hykin/depth2depth/v0.4.0";
    depth2depth.inputs.nixpkgs.follows = "nixpkgs";
  };

  # packages.default: Metal on a Mac, CPU elsewhere (e.g. a Pi). packages.tensorrt (Linux): TensorRT on an NVIDIA GPU,
  # a Jetson (JetPack 6, CUDA 12.6) on aarch64 or a PC (CUDA 12.8, through the RTX 50-series) on x86_64.
  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix, depth2depth }:
    flake-utils.lib.eachSystem [ "aarch64-darwin" "aarch64-linux" "x86_64-linux" ] (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          # TensorRT from nixpkgs (the build sandbox can't see the host's). Its CVE flag is about
          # malicious engine files; this one only loads engines it built itself or the crate pins.
          config = nixpkgs.lib.optionalAttrs (nixpkgs.lib.hasSuffix "-linux" system) {
            allowUnfree = true;
            allowInsecurePredicate = pkg: nixpkgs.lib.hasInfix "tensorrt" (pkg.name or "");
          } // nixpkgs.lib.optionalAttrs (system == "aarch64-linux") {
            # Orin; it also makes nixpkgs pick JetPack's CUDA and TensorRT builds over the server-ARM ones.
            cudaCapabilities = [ "8.7" ];
          };
        };
        cudaPackages = if system == "aarch64-linux" then pkgs.cudaPackages_12_6 else pkgs.cudaPackages_12_8;

        name = "dimos-depth2depth-cloud";
        src = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        ours = [ name "dimos-module" "dimos-module-macros" ];
        build = mode: features: (import generated {
          inherit pkgs;
          rootFeatures = [ "default" ] ++ features;
          buildRustCrateForPkgs = cratePkgs: crate: (cratePkgs.buildRustCrate.override {
            defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
              # stabby-macros writes the builder's core count (NUM_JOBS) into its code; one value makes every machine's copy match.
              stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; };
              # Builds libjpeg-turbo from source (the `cmake` feature).
              turbojpeg-sys = attrs: { nativeBuildInputs = (attrs.nativeBuildInputs or []) ++ [ pkgs.cmake pkgs.nasm ]; };
              # TensorRT references the driver's libraries (libcuda, and on a Jetson libnvdla_compiler), which the
              # sandbox lacks; they resolve at runtime from the host (see the wrapper below).
              ${name} = attrs: pkgs.lib.optionalAttrs (builtins.elem "tensorrt" (attrs.features or [])) {
                extraRustcOpts = (attrs.extraRustcOpts or []) ++ [ "-C" "link-arg=-Wl,--allow-shlib-undefined" ];
              };
              # The model it embeds, and for TensorRT CUDA + TensorRT.
              depth2depth = depth2depth.lib.crateOverride { inherit pkgs cudaPackages; };
            };
          }) (crate // pkgs.lib.optionalAttrs (mode != null && builtins.elem crate.crateName ours) ({
            release = false;
            extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-C" "debuginfo=0" ];
          } // pkgs.lib.optionalAttrs (mode == "lint") {
            useClippy = true;
            capLints = "forbid";
            extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-D" "warnings" "-C" "debuginfo=0" ];
          }));
        }).rootCrate.build;

        # nix's glibc doesn't read ld.so.cache, so hand it the host's NVIDIA driver: JetPack's directories on a
        # Jetson, else symlinks to just the driver's libraries (a whole /usr/lib would shadow nix's own).
        withHostDriver = unwrapped: pkgs.writeShellScriptBin "depth2depth_cloud" ''
          driver=''${XDG_CACHE_HOME:-$HOME/.cache}/depth2depth/host-driver
          mkdir -p "$driver"
          for lib in /usr/lib/x86_64-linux-gnu/lib{cuda,nvidia-}*.so* /run/opengl-driver/lib/lib{cuda,nvidia-}*.so*; do
            [ -e "$lib" ] && ln -sf "$lib" "$driver/"
          done
          export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:$driver''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          exec ${unwrapped}/bin/depth2depth_cloud "$@"
        '';
      in {
        packages = {
          default = build null [ ];
          lint = (build "lint" [ ]).override { runTests = true; testCrateFlags = [ "--list" ]; };
          tests = (build "test" [ ]).override { runTests = true; };
        } // pkgs.lib.optionalAttrs pkgs.stdenv.isLinux {
          tensorrt = withHostDriver (build null [ "tensorrt" ]);
        };
        checks.lint = self.packages.${system}.lint;
        checks.tests = self.packages.${system}.tests;
      });
}
