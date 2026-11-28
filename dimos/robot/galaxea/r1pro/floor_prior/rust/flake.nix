{
  description = "r1pro_head_depth native module for DimOS: depth2depth_cloud with the R1 Pro's floor prior";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
    # The tag of the depth2depth release depth2depth_cloud uses: its flake fetches the model that release embeds.
    depth2depth.url = "github:jeff-hykin/depth2depth/v0.4.1";
    depth2depth.inputs.nixpkgs.follows = "nixpkgs";
    # The depth2depth_cloud crate it builds on (a path dependency in Cargo.toml), pinned like any shared code.
    dimos-depth2depth-cloud.url = "github:dimensionalOS/dimos?dir=dimos/perception/depth2depth_cloud/rust";
    dimos-depth2depth-cloud.inputs.nixpkgs.follows = "nixpkgs";
    dimos-depth2depth-cloud.inputs.nix-filter.follows = "nix-filter";
    dimos-depth2depth-cloud.inputs.flake-utils.follows = "flake-utils";
    dimos-depth2depth-cloud.inputs.crate2nix.follows = "crate2nix";
    dimos-depth2depth-cloud.inputs.depth2depth.follows = "depth2depth";
  };

  # packages.default: Metal on a Mac, CPU elsewhere. packages.tensorrt (Linux): TensorRT on an NVIDIA GPU, as depth2depth_cloud.
  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix, depth2depth, dimos-depth2depth-cloud }:
    flake-utils.lib.eachSystem [ "aarch64-darwin" "aarch64-linux" "x86_64-linux" ] (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config = nixpkgs.lib.optionalAttrs (nixpkgs.lib.hasSuffix "-linux" system) {
            allowUnfree = true;
            allowInsecurePredicate = pkg: nixpkgs.lib.hasInfix "tensorrt" (pkg.name or "");
          } // nixpkgs.lib.optionalAttrs (system == "aarch64-linux") {
            cudaCapabilities = [ "8.7" ];
          };
        };
        cudaPackages = if system == "aarch64-linux" then pkgs.cudaPackages_12_6 else pkgs.cudaPackages_12_8;

        name = "dimos-r1pro-floor-prior";
        here = "robot/galaxea/r1pro/floor_prior/rust";
        own = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        # The two crates at their repo-relative places, so the path dependency resolves.
        src = pkgs.runCommand "${name}-src" { } ''
          mkdir -p $out/${here} $out/perception/depth2depth_cloud
          cp -r ${own}/. $out/${here}
          cp -r ${dimos-depth2depth-cloud} $out/perception/depth2depth_cloud/rust
        '';
        generated = crate2nix.tools.${system}.generatedCargoNix {
          inherit name src;
          cargoToml = "${here}/Cargo.toml";
        };

        ours = [ name "dimos-depth2depth-cloud" "dimos-module" "dimos-module-macros" ];
        build = mode: features: (import generated {
          inherit pkgs;
          rootFeatures = [ "default" ] ++ features;
          buildRustCrateForPkgs = cratePkgs: crate: (cratePkgs.buildRustCrate.override {
            defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
              stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; };
              turbojpeg-sys = attrs: { nativeBuildInputs = (attrs.nativeBuildInputs or []) ++ [ pkgs.cmake pkgs.nasm ]; };
              # TensorRT's driver libraries resolve at runtime from the host (see the wrapper below).
              ${name} = attrs: pkgs.lib.optionalAttrs (builtins.elem "tensorrt" (attrs.features or [])) {
                extraRustcOpts = (attrs.extraRustcOpts or []) ++ [ "-C" "link-arg=-Wl,--allow-shlib-undefined" ];
              };
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

        # As depth2depth_cloud's: nix's glibc doesn't read ld.so.cache, so hand it the host's NVIDIA driver.
        withHostDriver = unwrapped: pkgs.writeShellScriptBin "r1pro_head_depth" ''
          driver=''${XDG_CACHE_HOME:-$HOME/.cache}/depth2depth/host-driver
          mkdir -p "$driver"
          for lib in /usr/lib/x86_64-linux-gnu/lib{cuda,nvidia-}*.so* /run/opengl-driver/lib/lib{cuda,nvidia-}*.so*; do
            [ -e "$lib" ] && ln -sf "$lib" "$driver/"
          done
          export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:$driver''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          exec ${unwrapped}/bin/r1pro_head_depth "$@"
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
