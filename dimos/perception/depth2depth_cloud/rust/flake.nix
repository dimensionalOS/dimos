{
  description = "depth2depth_cloud native module for DimOS: the depth2depth crate behind an LCM wrapper";

  inputs = {
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
  outputs = { self, nixpkgs, flake-utils, crate2nix, depth2depth }:
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

        src = pkgs.runCommand "depth2depth-cloud-src" {} ''
          mkdir -p $out/dimos/perception/depth2depth_cloud/rust
          cp -r ${./src} $out/dimos/perception/depth2depth_cloud/rust/src
          cp ${./Cargo.toml} $out/dimos/perception/depth2depth_cloud/rust/Cargo.toml
          cp ${./Cargo.lock} $out/dimos/perception/depth2depth_cloud/rust/Cargo.lock

          mkdir -p $out/native/rust
          cp -r ${../../../../native/rust/dimos-module} $out/native/rust/dimos-module
          cp -r ${../../../../native/rust/dimos-module-macros} $out/native/rust/dimos-module-macros
        '';

        generatedCargoNix = crate2nix.tools.${system}.generatedCargoNix {
          name = "depth2depth-cloud";
          inherit src;
          cargoToml = "dimos/perception/depth2depth_cloud/rust/Cargo.toml";
        };

        build = features: (import generatedCargoNix {
          inherit pkgs;
          rootFeatures = [ "default" ] ++ features;
          buildRustCrateForPkgs = cratePkgs: cratePkgs.buildRustCrate.override {
            defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
              # Builds libjpeg-turbo from source (the `cmake` feature).
              turbojpeg-sys = attrs: { nativeBuildInputs = (attrs.nativeBuildInputs or []) ++ [ pkgs.cmake pkgs.nasm ]; };
              # TensorRT references the driver's libraries (libcuda, and on a Jetson libnvdla_compiler), which the
              # sandbox lacks; they resolve at runtime from the host (see the wrapper below).
              dimos-depth2depth-cloud = attrs: pkgs.lib.optionalAttrs (builtins.elem "tensorrt" (attrs.features or [])) {
                extraRustcOpts = (attrs.extraRustcOpts or []) ++ [ "-C" "link-arg=-Wl,--allow-shlib-undefined" ];
              };
              # The model it embeds, and for TensorRT CUDA + TensorRT.
              depth2depth = depth2depth.lib.crateOverride { inherit pkgs cudaPackages; };
            };
          };
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
          default = build [ ];
        } // pkgs.lib.optionalAttrs pkgs.stdenv.isLinux {
          tensorrt = withHostDriver (build [ "tensorrt" ]);
        };
      });
}
