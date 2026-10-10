{
  description = "V4L2 camera native module for dimos";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  # packages.default: software JPEG. packages.jetson (aarch64-linux): the hardware JPEG path, built against NVIDIA's
  # JetPack 6 (L4T r36.4.7) multimedia packages and run against the Jetson's own copies of those libraries.
  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "dimos-v4l2-camera";

        src = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        nvidiaDeb = repo: pname: hash: pkgs.fetchurl {
          url = "https://repo.download.nvidia.com/jetson/${repo}/pool/main/n/${pname}/${pname}_36.4.7-20250918154033_arm64.deb";
          inherit hash;
        };
        # Headers and the two libraries capture.c links; libnvjpeg itself is dlopen'd from the Jetson at runtime.
        jetsonMultimedia = pkgs.runCommand "jetson-multimedia-r36.4.7" { nativeBuildInputs = [ pkgs.dpkg ]; } ''
          for deb in ${nvidiaDeb "common" "nvidia-l4t-jetson-multimedia-api" "sha256-/z5jNgbhBVuE2fHlSdXBjsEoXqJhWxPA1bQK414Eu6g="} \
                     ${nvidiaDeb "t234" "nvidia-l4t-multimedia-utils" "sha256-HxTO82T4vzDa9LxIH8K3UhUsuZYZlX8nZAARKUbVelk="} \
                     ${nvidiaDeb "t234" "nvidia-l4t-multimedia" "sha256-Kt19t3cmVWRMZIIgH5hR2kC00emINZVGolb0mb0r89Y="}; do
            dpkg-deb -x $deb $out
          done
        '';

        ours = [ name "dimos-module" "dimos-module-macros" ];
        callWith = jetson: mode: import generated {
          inherit pkgs;
          buildRustCrateForPkgs = cratePkgs: crate:
            (cratePkgs.buildRustCrate.override {
              defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
                # stabby-macros writes the builder's core count (NUM_JOBS) into its code; one value makes every machine's copy match.
                stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; };
                # Builds libjpeg-turbo from source (the `cmake` feature).
                turbojpeg-sys = attrs: { nativeBuildInputs = (attrs.nativeBuildInputs or [ ]) ++ [ pkgs.cmake pkgs.nasm ]; };
                ${name} = attrs: pkgs.lib.optionalAttrs jetson {
                  preConfigure = ''
                    export JETSON_MULTIMEDIA_API=${jetsonMultimedia}/usr/src/jetson_multimedia_api/include
                    export JETSON_LIBS=${jetsonMultimedia}/usr/lib/aarch64-linux-gnu/nvidia
                  '';
                  # Those libraries reference the rest of the driver, which only the Jetson has.
                  extraRustcOpts = (attrs.extraRustcOpts or [ ]) ++ [ "-C" "link-arg=-Wl,--allow-shlib-undefined" ];
                };
              };
            }) (crate // pkgs.lib.optionalAttrs (mode != null && builtins.elem crate.crateName ours) ({
              release = false;
              extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-C" "debuginfo=0" ];
            } // pkgs.lib.optionalAttrs (mode == "lint") {
              useClippy = true;
              capLints = "forbid";
              extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-D" "warnings" "-C" "debuginfo=0" ];
            }));
        };
        buildOf = called:
          if called ? rootCrate then called.rootCrate.build
          else called.workspaceMembers.${name}.build;

        # nix's glibc doesn't read ld.so.cache, so point it at JetPack's driver directories.
        withJetPack = unwrapped: pkgs.writeShellScriptBin "v4l2_camera" ''
          export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          exec ${unwrapped}/bin/v4l2_camera "$@"
        '';
      in {
        packages = {
          default = buildOf (callWith false null);
          lint = (buildOf (callWith false "lint")).override { runTests = true; testCrateFlags = [ "--list" ]; };
          tests = (buildOf (callWith false "test")).override { runTests = true; };
        } // pkgs.lib.optionalAttrs (system == "aarch64-linux") {
          jetson = withJetPack (buildOf (callWith true null));
        };
        checks.lint = self.packages.${system}.lint;
        checks.tests = self.packages.${system}.tests;

        devShells.default = pkgs.mkShell { packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ]; };
      });
}
