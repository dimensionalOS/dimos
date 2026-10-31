{
  description = "Voxel ray tracing native module for dimos";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
    # The GPU map crate it builds on (a path dependency in Cargo.toml), pinned like any shared code.
    dimos-gpu-mls.url = "github:dimensionalOS/dimos?dir=dimos/mapping/gpu_mls/rust";
    dimos-gpu-mls.inputs.nixpkgs.follows = "nixpkgs";
    dimos-gpu-mls.inputs.flake-utils.follows = "flake-utils";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix, dimos-gpu-mls }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "dimos-voxel-ray-tracing";

        lib = pkgs.lib;
        here = "mapping/ray_tracing/rust";

        own = nix-filter.lib { root = ./.; exclude = [ "target" "build" "result" "__pycache__" ]; };
        # The two crates at their repo-relative places, so the path dependency resolves.
        src = pkgs.runCommand "${name}-src" { } ''
          mkdir -p $out/${here} $out/mapping/gpu_mls
          cp -r ${own}/. $out/${here}
          cp -r ${dimos-gpu-mls} $out/mapping/gpu_mls/rust
        '';

        generated = crate2nix.tools.${system}.generatedCargoNix {
          inherit name src;
          cargoToml = "${here}/Cargo.toml";
          additionalCrateHashes = builtins.fromJSON (builtins.readFile ./crate-hashes.json);
        };

        ours = [ name "dimos-gpu-mls" "dimos-module" "dimos-module-macros" ];
        callWith = mode: import generated {
          inherit pkgs;
          buildRustCrateForPkgs = cratePkgs: crate:
            # stabby-macros writes the builder's core count (NUM_JOBS) into its code; one value makes every machine's copy match.
            (cratePkgs.buildRustCrate.override {
              defaultCrateOverrides = cratePkgs.defaultCrateOverrides // { stabby-macros = _: { preConfigure = "export NIX_BUILD_CORES=1"; }; };
            }) (crate // pkgs.lib.optionalAttrs
              (mode != null && builtins.elem crate.crateName ours)
              ({
                release = false;
                extraRustcOpts = (crate.extraRustcOpts or [ ]) ++ [ "-C" "debuginfo=0" ];
              } // pkgs.lib.optionalAttrs (mode == "lint") {
                useClippy = true;
                capLints = "forbid";
                extraRustcOpts =
                  (crate.extraRustcOpts or [ ]) ++ [ "-D" "warnings" "-C" "debuginfo=0" ];
              }));
        };
        buildOf = called:
          if called ? rootCrate then called.rootCrate.build
          else called.workspaceMembers.${name}.build;

        rustTools = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ];

        built = buildOf (callWith null);
        # On Linux the binary reaches the host's NVIDIA Vulkan driver the way gpu_mls's do.
        hostGpuEnv = import "${dimos-gpu-mls}/host_gpu_env.nix" { inherit pkgs lib; };
        wrapped = pkgs.runCommand name { } ''
          mkdir -p $out/bin
          ln -s ${pkgs.writeShellScript "voxel_ray_tracing" ''
            ${hostGpuEnv}
            exec ${built}/bin/voxel_ray_tracing "$@"
          ''} $out/bin/voxel_ray_tracing
        '';
      in {
        packages.default = if pkgs.stdenv.hostPlatform.isLinux then wrapped else built;
        packages.${name} = self.packages.${system}.default;

        packages.lint = (buildOf (callWith "lint")).override {
          runTests = true;
          testCrateFlags = [ "--list" ];
        };
        checks.lint = self.packages.${system}.lint;

        packages.tests = (buildOf (callWith "test")).override { runTests = true; };
        checks.tests = self.packages.${system}.tests;

        devShells.default = pkgs.mkShell { packages = rustTools; };
      });
}
