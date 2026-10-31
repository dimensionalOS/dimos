{
  description = "GPU-resident voxel map and MLS surface extraction for dimos (wgpu compute)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # A rustc with the Linux standard libraries, for the zig cross builds.
    rust-overlay.url = "github:oxalica/rust-overlay";
    rust-overlay.inputs.nixpkgs.follows = "nixpkgs";
  };

  # packages.default: the tests and bench, built and run with nix. On Linux they are wrapped to reach the
  # host's NVIDIA Vulkan driver (see hostGpuEnv). packages.cross-aarch64-linux / cross-x86_64-linux, from
  # any of the systems: the same binaries linked against glibc 2.35 with zig, for copying to a Jetson
  # (JetPack 6) or a PC, where they use the host's Vulkan directly.
  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ] (system:
      let
        pkgs = import nixpkgs { inherit system; overlays = [ rust-overlay.overlays.default ]; };
        lib = pkgs.lib;
        crateDir = "dimos/mapping/gpu_mls/rust";
        # The crate plus the two CPU crates its parity tests compare against.
        src = lib.fileset.toSource {
          root = ../../../..;
          fileset = lib.fileset.unions [
            ./Cargo.toml
            ./Cargo.lock
            ./src
            ./tests
            ./examples
            ./host_gpu_env.nix
            ../../ray_tracing/rust/Cargo.toml
            ../../ray_tracing/rust/src
            ../../../navigation/global_planner/mls_planner/rust/Cargo.toml
            ../../../navigation/global_planner/mls_planner/rust/src
          ];
        };
        cargoDeps = pkgs.rustPlatform.importCargoLock {
          lockFile = ./Cargo.lock;
          allowBuiltinFetchGit = true;
        };

        hostGpuEnv = import ./host_gpu_env.nix { inherit pkgs lib; };

        # Copy every test and example executable cargo reported out of build.json, and write runners.
        # $1 is the runners' shell, $2 an environment prelude.
        installRunners = shell: prelude: ''
          mkdir -p $out/libexec/tests $out/libexec/examples $out/bin
          jq -r 'select(.reason == "compiler-artifact" and .executable != null and .profile.test) | .executable' build.json \
            | while read -r exe; do cp "$exe" $out/libexec/tests/; done
          jq -r 'select(.reason == "compiler-artifact" and .executable != null and (.target.kind | index("example"))) | .executable' build.json \
            | while read -r exe; do cp "$exe" $out/libexec/examples/; done
          cat > $out/bin/gpu-mls-tests <<'EOF'
          #!${shell}
          ${prelude}
          here=$(dirname "$0")/../libexec/tests
          status=0
          for test in "$here"/*; do "$test" "$@" || status=1; done
          exit $status
          EOF
          cat > $out/bin/gpu-mls-bench <<'EOF'
          #!${shell}
          ${prelude}
          exec "$(dirname "$0")/../libexec/examples/bench" "$@"
          EOF
          chmod +x $out/bin/*
        '';

        native = pkgs.stdenv.mkDerivation {
          pname = "dimos-gpu-mls";
          version = "0.1.0";
          inherit src cargoDeps;
          cargoRoot = crateDir;
          nativeBuildInputs = [ pkgs.rustPlatform.cargoSetupHook pkgs.cargo pkgs.rustc pkgs.jq ];
          buildInputs = lib.optionals pkgs.stdenv.hostPlatform.isDarwin [ pkgs.libiconv ];
          buildPhase = ''
            cd ${crateDir}
            cargo build --release --frozen --tests --examples --message-format=json-render-diagnostics > build.json
          '';
          installPhase = installRunners pkgs.runtimeShell (lib.optionalString pkgs.stdenv.hostPlatform.isLinux hostGpuEnv);
        };

        crossToolchain = pkgs.rust-bin.stable.latest.minimal.override {
          targets = [ "aarch64-unknown-linux-gnu" "x86_64-unknown-linux-gnu" ];
        };
        # glibc 2.35 is JetPack 6's (Ubuntu 22.04); newer hosts run it too.
        cross = target: pkgs.stdenv.mkDerivation {
          pname = "dimos-gpu-mls-${target}";
          version = "0.1.0";
          inherit src cargoDeps;
          cargoRoot = crateDir;
          nativeBuildInputs = [ pkgs.rustPlatform.cargoSetupHook crossToolchain pkgs.cargo-zigbuild pkgs.zig pkgs.jq ];
          buildInputs = lib.optionals pkgs.stdenv.hostPlatform.isDarwin [ pkgs.libiconv ];
          dontFixup = true;
          buildPhase = ''
            export HOME=$TMPDIR ZIG_GLOBAL_CACHE_DIR=$TMPDIR/zig ZIG_LOCAL_CACHE_DIR=$TMPDIR/zig
            cd ${crateDir}
            cargo zigbuild --release --frozen --target ${target}.2.35 --tests --examples \
              --message-format=json-render-diagnostics > build.json
          '';
          # Plain /bin/sh runners: the target need not have nix.
          installPhase = installRunners "/bin/sh" "";
        };
      in {
        packages = {
          default = native;
          cross-aarch64-linux = cross "aarch64-unknown-linux-gnu";
          cross-x86_64-linux = cross "x86_64-unknown-linux-gnu";
        };
        # cargo test / run from this shell reach the host GPU the same way.
        devShells.default = pkgs.mkShell {
          packages = [ pkgs.cargo pkgs.rustc pkgs.rustfmt pkgs.clippy ] ++ lib.optionals pkgs.stdenv.hostPlatform.isDarwin [ pkgs.libiconv ];
          shellHook = lib.optionalString pkgs.stdenv.hostPlatform.isLinux hostGpuEnv;
        };
      });
}
