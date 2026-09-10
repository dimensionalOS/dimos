{
  description = "Dev shell for the hyperspace native module: rust toolchain, and CUDA when the machine has it";

  # Deliberately a dev shell only, no package. The crate is a member of the
  # repo-root cargo workspace (PR #3666 moved these crates there), so its
  # build_command stays `cargo build --release` against the root workspace and
  # nothing here needs to snapshot the repo into the nix store. This flake
  # exists because hyperspace needs things the other members do not: candle, a
  # CUDA toolchain for the `cuda` feature, and a python for the CLI.

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    # Not eachDefaultSystem: nixpkgs 26.11 dropped x86_64-darwin, and naming it
    # is an eval error.
    flake-utils.lib.eachSystem [ "aarch64-darwin" "aarch64-linux" "x86_64-linux" ] (system:
      let
        isDarwin = nixpkgs.lib.hasSuffix "-darwin" system;
        pkgs = import nixpkgs {
          inherit system;
          config = { allowUnfree = true; cudaSupport = !isDarwin; };
        };

        # The `cuda` cargo feature needs nvcc and the cuBLAS/cuDNN libraries that
        # candle links. Linux only: candle falls back to the CPU elsewhere, and
        # asking for cudaPackages on darwin fails evaluation.
        cudaPackages = nixpkgs.lib.optionals (!isDarwin) (with pkgs.cudaPackages; [
          cuda_nvcc
          cuda_cudart
          libcublas
          cudnn
        ]);
      in
      {
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            cargo
            rustc
            rustfmt
            clippy
            rust-analyzer
            pkg-config
            openssl
            # The CLI writes the rrd, and the model conversion scripts are python.
            python3
          ] ++ cudaPackages;

          shellHook = nixpkgs.lib.optionalString (!isDarwin) ''
            export CUDA_ROOT="${pkgs.cudaPackages.cuda_nvcc}"
            export CUDA_COMPUTE_CAP="''${CUDA_COMPUTE_CAP:-89}"
          '' + ''
            echo "hyperspace dev shell. Build from the repo root workspace:"
            echo "  cargo build --release -p dimos-hyperspace"
            echo "Real models need the features, e.g.:"
            echo "  cargo build --release -p dimos-hyperspace --features cuda"
          '';
        };
      }
    );
}
