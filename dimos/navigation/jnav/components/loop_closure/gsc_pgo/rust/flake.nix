{
  # Build/test workflow (this crate lives inside a larger git repo, so use
  # the path: ref to keep nix from copying the whole repo to the store):
  #
  #   cd dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust
  #   nix develop path:. --command cargo test
  #
  # The dev shell exports GTSAM_INCLUDE_DIR / GTSAM_LIB_DIR /
  # EIGEN_INCLUDE_DIR / BOOST_INCLUDE_DIR, which build.rs consumes directly.
  description = "dimos-gsc-pgo: Rust port of the gsc_pgo PGO core (gtsam FFI shim + Scan Context)";

  # Pins mirror ~/repos/gsc_pgo's flake.lock (nixpkgs 549bd84, gtsam-extended
  # f4572a8, gtsam develop 1a9792a) so the gtsam derivation is byte-identical
  # to the C++ module's — same store path, no rebuild.
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/549bd84d6279f9852cae6225e372cc67fb91a4c1";
    flake-utils.url = "github:numtide/flake-utils/11707dc2f618dd54ca8739b309ec4fc024de578b";
    gtsam-extended = {
      url = "github:jeff-hykin/gtsam-extended/f4572a80b6339181693aee6029ca28153e59a993";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, gtsam-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        gtsam-base = gtsam-extended.packages.${system}.gtsam-cpp;
        # Same source override as gsc_pgo's flake: gtsam develop @ 1a9792a.
        gtsam = gtsam-base.overrideAttrs (_old: {
          src = pkgs.fetchFromGitHub {
            owner = "borglab";
            repo = "gtsam";
            rev = "1a9792a7ede244850a413739557635b606f295c0";
            sha256 = "sha256-zxm5TGVPW1vipFVpw01zcvKRw4mkh+5ZBCR1n6G466o=";
          };
          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";
        });

        # The env-var contract build.rs consumes (stage 1 of its discovery).
        # tbb is part of gtsam's public headers (GTSAM_USE_TBB build).
        buildEnv = {
          GTSAM_INCLUDE_DIR = "${gtsam}/include";
          GTSAM_LIB_DIR = "${gtsam}/lib";
          EIGEN_INCLUDE_DIR = "${pkgs.eigen}/include/eigen3";
          BOOST_INCLUDE_DIR = "${pkgs.boost.dev}/include";
          TBB_INCLUDE_DIR = "${pkgs.lib.getDev pkgs.tbb}/include";
          TBB_LIB_DIR = "${pkgs.lib.getLib pkgs.tbb}/lib";
        };

      in {
        devShells.default = pkgs.mkShell {
          # clippy + rustfmt come from the same nixpkgs pin as cargo/rustc so the
          # `cargo fmt` / `cargo clippy` subcommands resolve to a matching toolchain
          # (CI runs them inside this shell; no rustup is present there).
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt pkgs.pkg-config ];
          buildInputs = [ gtsam pkgs.eigen pkgs.boost pkgs.tbb ];
          env = buildEnv;
        };

        # NOTE: there is deliberately no packages.default anymore. The crate's
        # module binary depends on the in-repo `dimos-module` crate by path
        # (../../../../../../../native/rust/dimos-module), which a store-copied
        # crate source cannot resolve, so `nix build` of this crate alone is
        # impossible. Build via the dev shell instead (what module.py's
        # build_command does):
        #
        #   nix develop path:. --command cargo build --release
      });
}
