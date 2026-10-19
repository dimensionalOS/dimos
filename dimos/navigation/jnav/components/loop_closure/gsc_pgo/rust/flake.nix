{
  # Build/test workflow (this crate lives inside a larger git repo, so use
  # the path: ref to keep nix from copying the whole repo to the store):
  #
  #   cd dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust
  #   nix develop path:. --command cargo test
  #   nix build path:.#checks.<system>.lint    # clippy, warnings as errors
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
    crate2nix = {
      url = "github:nix-community/crate2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, gtsam-extended, crate2nix, ... }:
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

        src = pkgs.lib.cleanSourceWith {
          src = ./.;
          filter = path: _type: !(builtins.elem (baseNameOf path) [ "target" "result" ]);
        };

        # crate2nix turns Cargo.lock into one derivation per crate, so there is
        # no vendor-tarball `cargoHash` to regenerate whenever a dependency
        # moves. `crate-hashes.json` pins the git dependencies (lcm-msgs, dimos-module);
        # refresh it with `crate2nix generate` when that ref changes.
        generated = crate2nix.tools.${system}.generatedCargoNix {
          name = "dimos-gsc-pgo";
          inherit src;
        };
        crateOverrides = pkgs.defaultCrateOverrides // {
          dimos-gsc-pgo = _attrs: {
            nativeBuildInputs = [ pkgs.pkg-config ];
            buildInputs = [ gtsam pkgs.eigen pkgs.boost pkgs.tbb ];
          } // buildEnv;
        };
        # mode null: the release module; "lint": clippy with warnings as errors; "test": a debug build
        # that runs the tests. Only our own crates change; dependencies build as for the module.
        ours = [ "dimos-gsc-pgo" "dimos-module" "dimos-module-macros" ];
        cargoNixWith = mode: pkgs.callPackage generated {
          buildRustCrateForPkgs = cratePkgs: crate:
            (cratePkgs.buildRustCrate.override { defaultCrateOverrides = crateOverrides; }) (crate // pkgs.lib.optionalAttrs
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
        cargoNix = cargoNixWith null;
      in {
        devShells.default = pkgs.mkShell {
          # clippy + rustfmt come from the same nixpkgs pin as cargo/rustc so the
          # `cargo fmt` / `cargo clippy` subcommands resolve to a matching toolchain
          # (CI runs them inside this shell; no rustup is present there).
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt pkgs.pkg-config ];
          buildInputs = [ gtsam pkgs.eigen pkgs.boost pkgs.tbb ];
          env = buildEnv;
        };

        packages.default = cargoNix.rootCrate.build.overrideAttrs (old: {
          meta = (old.meta or { }) // { mainProgram = "gsc-pgo"; };
        });

        packages.lint = (cargoNixWith "lint").rootCrate.build.override {
          runTests = true;
          testCrateFlags = [ "--list" ];
        };
        checks.lint = self.packages.${system}.lint;

        packages.tests = (cargoNixWith "test").rootCrate.build.override { runTests = true; };
        checks.tests = self.packages.${system}.tests;
      });
}
