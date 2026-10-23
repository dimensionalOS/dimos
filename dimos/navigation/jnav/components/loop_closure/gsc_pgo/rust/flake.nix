{
  # Build/test workflow (this crate lives inside a larger git repo, so use
  # the path: ref to keep nix from copying the whole repo to the store):
  #
  #   cd dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust
  #   nix develop path:. --command cargo test
  #
  # `nix build` must use `.#default` rather than `path:.`: the package pulls in
  # the native/rust path deps, which sit above this directory.
  description = "dimos-gsc-pgo: lidar loop closure (Scan Context + ICP) and pose graph optimization, all Rust";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/549bd84d6279f9852cae6225e372cc67fb91a4c1";
    flake-utils.url = "github:numtide/flake-utils/11707dc2f618dd54ca8739b309ec4fc024de578b";
    crate2nix = {
      url = "github:nix-community/crate2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, crate2nix, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        crateSubdir = "dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust";

        # Reassemble the crate next to the in-repo path deps it references
        # (native/rust/dimos-module{,-macros}) so cargo's relative `path = `
        # entries resolve inside the sandbox.
        src = pkgs.runCommand "dimos-gsc-pgo-src" { } ''
          mkdir -p "$out/${crateSubdir}"
          cp -r ${./src} "$out/${crateSubdir}/src"
          cp ${./Cargo.toml} "$out/${crateSubdir}/Cargo.toml"
          cp ${./Cargo.lock} "$out/${crateSubdir}/Cargo.lock"
          cp ${./crate-hashes.json} "$out/${crateSubdir}/crate-hashes.json"

          mkdir -p "$out/native/rust"
          cp -r ${../../../../../../../native/rust/dimos-module} "$out/native/rust/dimos-module"
          cp -r ${../../../../../../../native/rust/dimos-module-macros} "$out/native/rust/dimos-module-macros"
        '';

        # crate2nix turns Cargo.lock into one derivation per crate, so there is
        # no vendor-tarball `cargoHash` to regenerate whenever a dependency
        # moves. `crate-hashes.json` pins the git dependency (lcm-msgs);
        # refresh it with `crate2nix generate` when that ref changes.
        cargoNix = pkgs.callPackage
          (crate2nix.tools.${system}.generatedCargoNix {
            name = "dimos-gsc-pgo";
            inherit src;
            cargoToml = "${crateSubdir}/Cargo.toml";
          })
          {
            defaultCrateOverrides = pkgs.defaultCrateOverrides;
          };
      in {
        devShells.default = pkgs.mkShell {
          # clippy + rustfmt come from the same nixpkgs pin as cargo/rustc so the
          # `cargo fmt` / `cargo clippy` subcommands resolve to a matching toolchain
          # (CI runs them inside this shell; no rustup is present there).
          packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt pkgs.pkg-config ];
        };

        packages.default = cargoNix.rootCrate.build.overrideAttrs (old: {
          meta = (old.meta or { }) // { mainProgram = "gsc-pgo"; };
        });
      });
}
