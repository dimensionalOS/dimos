{
  description = "Point-LIO + Livox Mid-360 native module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    livox-sdk.url = "path:../../livox/cpp";
    livox-sdk.inputs.nixpkgs.follows = "nixpkgs";
    livox-sdk.inputs.flake-utils.follows = "flake-utils";
    livox-sdk.inputs.lcm-extended.follows = "lcm-extended";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    fast-lio = {
      # Point-LIO fork (split out of dimos-module-fastlio2's pointlio branch).
      # Repo is org-internal for now, hence git+ssh instead of github:.
      url = "git+ssh://git@github.com/dimensionalOS/dimos-module-pointlio?ref=main";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nixpkgs, flake-utils, livox-sdk, dimos-lcm, fast-lio, lcm-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        # Overlay fixes for darwin-broken nixpkgs recipes in our transitive
        # dep chain (pcl → vtk → pdal → tiledb → libpqxx).  Each of these
        # should go upstream; kept here so we can build in the meantime.
        #
        # Gated on isDarwin so Linux keeps binary-cache hits for the stock
        # libpqxx / tiledb / pdal / vtk / pcl derivations.  Applying the
        # override on Linux would change their input hashes and force a
        # from-source rebuild of the whole chain for no benefit.
        darwinDepFixes = final: prev:
          if !prev.stdenv.isDarwin then { } else {
            # libpqxx: postgresqlTestHook is in nativeCheckInputs
            # unconditionally and that package is marked broken on darwin.
            # The list is eagerly evaluated, so simply referencing it aborts
            # eval.  Upstream fix is to wrap the list in
            # `lib.optionals (meta.availableOn ...)`.
            libpqxx = prev.libpqxx.overrideAttrs (_old: {
              nativeCheckInputs = [ ];
              doCheck = false;
            });
            # tiledb: darwin-only patch `generate_embedded_data_header.patch`
            # targets a file that doesn't exist in tiledb 2.30.0 (the
            # upstream code path was reworked and `file(ARCHIVE_CREATE ...)`
            # is no longer used anywhere in the source).  Filter out only
            # that patch — don't drop everything, in case nixpkgs adds an
            # unrelated security patch in a future bump.
            tiledb = prev.tiledb.overrideAttrs (old: {
              patches = builtins.filter
                (p: !(prev.lib.hasSuffix "generate_embedded_data_header.patch" (toString p)))
                (old.patches or [ ]);
            });
          };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ darwinDepFixes ];
        };
        livox-sdk2 = livox-sdk.packages.${system}.livox-sdk2;
        lcm = lcm-extended.packages.${system}.lcm;

        livox-common = ../../common;

        # Patch the Point-LIO fork in place: resize (not reserve) the per-point
        # vectors in run_once, whose reserve+operator[] is out-of-bounds UB that
        # macOS libc++'s hardened operator[] turns into a SIGTRAP. applyPatches
        # gives a writable, patched copy of the read-only flake input.
        fast-lio-patched = pkgs.applyPatches {
          name = "fast-lio-pointlio-patched";
          src = fast-lio;
          patches = [ ./fastlio-resize-darwin.patch ];
        };

        pointlio_native = pkgs.stdenv.mkDerivation {
          pname = "pointlio_native";
          version = "0.2.0";

          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            livox-sdk2
            lcm
            pkgs.glib
            pkgs.eigen
            pkgs.pcl
            pkgs.glog
            pkgs.boost
            pkgs.llvmPackages.openmp
          ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFASTLIO_DIR=${fast-lio-patched}"
            "-DLIVOX_COMMON_DIR=${livox-common}"
          ];
        };
      in {
        packages = {
          default = pointlio_native;
          inherit pointlio_native;
        };
      });
}
