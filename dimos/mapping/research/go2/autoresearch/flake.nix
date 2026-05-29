{
  description = "LIO autoresearch: Point-LIO build deps (pcl, yaml-cpp, boost) layered onto the dimos dev shell";

  inputs = {
    # The root dimos flake, pinned to a committed rev. The pin is load-bearing:
    # this repo's worktree is ~100 G and usually dirty, so a bare git+file would
    # copy the whole thing into the store. Pinning a rev exports from git objects
    # instead — LFS files stay as pointers, untracked files are ignored, ~16 M
    # closure. Bump this rev only if the root flake's dev shell changes.
    dimos.url = "git+file:../../../../..?rev=6c24579da002fb050231a9486d2b008cc0d7cada&shallow=1";
    nixpkgs.follows = "dimos/nixpkgs";
    flake-utils.follows = "dimos/flake-utils";
  };

  outputs = { self, dimos, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in {
        # The full dimos dev shell + the three native deps Point-LIO needs that
        # the root flake doesn't already carry (eigen and cmake come from root).
        devShells.default =
          dimos.devShells.${system}.default.overrideAttrs (old: {
            buildInputs = (old.buildInputs or []) ++ [
              pkgs.pcl
              pkgs.yaml-cpp
              pkgs.boost
            ];
            # nixpkgs lays PCL headers under include/pcl-<major.minor>/, which
            # point_lio's hand-rolled find_path (HINTS only /usr/include/pcl*)
            # can't see. Put that dir on CMAKE_INCLUDE_PATH, which find_path
            # honors — keeps the vendored substrate untouched. Libs resolve via
            # CMAKE_PREFIX_PATH (pcl is in buildInputs).
            shellHook = (old.shellHook or "") + ''
              for d in ${pkgs.pcl}/include/pcl-*; do
                export CMAKE_INCLUDE_PATH="$d''${CMAKE_INCLUDE_PATH:+:$CMAKE_INCLUDE_PATH}"
              done
            '';
          });
      });
}
