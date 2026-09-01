# Names the cargo package that is this module's executable, so the root flake
# can give it a build of its own. Crate sources and workspace membership come
# from Cargo.nix and Cargo.toml and are not repeated here.
_: {
  binaries = [ "dimos-mls-planner" ];
}
