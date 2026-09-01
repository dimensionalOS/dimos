# Names the cargo package that is this module's executable, so the root flake
# can give it a build of its own. Crate sources and workspace membership come
# from Cargo.nix and Cargo.toml and are not repeated here.
_: {
  # The `py` member is a pyo3 cdylib for the python side, not a module
  # executable, so it is not listed here.
  binaries = [ "dimos-voxel-ray-tracing" ];
}
