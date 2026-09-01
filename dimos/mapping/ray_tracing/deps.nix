_: {
  sources."dimos/mapping/ray_tracing/rust" = ./rust;
  # The `py` member is a pyo3 cdylib for the python side, not a module
  # executable, so it is not listed here.
  binaries = [ "dimos-voxel-ray-tracing" ];
}
