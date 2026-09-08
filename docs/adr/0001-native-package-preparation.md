# Share native package preparation through Nix

DimOS native launchers depended on build directories beside Python modules,
which are absent from pip installations. The recorder, Livox Mid360, PointLIO,
and FastLIO2 now share a package catalog and automatic preparation: Nix reuses
local outputs, substitutes from Cachix, or builds the selected sources. Wheels
and source distributions carry an immutable source revision; editable
checkouts use local inputs.

We retain Nix as a runtime prerequisite instead of distributing portable
binaries in Python wheels. This reuses the existing CI publisher and preserves
native dependency closures, at the cost of Nix installation and potentially
slow first-use compilation. Builds are automatic, with conservative
parallelism; process lifecycles remain owned by NativeModule or the recording
context. GPU-specific and Cargo-only packaging remains separate work.
