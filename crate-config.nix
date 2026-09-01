# crate2nix's per-crate override hook, read by Cargo.nix. Empty because no
# crate needs an override yet.
#
# It exists as a real file rather than being left absent -- Cargo.nix guards it
# with `builtins.pathExists` -- so that `bin/build-native-modules --inputs-hash`
# has something to hash. That gate scans path literals textually and cannot
# evaluate the guard, so an absent-but-referenced file would either break it or
# have to be exempted, and an override added here later would then not rekey the
# Cachix marker.
_: { }
