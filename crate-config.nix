# Placeholder. Cargo.nix takes a `crateConfig` argument that defaults to
# importing this path, but never reads it -- the argument is unused in the
# generated file. So do not add crate overrides here expecting them to apply;
# the working hook is the `defaultCrateOverrides` argument to Cargo.nix, set
# where flake.nix imports it.
#
# The file exists only so that the path Cargo.nix names actually resolves.
# `bin/build-native-modules --inputs-hash` walks nix path literals textually
# and cannot evaluate the `builtins.pathExists` guard around this one, so it
# fails outright when the file is absent:
#
#   Cargo.nix: reference './crate-config.nix' does not exist
#
# An empty file is a cheaper fix than teaching that parser about conditional
# paths, which would risk it silently skipping a path that does matter.
_: { }
