# Standalone ROS2 message generation

This package reads ROS2 `.msg` files without importing or installing ROS. A pinned
upstream `rosidl_adapter` parser validates syntax. DimOS resolves package names and
dependencies, then emits C++ value types and Fast CDR customizations, ordinary Python source values, and native Rust types using Serde and `re_cdr`.

```bash
python -m dimos.message_codegen.generate \
  --package-root examples/message-codegen \
  --type demo_msgs/msg/Telemetry \
  --output build/message-codegen/demo
```

The output contains `cpp/`, `rust/`, and `schemas.json`. Definitions are resolved
before output is written. Repeat `--package-root` and `--type` for multiple inputs;
omitting `--type` generates all available definitions. The output belongs in an
ignored build directory. Generation never downloads dependencies.

Python emits plain source values and uses rosbags 0.11.0 for CDR; C++ uses
Fast CDR 2.4.0. Python message use requires no native message compilation. Rust uses
`re_cdr` 0.1.0 and `serde-big-array` 0.5.1. The current generator explicitly rejects
`wstring` because the selected Rust backend has no matching wide-string Serde
representation. No bundled definition uses it. Service/action generation is out
of scope.

The C++ generator emits declarations, defaults, validation, and library calls;
Fast CDR owns sizing, alignment, strings, sequences, primitive representation,
and byte order. The Rust backend delegates those operations to `re_cdr`. Both
emit plain CDR/XCDR1 with its standard four-byte encapsulation.

## Upstream sources and licenses

`sources.json` records immutable revisions and SHA-256 hashes for the vendored
files. The full standard definition collection, licenses and generated message packages
belong to the following message-package layer. This layer contains only small
self-contained test definitions under `examples/message-codegen`. The parser's
original copyright header is preserved. `rosidl_parser.pyi` is DimOS's type stub;
the upstream implementation has one documented patch replacing an ambiguous
constant-name regex with its linear-time equivalent. `sources.json` records both
the original and patched hashes; the maintenance script reapplies the patch.

To refresh the pinned inputs intentionally, edit the revisions in
`scripts/vendor_message_definitions.py`, then run that maintenance command with
`requests` installed. Review the source diff and update the conformance evidence.
Applications and builds never run the maintenance downloader.

This work is being delivered through the `replace-lcm-message-encoding` OpenSpec
change. The generated pipeline is under development; the old runtime message APIs
have not yet been replaced.

Generation resolves only explicitly supplied roots by default in this layer.
The following package layer provides bundled standard definitions. Raw LCM bus
interop belongs to the runtime layer; this layer tests CDR bytes independently
of transport.
