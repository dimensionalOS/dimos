# Standalone ROS2 message generation

This package reads ROS2 `.msg` files without importing or installing ROS. A pinned
upstream `rosidl_adapter` parser validates syntax. DimOS resolves package names and
dependencies, then emits C++ value types and Fast CDR customizations, Python
bindings to those types, and native Rust types using Serde and `re_cdr`.

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

Python and C++ use Fast CDR 2.4.0; Python bindings use pybind11 3.0.1. Rust uses
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
files. `schemas/*/package.xml` retains upstream authorship and declared licenses.
`schemas/licenses/` contains upstream license texts, including the Apache 2.0
license shared by the standard interface packages and parser. The parser's
original copyright header is preserved. `rosidl_parser.pyi` is DimOS's type stub;
the upstream implementation remains unmodified.

To refresh the pinned inputs intentionally, edit the revisions in
`scripts/vendor_message_definitions.py`, then run that maintenance command with
`requests` installed. Review the source diff and update the conformance evidence.
Applications and builds never run the maintenance downloader.

This work is being delivered through the `replace-lcm-message-encoding` OpenSpec
change. The generated pipeline is under development; the old runtime message APIs
have not yet been replaced.
