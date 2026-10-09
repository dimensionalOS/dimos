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
the original and patched hashes.

The following message-package layer provides `scripts/vendor_message_definitions.py`
for intentional updates of pinned upstream inputs and reapplication of the parser
patch. It is not part of this standalone generator layer. Applications and builds
never run a maintenance downloader.

This work is being delivered through the `replace-lcm-message-encoding` OpenSpec
change. The generated pipeline is under development; the old runtime message APIs
have not yet been replaced.

Generation resolves only explicitly supplied roots by default in this layer.
The following package layer provides bundled standard definitions. Raw LCM bus
interop belongs to the runtime layer; this layer tests CDR bytes independently
of transport.

## Upstream generation and remaining adapters

Python class declarations, annotations and constants come from pinned rosbags
0.11.0 `generate_python_code`, not a DimOS field-to-Python type emitter. A small
AST adapter attaches the existing Message base and maps array annotations to the
owned Sequence API. Standard dataclasses supply equality and representation.
The adapter preserves upstream generated-code attribution.

Rosbags does not preserve declared field defaults in its emitted dataclasses.
The Message/Sequence runtime still supplies those defaults, constructor and
assignment validation, dependency-owned nested values, read-only borrowed views
and resize guards. Its malformed-input precheck still traverses CDR layout; it
must not be described as entirely library-owned serialization validation.

C++ struct declarations, constructors, default values, constants, equality and
field types now come from the pinned upstream Jazzy `rosidl_generator_cpp`, using
`rosidl_adapter` for MSG-to-IDL conversion. Vendored upstream resources have a
separate revision/hash manifest under `_vendor/rosidl`; namespace relocation is
recorded. EmPy 4.2 and Lark 1.2.2 are build-time Python dependencies. Generated
headers include the minimal upstream runtime declarations, so consumers need no
ROS installation. DimOS still supplies metadata, validation and Fast CDR field
visitation. Bounded-vector adaptation currently copies through a standard vector;
this is not claimed as zero-copy.

The Rust emitter has not yet been replaced. Actual compatibility probes
found that Fast-DDS-Gen 4.3.0 rejects array `@default({...})`, while ros2msg 0.5.3
emits uncompilable string and floating-array defaults for the existing Telemetry
fixture. ROS Jazzy's C++ generator does preserve these defaults and public fields;
its types compile using only packaged runtime headers, without a ROS installation.
The bounded-vector codec adapter passes the same conformance checks. These are isolated
integration gaps, not evidence that an entire custom generator is necessary.
