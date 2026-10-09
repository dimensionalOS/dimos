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
0.11.0 code generation. An AST adapter attaches the Message base and owned
Sequence annotations. Dataclasses supply equality and representation. The
remaining runtime supplies declared defaults, assignment validation and borrowed
NumPy ownership. These are the Python value API, not requirements of CDR itself.

The independent Python CDR layout walker has been deleted. Rosbags owns traversal,
alignment and primitive decoding. A scoped compiler callback adds only strict
bool, string length/terminator and sequence-length checks to the pinned upstream
decoder. It clones the generator function with a local compilation callback and
does not modify library globals. Tests cover every truncation, malformed bool
scalars/arrays, invalid strings, both byte orders, legal nonzero padding and
signaling NaNs. A decode/reserialize comparison was rejected because it would
reject valid signaling NaNs; it is not part of production decoding.

C++ declarations/defaults use ROSIDL Jazzy. Serialization, deserialization and
size calculation use the official ROSIDL FastRTPS generator template fragment,
not DimOS field-visitation loops. The former bounded-vector copy adapters and
C++ type/keyword mapping tables are deleted. Remaining adapters expose the
Fast CDR customization interface and validate message bounds. Consumers still
need Fast CDR but no ROS installation or ROS node/type-support runtime.

The serialization fragment is pinned separately under
`_vendor/rosidl/serialization`: its manifest records the original complete
upstream template hash, extracted fragment hash and namespace/inline patches.
Two bool checks preserve rejection before the upstream uint8-to-bool conversion.
No ROS transport or service/action support is implied.

Vendored package entry points live in ordinary `api.py` modules, with matching
typing stubs. Imports and EmPy templates reference those modules directly; the
empty upstream parser initializer is omitted. This preserves namespace-package
policy without exempting vendored files from the repository test. Source manifests
retain original upstream paths/hashes and record the layout/import patch.

Rust declarations, primitive/nested/array type mappings and constants now come
from pinned **ros2msg 0.5.3** during the crate's normal Cargo `build.rs` step.
DimOS supplies only defaults, schema/codec metadata, bounds validation and
cross-package reexports. The defaults adapter handles the verified upstream
string/floating-array default gap. Standard Header references reexport the
owning dependency's type rather than regenerating it.

Python generation packages the Rust build script and owned `.msg` inputs without
invoking Cargo. `pip install` and wheel use therefore require no Rust compiler.
Explicit Rust builds (including `dimos build --language rust` in the authoring
layer) run the pinned build dependency through ordinary Cargo, which caches it.
Generated Rust declarations live in Cargo's `OUT_DIR`; no executable is shipped
or downloaded at Python import. Cargo's ordinary dependency setup and lockfile
control network/offline/reproducible native builds. No new tool manager exists.

### Responsibility and code accounting

No backend file is merely renamed or hidden. `rust.py` now emits compatibility
adapters and crate inputs; `cpp.py` invokes upstream declaration/serialization
templates. `python.py` adapts upstream classes. The custom Python Sequence API
still provides copy, append/extend/clear and read-only borrowed views with resize
guards; those compatibility conveniences are not described as wire requirements.

The accounting includes the new Rust build script and EmPy orchestration template
as maintained code, separately from upstream vendored template fragments and
mechanically generated package artifacts. Against original generator
`d7d3e4e2e9280e05d3d507bc507f57d08c6af1db` (1,463 lines across eight core files),
the current ten files are:

| File | Physical lines |
| --- | ---: |
| python.py | 163 |
| cpp.py | 170 |
| rust.py | 138 |
| definitions.py | 193 |
| generate.py | 99 |
| templates/runtime.py | 383 |
| templates/codec.rs | 56 |
| templates/dimos_cdr.hpp | 71 |
| templates/message_build.rs | 99 |
| templates/idl_cdr.hpp.em | 8 |
| Total, including both new helpers | 1380 |

Counts include blank lines and comments. Tests, schema inputs, vendored sources
and generated outputs are not counted as handwritten generator/runtime code.
The point of this change is removing duplicate generation rules, not relocating
those rules into an uncounted helper.

The core-only reduction is 1,463 to 1,380 (83 lines). Including maintained typing
stubs inside `_vendor` changes the comparison to 1,493 to 1,444 (49 lines): the
old stub had 30 lines; the current three stubs total 64. These PR1 counts exclude
the later layers' ownership, build and packaging implementation.
