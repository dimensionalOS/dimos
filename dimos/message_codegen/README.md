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

The `typing/<module>/` output describes the native Python fields, keyword-only
constructors, sequence operations, and NumPy views. For a local generated build,
point `MYPYPATH` at `typing/`. Wheels install the same interfaces as a PEP 561
`<module>-stubs` package, so callers get type checking without source generation
or runtime introspection. Fixed-array interfaces omit resizing methods.

DimOS-owned definitions live under `schemas/dimos_msgs/msg/`. They use the same
generation and packaging path as the pinned standard definitions. In particular,
weighted line segments have explicit endpoints and weights; they do not reuse
Path poses or quaternion fields. Stamped custom messages use standard Header,
and trajectory durations use builtin_interfaces/Duration. Numeric convenience
operations belong in helpers such as `dimos.msgs.time`, outside generated types.

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

## Distribution

Pass `--package` to emit a setuptools source project in `python/`, alongside the
CMake project and Cargo crate. `--python-module` gives independent message
packages distinct extension names; `--version` sets the package version.
The Python sdist contains the generator, its pinned parser, and all definition
inputs. Building the sdist regenerates source without ROS. The wheel contains
native code plus definitions, licenses, and the `dimos.messages` provider.

DimOS's own wheel uses the same `MessageBuildExt` and generator from `setup.py`.
The release workflow preserves the existing Linux x86_64/aarch64 and macOS arm64
wheel matrix and adds CMake/schema and Cargo source packages to GitHub releases.
Source developers run `bash scripts/setup_message_codegen.sh` before building;
`scripts/install.sh --mode dev` does this during dependency setup. Ordinary wheel
users need no compiler, generator run, or ROS installation.
