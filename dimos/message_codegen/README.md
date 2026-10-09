# Message build implementation

Custom-message authors should start with the self-contained
[external-project walkthrough](../../docs/development/messages-external-project.md).
A project declares its name/version and selected languages once in `pyproject.toml`;
`interfaces/<package>/msg/*.msg` supplies its definitions. `dimos build` and the
PEP 517/660 backend share `Project`, dependency resolution and generation. Neither
builds the whole dimOS runtime. Built-in messages are distributed separately.

## Native libraries and ownership

- Python uses unmodified **rosbags 0.11.0** dataclasses from one frozen typestore.
  Generated package imports alias those classes. The pinned upstream
  `generate_python_code` API emits the static `.pyi` declarations; runtime classes
  are neither rewritten nor monkey-patched. `dimos_message_build.registry.encode`
  and `decode` call the native typestore. There are no Message/Sequence wrappers.
- C++ uses full upstream **ROSIDL 4.6.9 / FastRTPS 3.6.4** source generators and
  **Fast CDR 2.4.0**. `native_sources.json` pins the support source closure.
  Native preparation builds that closure in a writable cache; CMake consumers
  link the generated typesupport libraries as well as Fast CDR. This requires no
  system ROS installation, but it does use ROSIDL/ament support libraries and
  build resources. It is not evidence of ROS node integration.
- Rust uses unmodified **ros2msg 0.5.3** during Cargo's normal `build.rs` step.
  Its public callback adds Serde/PartialEq derives and fixed-array annotations.
  **re_cdr 0.1.0** owns wire serialization. SDK framing supplies the four-byte
  XCDR1 header and verifies complete consumption. Generated declarations live in
  Cargo's `OUT_DIR`; no class emitter or generated-output rewrite remains here.

Dependencies own their types. A custom message containing `std_msgs/Header`
reuses the dependency's Python class, C++ declaration and Rust crate type.
Manifests reject conflicting owners, versions, ABIs and schema hashes. The entire
ros2msg schema closure still travels with each root message for MCAP/viewers;
Rust exposes that metadata as `ROS2MSG_SCHEMAS`. Metadata is not a wire codec.

## Build contracts

`pip install .` and `pip install -e .` generate/package only Python resources.
They require no C/C++/Rust compiler and do not prepare native support. Editable
users rerun installation after `.msg` edits and restart consumers. Wheel install,
message imports and `dimos list` perform no native compilation, generated-file
writes or downloads. On first registry initialization, rosbags builds its native
Python classes and codecs in memory from the installed schemas; this is Python
library initialization, not precompiled Python class delivery.

An explicit C++ build currently supports Linux and requires C/C++, CMake and the
backend's `native` Python extra. It can fetch pinned upstream sources at build
time; offline preparation requires cached sources. The exported local CMake
`toolchain.cmake` supplies installed prefixes, the build interpreter and ament
Python resources. Regenerate it after moving the environment. It is not a
portable binary SDK archive.

Explicit Rust builds use ordinary Cargo dependency resolution. An installed
Python-only dependency can supply schemas from which the shared core prepares
its native source crate without mutating the installed package. The Cargo bundle
uses relative dependency paths. Offline builds require Cargo's dependency cache.
No frontend silently installs OS tools or downloads executable toolchains.

`python -m dimos.message_codegen.generate` remains a low-level fixture/maintainer
entrypoint; normal custom-message users do not need its root/type/output flags.
Maintainers refresh built-in checked-in resources with:

```sh
python -m scripts.generate_builtin_messages
python -m scripts.generate_builtin_messages --check
```

## Native semantics and incomplete acceptance

Python/Rust constructors require explicit fields. Rosbags does not apply `.msg`
defaults; Rust's upstream floating default emission is not usable for this catalog,
so the public `derive_default(false)` option is selected. Python numeric arrays
are native NumPy arrays. Construction does not clone nested objects, and native
NumPy-backed dataclass equality is not scalar equality. Runtime helpers explicitly
copy or expose read-only views where their own contracts require those behaviors.

Bounds are not silently ignored: Python encode/decode rejects affected bounded
closures; selected C++/Rust bounded strings and Rust wide strings fail generation.
These are unsupported semantics, not successful conformance tests.

The user provisionally accepted the native-library limitations as proposal
non-blockers on 2026-10-09. The original strict assertions remain, with precise
`xfail(strict=True)` cases linked to the [limitation register](../../docs/development/message-limitations.md).
This records permissive malformed boolean/string/representation decoding,
exact-consumption differences, and unsupported bounded schemas. It is not a
parser-safety or correctness claim. Use schema-matched inputs from trusted
producers; the Python decoder is not an untrusted-input validation boundary.
No custom decoder, upstream patch, silent bound truncation, or broad test skip
was introduced. Valid bounded Python messages still fail as unsupported.

## Provenance

`sources.json` records immutable revisions/hashes of the schema catalog and the
standalone vendored ROSIDL parser. That parser retains its documented linear-time
constant-name regex patch; it is not described as pristine upstream code.
Schema package XML files and `schemas/licenses` retain upstream license notices.
The former partial vendored C++ generator/template subtree is removed. Full
native source pins live separately in `native_sources.json`.
