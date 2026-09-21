# Stage 1 local verification

Captured on 2026-09-21 on Linux x86_64 with Python 3.12, GCC 16, and Rust 1.98.
These are terminal demos intended for human inspection, executed automatically.
The ROS reference runs independently in a Jazzy container.

The full build generated 142 message types. Focused verification after adding
the all-types oracle passed **22 tests**. The raw transport passed nine unit tests
and one doctest. `cargo build --offline` also passed after dependency setup.

## relay

```text
Python sends: sequence=40 label=start hops=[1]
C++ received: frame=map sequence=40 label=start temperature=21.5 x=4.25 axes=[1,2,3]
Rust received: frame=map sequence=41 label=start/cpp temperature=21.5 x=4.25 axes=[1.0, 2.0, 3.0]
Python receives: sequence=42 label=start/cpp/rust hops=[1, 2, 3]
Nested fields, fixed arrays, bytes, defaults, and edits survived. Evidence: build/message-codegen/demo/evidence
```

## conformance

```text
LE Python -> Python: fields and bytes match
LE Python -> C++   : fields and bytes match
LE Python -> Rust  : fields and bytes match
LE C++    -> Python: fields and bytes match
LE C++    -> C++   : fields and bytes match
LE C++    -> Rust  : fields and bytes match
LE Rust   -> Python: fields and bytes match
LE Rust   -> C++   : fields and bytes match
LE Rust   -> Rust  : fields and bytes match
BE Python -> Python: fields and bytes match
BE Python -> C++   : fields and bytes match
BE Python -> Rust  : fields and bytes match
BE C++    -> Python: fields and bytes match
BE C++    -> C++   : fields and bytes match
BE C++    -> Rust  : fields and bytes match
BE Rust   -> Python: fields and bytes match
BE Rust   -> C++   : fields and bytes match
BE Rust   -> Rust  : fields and bytes match
Python/C++/Rust reject truncated
Python/C++/Rust reject unsupported representation
Python/C++/Rust reject trailing bytes
Python/C++/Rust reject missing header
All nine encoder/decoder combinations and both input byte orders match the independent ROS2 codec.
```

## buffers

```text
640x480 RGB image: 921,600 bytes, shared read-only view; resize rejected
  local median (microseconds): borrow=4.2, copy=31.6, encode=71.4, decode=74.2
100,000-point buffer: 1,200,000 bytes, shared read-only view; resize rejected
  local median (microseconds): borrow=5.7, copy=47.0, encode=119.3, decode=128.2
Borrowed image remains readable after its message variable is released.
```

## transport

```text
Python -> native Rust -> Python: 128 raw bytes match
Python -> native Rust -> Python: 65536 raw bytes match
Python -> native Rust -> Python: 1048576 raw bytes match
```

## jazzy-reference

```text
ROS2 Jazzy decoded python-le.cdr: every field matches
ROS2 Jazzy decoded python-be.cdr: every field matches
ROS2 Jazzy decoded C++.cdr: every field matches
ROS2 Jazzy decoded Rust.cdr: every field matches
ROS2 Jazzy confirms declared defaults, nested fields, bounds, arrays, Unicode, and both byte orders.
```

Reproduce with the commands in [README.md](README.md). CI retains fresh output
and binary payloads as artifacts; generated sources and build products are ignored.

## Stage 2 installed application

Built a Python wheel from its sdist, installed CMake packages, and compiled a
separate consumer from the Cargo archive. The Python consumer ran with `-I` in a
fresh environment containing only the external message wheel.

```text
Installed Python package sends new field: added-locally
Installed C++ package received: added-locally
Packaged Rust crate received: added-locally/cpp
Installed Python package receives: added-locally/cpp/rust
A locally added field crossed three installed native packages; no DimOS source change or upstream PR.
```

The full DimOS wheel also built with web assets. Its sdist passed the repository
content check (1,928 entries; 10.7 MB). A fresh environment with both wheels
discovered 141 types and accepted a built-in Point inside external Telemetry.
Two separately installed CMake packages can include the same standard Point
without duplicate definitions. Focused tests now pass 34 cases.
