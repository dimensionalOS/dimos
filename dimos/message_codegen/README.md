# Standalone native message generation

This first CDR layer defines the final generation/build contract. It does not
change the running dimOS message catalog, transports or recording formats.
Later layers provide built-in packages, project/CLI frontends and runtime integration.

- Python: unmodified rosbags 0.11.0 dataclasses and codecs in one frozen registry.
- C++: full pinned ROSIDL/FastRTPS generators and Fast CDR 2.4.0, built with CMake.
- Rust: ros2msg 0.5.3 in Cargo `build.rs`, serialized by re_cdr 0.1.0.

There are no generated Message/Sequence wrapper classes. The retained ROSIDL
MSG parser and its upstream conformance tests have pinned source hashes; it is
not a replacement parser. This package layer adds the complete standard/built-in schema catalog and a
compiler-free `dimos-generated` distribution. CI checks regenerated source drift
and installs the wheel, source and editable package with C/C++ compilers disabled.

## Generate one package

Install the lightweight core from this proposal checkout:

```sh skip
python scripts/prepare_message_parser.py
pip install './dimos/message_codegen[native]'
```

Create `interfaces/demo_msgs/msg/Reading.msg`:

```text
std_msgs/Header header
float64 value
```

For this low-level generator layer:

```sh skip
python -m dimos.message_codegen.generate --package-root interfaces \
  --type demo_msgs/msg/Reading --python-module demo_messages --output build/reading
pip install ./build/reading/python
```

The next authoring layer supplies project discovery and `dimos build`; those
frontends are not prerequisites for this generator or its tests.

```python skip
from demo_messages.demo_msgs.msg import Reading
from demo_messages.std_msgs.msg import Header
from demo_messages.builtin_interfaces.msg import Time
from dimos_message_build.registry import encode, decode

value = Reading(header=Header(stamp=Time(sec=1, nanosec=2), frame_id="map"), value=20.5)
assert decode(encode(value), Reading).value == 20.5
```

Constructors require explicit fields. NumPy owns numeric arrays. A nested value
is shared unless the caller copies it explicitly. Separate message packages reuse
the dependency-owned Python class, C++ declaration and Rust crate type; manifests
validate schema hashes, owner identity, versions and ABI before native building.

## Native prerequisites and platform validation

C++ source builds support Linux and macOS. Supply a C/C++ compiler and CMake;
macOS additionally needs Apple's Command Line Tools (`xcrun` and a macOS SDK).
The build forwards SDK, deployment target and architecture to every nested CMake
project, separates target configurations in its cache, and uses the platform's
loader-relative library paths. It does not install system tools or require ROS.

`prepare_cpp` builds pinned source dependencies only when explicitly requested:

```python skip
from pathlib import Path
from dimos_message_build.native_build import prepare_cpp, write_cmake_toolchain

prefix = prepare_cpp(Path("build/reading"))
write_cmake_toolchain(prefix, Path("build/toolchain.cmake"))
```

CMake consumers use `find_package(demo_messages CONFIG REQUIRED)` and link
`demo_messages::messages`. Cargo uses `build/reading/rust/Cargo.toml`; generated
Rust declarations live in Cargo's `OUT_DIR`. No native build happens at import.
Python wheel installation requires no compiler. Offline native building requires
cached pinned sources and Cargo dependencies; missing prerequisites fail explicitly.

## Validation

```sh skip
bash scripts/test_message_codegen.sh
```

The standalone CI matrix runs this on Ubuntu and macOS ARM64. It compiles and
executes a dependency-owned Header exchange through Python, C++ and Rust, then
checks the decoded result and schema/version conflicts. This is a codec/consumer
check, not native SDK pub/sub or a ROS node integration. Jazzy independently reads
the resulting payloads in its Linux reference container.

Linux tests which simulate macOS target configuration are only configuration
tests. Actual macOS acceptance requires the hosted macOS matrix job to pass.

[Accepted native-library limitations](../../docs/development/message-limitations.md)
remain explicit: unsupported bounds and permissive native decoder behavior are
not fixed by wrappers or weakened assertions. The later full-catalog package
layer retains its complete strict expected-failure suite.

## Upstream parser preparation and offline installation

The parser is imported directly from upstream `rosidl_adapter.parser`; its source
is not checked into this repository. `parser-source.json` pins the ROSIDL revision,
archive SHA256 and unchanged parser SHA256. Preparation copies the upstream Python
package, Apache-2.0 license and package metadata into ignored `.upstream/` output.
It never patches upstream code. A small constant-name guard in `definitions.py`
rejects invalid names before upstream's backtracking regex runs.

Prepare once before building this backend from a repository checkout:

```sh
python scripts/prepare_message_parser.py
pip wheel ./dimos/message_codegen --no-deps --wheel-dir dist
```

Preparation needs `requests`; install it explicitly in the preparation environment.
It downloads only the pinned project archive. Reuse the cache without network:

```sh
python scripts/prepare_message_parser.py --offline
# Or seed a new cache with the same hash-verified archive:
python scripts/prepare_message_parser.py --offline --archive /path/to/rosidl.tar.gz
```

The backend wheel **and sdist contain these already-prepared upstream files**.
Ordinary `pip install backend.whl` and `pip install backend.tar.gz` do not fetch
GitHub. An unprepared repository source build fails with the preparation command;
it never downloads inside a PEP517 hook or at runtime. No upstream parser wheel
on PyPI is assumed: this is our backend artifact containing licensed upstream
files. Distribution of matching proposal wheels remains required.

A fully offline PEP517 source installation also requires setuptools/wheel and
normal pinned Python dependencies in a wheelhouse (`--no-index --find-links`).
Python wheels need no native compiler. C++ source preparation remains separate
and retains its existing explicit toolchain and offline source-cache contract.
The unchanged parser package is platform-independent; actual macOS checks remain
CI acceptance, not something proven by Linux configuration tests.
