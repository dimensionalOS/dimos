# One message in Python, C++ and Rust

This directory is a standalone message project. Install matching dimOS artifacts
with the `message-build` extra, activate that Python environment, and provide
normal CMake/C++ and Rust/Cargo toolchains. For source installation, pip's isolated
build environment must receive the same matching wheel set.

The definition and pyproject are the source of truth. No `dimos build` command or
per-type generation flags are used. Python installation selects only Python;
CMake and Cargo independently select their own outputs through the shared core.

Set up installed native dependency locations once:

```sh
export CMAKE_PREFIX_PATH="$(python -m dimos.message_codegen.native cmake-prefix)"
python -m dimos.message_codegen.native configure-cargo
```

Then run the complete value/codec example:

```sh
pip install .
python -I demo_reading.py write python.cdr
cmake -S . -B build/cpp
cmake --build build/cpp
build/cpp/telemetry_relay python.cdr cpp.cdr
cargo build --example relay
./target/debug/examples/relay cpp.cdr rust.cdr
python -I demo_reading.py verify rust.cdr
```

The final line prints:

```text
PASS: Python -> C++ -> Rust -> Python; one shared Header identity
```

These file codecs test values and wire interoperability; they are not SDK pub/sub
or a coordinator demo. C++ and Rust construct/assign the dependency-owned Header.
Python checks the final value and exact Header class. The expected value changes
from `2.5` to `4.5`, the label becomes `sensor/cpp/rust`, and the source stamp
`17s + 23ns` and frame `map` remain unchanged.

The full guide with embedded definitions and language examples is
[External message projects](../../docs/development/messages-external-project.md).
The installed Python provider includes complete dependency schemas and native
source inputs, not compiled message libraries. Do not commit `.cargo/config.toml`:
it contains this environment's installed artifact locations.
