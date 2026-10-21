# CDR generation in dimOS

**WIP:** Built-in messages work in standalone examples. CDR transport and application
support are incomplete. Existing code continues to use `dimos.msgs`.

`setup.py` reads the definitions in `packages/dimos-generated` and calls
`dimos-cdrgen==0.1.1`. The wheel contains `dimos.messages`, schemas, and native
source code. The examples use these installed files.

The [cdrgen guides](https://tomcc7.github.io/dimos-cdrgen/) describe its APIs and
supported schemas. `dimos.message_codegen.native_build.write_cmake_toolchain`
adds paths for the dimOS SDK.

## Run the built-in examples

Run all commands below from the repository root. Install `uv` and a C++ compiler
first; the compiler is needed by dimOS's existing A* extension even for a source
installation used only by Python. The examples use Python 3.12.

### Install and run Python

```sh skip
uv venv .venv-msg-review --python 3.12
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install --python .venv-msg-review/bin/python .
.venv-msg-review/bin/python -I examples/message-package/demo_installed_wheel.py
```

`DIMOS_ALLOW_MISSING_COCKPIT=1` is the existing option to install without prepared
web UI bundles. Omit it when building a complete distribution with those assets.
The example imports the installed package, prints the generated Python and
schema/native-source directories, and checks image and custom-message CDR roundtrips.
Expected output includes:

```text
Generated Python messages: <venv>/lib/python3.12/site-packages/dimos/messages
Packaged schemas and native sources: <venv>/lib/python3.12/site-packages/dimos/messages_schemas/package
Messages: 73 types
PASS: sensor_msgs/msg/Image pixels, encoding and source stamp
PASS: dimos_msgs/msg/LineSegments3D geometry, weight and nested type identity
PASS: packaged definitions and C++/Rust build inputs; no compiled message libraries
```

The generated directories are installation outputs, not files to edit. Message
inputs live under `packages/dimos-generated` in the checkout.

### Run C++ and Rust

Install CMake and Rust/Cargo, then reuse the same environment:

```sh skip
uv pip install --python .venv-msg-review/bin/python 'dimos-cdrgen[native]==0.1.1'
.venv-msg-review/bin/python -I examples/message-package/demo_native.py
```

This one command compiles the ordinary CMake and Cargo consumers in
`examples/message-package/native`. Python writes an installed built-in Header,
C++ appends `/cpp` to its frame ID, Rust appends `/rust`, and Python verifies
`map/cpp/rust` with the timestamp unchanged. Expected final output:

```text
PASS: built-in Header Python -> C++ -> Rust -> Python
```

The script also prints the installed native-source directory. Temporary example
files are cleaned up. Native support builds use `~/.cache/dimos/messages`;
`DIMOS_MESSAGE_NATIVE_CACHE` overrides that location. Set `CARGO_TARGET_DIR` to
retain Rust build artifacts. Cold builds download pinned dependencies;
`DIMOS_OFFLINE=1` requires populated native and Cargo caches.

### What installation does

Definitions → setuptools calls cdrgen → Python/schema/native-source
artifacts in the dimOS wheel → installed consumers. Source installation prepares
verified definitions and generates artifacts. Wheel installation only installs
those artifacts. The sdist includes definitions needed to rebuild the wheel.

Installation does not compile or test the C++/Rust message consumers. Running
`demo_native.py` performs that verification explicitly. These examples use files,
with no robot connection, transport or SDK pub/sub. No dedicated native-message
CI is required to install or run them.

## Verify the integration

Use a Python environment with the published generator and test tools:

```sh skip
uv pip install 'dimos-cdrgen==0.1.1' numpy build pytest pytest-asyncio pytest-env
python -m pytest --noconftest -o addopts='' -q dimos/message_codegen/test_integration.py
```

These checks cover SDK path discovery and an installed Python message consumer.
Native message generation/compilation is not run by CI. To manually verify the
installed built-in messages across Python, C++ and Rust, run the
[built-in example above](#run-c-and-rust).
Generic generator regression tests belong to the cdrgen repository.
