# Develop a built-in message in the repository

Use this tutorial when changing dimOS's own message definitions. Work in a
checkout of the CDR proposal; this is not a main-branch or published-release API.
The separate `packages/dimos-generated` package owns the Python extension.
The root dimOS package consumes it and does not generate it in `setup.py`.

Prepare Python 3.12, uv, a C++ compiler with Python development headers, CMake,
and Rust/Cargo. No ROS installation is required. These are explicit toolchain
prerequisites; message builds do not install OS packages. From the checkout root:

```sh skip
bash scripts/setup_message_codegen.sh
export CMAKE_PREFIX_PATH="$PWD/build/message-codegen/install"
uv sync --group tests --frozen
```

The setup script downloads and hash-checks Fast CDR 2.4.0, then installs it into
this checkout's ignored build directory. `uv sync` uses the local packages in
`[tool.uv.sources]`. **Plain `pip install .` at the dimOS root does not read those
uv sources.** It needs matching `dimos-generated` and `dimos-message-build`
distributions available to pip; it is not a substitute for this checkout setup.

## Define and rebuild one message

Create `dimos/message_codegen/schemas/dimos_msgs/msg/DeviceReading.msg`:

<!-- builtin-message: dimos_msgs/msg/DeviceReading -->
```text
std_msgs/Header header
uint32 sequence
float64 value
string label "sensor"
```

The directory supplies `dimos_msgs/msg/DeviceReading`. The vendored `.msg` is the
source of truth; do not edit generated bindings. Standard message inputs are
vendored separately and intentionally pinned; avoid editing a standard Header
just to add an application-specific field.

Build the independently packaged built-ins, then install the resulting wheel:

```sh skip
uv build packages/dimos-generated --python .venv/bin/python \
  --out-dir build/message-codegen/ux-wheelhouse
uv pip install --python .venv/bin/python --reinstall --no-deps \
  build/message-codegen/ux-wheelhouse/dimos_generated-*.whl
```

Keep that output directory to one compatible built-in wheel, or pass the exact
wheel filename. This invokes generation and the Python extension build; it does
not rebuild the dimOS robot runtime. Restart running Python processes after a
native package rebuild. Import and check the new value:

<!-- builtin-value-check -->
```python skip
from dimos_generated.dimos_msgs.msg import DeviceReading
from dimos_generated.std_msgs.msg import Header

reading = DeviceReading(header=Header(frame_id="sensor"), value=20.5)
assert type(reading.header) is Header
assert DeviceReading.decode(reading.encode()).value == 20.5
```

Run the package and runtime regressions appropriate to your changed consumers:

```sh skip
.venv/bin/python -m pytest dimos/message_codegen/test_definitions.py \
  dimos/message_codegen/test_stubs.py
```

Use the active interpreter directly after installing your rebuilt wheel; a
subsequent `uv run` may resync the local-source dependency from its cache.
The new built-in wheel must be installed before testing a runtime module that
imports the new type. In the runtime-cutover layer, use the same
`Module`/`In[DeviceReading]`/`Out[DeviceReading]` API as other generated messages.
Do not use `dimos build` at the repository root for this workflow: that command
builds an **external message project**, not all built-in messages.

## Native artifacts and CI distribution

Build CMake headers/schemas and the Rust crate with the package's configured
version:

```sh skip
bash scripts/package_messages.sh
```

Artifacts land in `build/message-codegen/release/dist/`: a
`dimos-messages-cmake-0.1.0.tar.gz` and `dimos-generated-messages-0.1.0.crate` for
the current proposal version. CMake consumers use `dimos_generated::messages`;
Rust consumers use the `dimos-generated-messages` dependency. Native module
applications additionally need the transport SDK and its native dependencies;
a message wheel alone does not supply an installed SDK.

CI builds the Python wheel/sdist and native artifacts and verifies consumers.
Ordinary users install matching prebuilt artifacts; they never need to run this
maintainer generation workflow. Changes to wire layout require matching rebuilt
producers and typed consumers. Version changes must update dependency pins
consistently; package registry publication is a separate release operation.

See [message contracts](/docs/development/message-reference.md) for shared type
ownership, schema distribution and value/helper boundaries.
