# Develop a built-in message in the repository

Use this tutorial when changing dimOS's own message definitions. Work in a
checkout of the CDR proposal; this is not a main-branch or published-release API.
The separate `packages/dimos-generated` package owns checked-in generated Python source.
The root dimOS package consumes it and does not generate it in `setup.py`.

Prepare Python 3.12 and uv for installation. Message maintainers additionally
need Ruff 0.14.3 and rustfmt for explicit generation. Native applications need
their C++/Rust toolchains. No ROS installation is required. These are explicit toolchain
prerequisites; message builds do not install OS packages. From the checkout root:

```sh skip
uv sync --group tests --group message-codegen --frozen
```

`uv sync` installs the built-in message package as editable Python source from
`packages/dimos-generated/src`; no message-native extension is built. It consumes
the local packages in
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

Generate source explicitly after editing `.msg`:

```sh skip
uv run python -m scripts.generate_builtin_messages
uv run python -m scripts.generate_builtin_messages --check
```

The checkout's editable dependency uses the regenerated source directly; no
wheel rebuild or message compiler is needed for development. Restart Python
processes after changing generated classes. Editing `.msg` alone does not change
installed classes: the drift check rejects missing, stale or changed outputs
until you explicitly regenerate and commit them. Import and check the new value:

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

In the runtime-cutover layer, use the same
`Module`/`In[DeviceReading]`/`Out[DeviceReading]` API as other generated messages.
Do not use `dimos build` at the repository root for this workflow: that command
builds an **external message project**, not all built-in messages.

The main CI **Built-in message alignment** check fails `ci-complete` if definitions,
package versions, generator/templates, or any checked-in output drift. Python,
C++ headers, Rust build-script/schema inputs and viewer schemas are committed; the check independently regenerates
in a temporary directory and includes new/untracked and deleted files. Prepare
Rust 1.92.0 with `rustup toolchain install 1.92.0 --component rustfmt` once.
[The alignment guide](/docs/development/message-alignment.md) gives the small
locked check environment and repair commands; CI does not make bot commits.

## Package artifacts and CI distribution

CI builds the committed generated source as an ordinary compiler-free Python
wheel/sdist; wheel users do not need an editable checkout:

```sh skip
uv build packages/dimos-generated --python .venv/bin/python \
  --out-dir build/message-codegen/ux-wheelhouse
```

## Native artifacts

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
