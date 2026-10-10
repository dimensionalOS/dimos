# Use installed dimOS with a custom message project

There are two workflows. dimOS's built-in messages are built in CI and arrive as
prebuilt packages with dimOS. Ordinary users install and import them; they do not
run a generator. `dimos build` is for authors of **external custom messages**.
It builds that message project, not the robot runtime. `dimos bake` remains the
separate command for composing Rust native modules into a host executable.

These packages currently belong to the CDR proposal, not a released main-branch
API. Use matching review wheels for `dimos`, `dimos-generated` and
`dimos-message-build`; `PIP_FIND_LINKS` can point pip at that wheel directory.
Do not assume these proposal versions are available on a public package index.
The `message-authoring-packages` CI artifact also contains matching built-in
native source inputs (`dimos-messages-sources-0.1.0.tar.gz`) and a Rust crate
(`dimos-generated-messages-0.1.0.crate`). These use the message package's version,
independently of the dimOS runtime version. The C++ archive is **not a prebuilt
SDK**: the shared native build core still generates and compiles ROSIDL typesupport
and its pinned support libraries. C++ distribution is source-only in this proposal;
prebuilt native SDK archives are out of scope. Python wheel installation does not
perform this native build.

Read the [accepted native-library limitations](/docs/development/message-limitations.md) before using
these packages: malformed inputs may be accepted and bounded Python messages are
unsupported. These provisional non-blockers are not proof of parser safety.

## Prepare matching review packages

Start in your own project directory, outside the dimOS checkout. This workflow
requires the complete runtime-cutover proposal. Download **matching HEAD** review
artifacts from the draft stack's Actions runs; these packages are not assumed to
exist on PyPI. The nonpublishing wheel matrix uploads `checked-wheels-<os>-<arch>`;
choose exactly one compatible runtime wheel and built-in wheel for your Python
and platform. `checked-message-backend` supplies the backend wheel. Put those
three wheels in `review/wheels/`, then:

```sh skip
python -m venv .venv
. .venv/bin/activate
export PIP_FIND_LINKS="$(realpath review/wheels)"
pip install review/wheels/dimos-*.whl review/wheels/dimos_generated-*.whl \
  review/wheels/dimos_message_build-*.whl
```

Pip installs their declared dependencies. An offline installation additionally
needs those dependency wheels in a prepared wheelhouse and `--no-index`.
Direct wheel filenames select the proposal builds; an unqualified `pip install
dimos` may select a released package with a different API. Pip does not consult
`[tool.uv.sources]` from a dimOS checkout.

Python message source builds generate ordinary Python classes and use setuptools;
they require no C++ compiler, Python development headers, CMake or Fast CDR.
Installed messages use NumPy and pinned rosbags 0.11.0 for CDR. A three-language
application additionally needs CMake, Rust/Cargo, Fast CDR and the native SDK.
The currently verified coordinator example runs on Linux x86_64.

## Prepare native consumers once

The main CI run's `external-message-project` artifact contains both
`examples/message-project/sdk/` (Rust SDK sources) and `build/native-sdk/` (the
installed C++ SDK). Download it from the same HEAD as your packages:

```sh skip
gh run download "$NATIVE_RUN" --repo dimensionalOS/dimos \
  --name external-message-project --dir review/native
cp -R review/native/examples/message-project/sdk .
```

Set `NATIVE_RUN` to that verified successful main-CI run ID. GitHub artifact access
requires your existing account access; no credentials are created by this
workflow. Native C++ additionally requires Fast CDR 2.4.0, Zenoh C/C++ >=1.9.0
with unstable API enabled, raw LCM development files discoverable by pkg-config,
and threads support. PFR/JSON headers are included in the installed SDK. Prepare
those dependencies explicitly; the message wheel does not provide them.

With `FASTCDR_PREFIX`, `ZENOHC_PREFIX` and `ZENOHCXX_PREFIX` set to those prepared
installations, configure the environment:

```sh skip
export CMAKE_PREFIX_PATH="$(realpath review/native/build/native-sdk):$FASTCDR_PREFIX:$ZENOHC_PREFIX:$ZENOHCXX_PREFIX"
pkg-config --exists lcm
```

For libraries outside default loader locations, also configure your environment's
library search paths. Cargo dependencies must be cached before `--offline` works.
The SDK does not hide these toolchain contracts or install OS packages.

## Define one message

Create `interfaces/story_msgs/msg/DeviceReading.msg`:

<!-- source: examples/message-project/interfaces/story_msgs/msg/DeviceReading.msg -->
```text
std_msgs/Header header
uint32 sequence
float64 value
string label "sensor"
```

The directory supplies the identity `story_msgs/msg/DeviceReading`. `Header`
comes from the built-in package. The custom package imports that existing type
in all three languages; it does not regenerate another Header.

Add `pyproject.toml` once:

<!-- source: examples/message-project/pyproject.toml -->
```toml
[build-system]
requires = ["dimos-message-build==0.1.0"]
build-backend = "dimos_message_build.backend"

[project]
name = "story-messages"
version = "0.1.0"

[tool.dimos.messages]
languages = ["python", "cpp", "rust"]
```

The module name defaults to `story_messages`, derived from the project name.
All `.msg` files under `interfaces/<package>/msg/` are discovered automatically.
The default dependency is the compatible `dimos_generated` message package at
version `0.1.0`; additional packages require explicit exact versions under
`[tool.dimos.messages.dependencies]`. Conflicting owners, definitions, versions
or binding ABIs are errors. Runtime never downloads schemas or invokes a native toolchain. On first use,
rosbags initializes native Python classes/codecs in memory from the installed
schemas; there is no generated-file write or constructor/class postprocessing.

## Build or install

For a Python source checkout, this is sufficient:

```sh skip
pip install .
```

The normal PEP 517 backend generates and packages Python source only, even when
the project lists C++ and Rust. Build isolation installs the lightweight backend,
setuptools/wheel and pinned message dependencies; it does not install dimOS or
build the robot runtime. No message-native compiler is needed. Python installation
resolves NumPy and rosbags dependencies normally; it performs no OS installation
or native compilation at import. Rosbags initializes its Python classes/codecs
in memory from the installed schemas.

To build all configured language artifacts instead of installing Python:

```sh skip
dimos build
```

Outputs go to `dist/`, with local generated/build files in `build/dimos/`.
`build/dimos/artifacts.json` records the wheel, CMake prefix, Cargo manifest and
schema paths. Unchanged generation inputs are reused; edited, deleted and renamed
messages invalidate the generated package. CMake and Cargo perform native builds.

```sh skip
dimos build --language cpp       # select one output language
dimos build --install            # also install Python into the active virtualenv
dimos build --offline            # require cached native sources and Cargo dependencies
```

Use pip's `--no-index --find-links` with a prepared wheelhouse for offline Python
builds. C++ message source preparation supports Linux and macOS and requires a C/C++
compiler, CMake and `dimos-message-build[native]` in the active environment.
It builds pinned upstream ROSIDL/FastRTPS/Fast CDR support in a writable local
cache; it does not install ROS system packages. The first online native build
fetches those explicitly selected source dependencies; `--offline` fails if they
are absent. Rust selection requires Cargo and its dependency cache. Python-only builds need neither native
toolchain. Missing prerequisites fail clearly without installing system tools.

Editable installation is also supported:

```sh skip
pip install -e .
# After changing a .msg, rebuild explicitly, then restart running Python processes:
pip install -e .
```

An editable install is not import-time compilation or native-code hot reload.
Consumers installing a matching wheel need no compiler:

```sh skip
pip install --only-binary=:all: dist/story_messages-*.whl
```

## Python: a real module input and output

`demo_modules.py` receives `raw`, adds one, and publishes `reading`:

<!-- source: examples/message-project/demo_modules.py -->
```python skip
from story_messages.story_msgs.msg import DeviceReading

from dimos.core.module import Module
from dimos.core.stream import In, Out


class ReadingProcessor(Module):
    raw: In[DeviceReading]
    reading: Out[DeviceReading]

    async def handle_raw(self, message: DeviceReading) -> None:
        self.reading.publish(
            DeviceReading(
                header=message.header,
                sequence=message.sequence,
                value=message.value + 1,
                label=message.label,
            )
        )
```

Automatic `handle_<input>` handlers are **async**. No manual CDR calls or
`PYTHONPATH` changes are required after installation. These are native rosbags
dataclasses: this constructor reuses the Header object and its dependency-owned
Python type. It changes only the new reading's value. Use `copy.deepcopy` when
your module needs an independently mutable nested message. Python and Rust
constructors require every field explicitly; `.msg` defaults are not injected.
Numeric Python array fields use NumPy arrays with the declared dtype.

## C++: use the installed message and SDK packages

`cpp/processor.cpp` receives `reading` and publishes `processed`:

<!-- source: examples/message-project/cpp/processor.cpp -->
```cpp
#include <dimos/native.hpp>
#include <story_msgs/msg/device_reading.hpp>

using namespace dimos::native;
using Reading = story_msgs::msg::DeviceReading;

class Processor : public Module {
    Output<Reading> processed_;
public:
    void build(Builder& builder, Config&) override {
        processed_ = builder.output<Reading>("processed");
        builder.input<Reading>("reading", &Processor::process, this);
    }
    void process(const Reading& input) {
        auto output = input;
        output.value += 1;
        processed_.publish(output);
    }
};

int main() { run_with_transport<Processor>(); }
```

The consumer's CMake file declares packages, not generated include paths:

<!-- source: examples/message-project/cpp/CMakeLists.txt -->
```cmake
cmake_minimum_required(VERSION 3.20)
project(reading_processor LANGUAGES CXX)
find_package(dimos_native CONFIG REQUIRED)
find_package(story_messages CONFIG REQUIRED)
add_executable(processor processor.cpp)
target_link_libraries(processor PRIVATE dimos_native::dimos_native story_messages::messages)
```

Prepare the SDK and its pinned Zenoh/raw-LCM dependencies once. Its installation
exports `dimos_native::dimos_native` and propagates required includes/libraries.
`dimos build` emits a CMake toolchain file locating this project's message package
and its dependencies, including the Python resources used by upstream ament
CMake configuration. This file contains absolute local paths; rerun `dimos build`
after moving the environment. The example's standard CMake preset uses that file:

<!-- source: examples/message-project/cpp/CMakePresets.json -->
```json
{
  "version": 3,
  "configurePresets": [
    {
      "name": "dimos",
      "generator": "Unix Makefiles",
      "binaryDir": "${sourceDir}/../build/cpp",
      "toolchainFile": "${sourceDir}/../build/dimos/toolchain.cmake"
    }
  ],
  "buildPresets": [
    {
      "name": "dimos",
      "configurePreset": "dimos",
      "jobs": 2
    }
  ]
}
```

With the SDK/Fast CDR prefixes supplied by your development environment:

```sh skip
cd cpp
cmake --preset dimos
cmake --build --preset dimos
cd ..
```

## Rust: typed SDK ports, not a raw-byte example

`rust/src/main.rs` receives `processed` and publishes `checked`:

<!-- source: examples/message-project/rust/src/main.rs -->
```rust
use dimos_module::{Input, Module, Output, cdr, run_with_transport};
use story_messages_messages::story_msgs::msg::device_reading::DeviceReading;

#[derive(Module)]
struct Processor {
    #[input(decode = cdr::decode)]
    processed: Input<DeviceReading>,
    #[output(encode = cdr::encode)]
    checked: Output<DeviceReading>,
}

impl Processor {
    async fn handle_processed(&mut self, mut message: DeviceReading) {
        message.value += 1.0;
        if let Err(error) = self.checked.publish(&message).await {
            tracing::error!(%error, "publish failed");
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<Processor>().await;
}
```

The generated crate shares the built-in package's codec trait, so these SDK
`cdr::encode/decode` adapters accept the custom type directly. Package locations
belong in Cargo configuration, not application source:

<!-- source: examples/message-project/rust/Cargo.toml -->
```toml
[package]
name = "reading-processor"
version = "0.1.0"
edition = "2024"

[workspace]

[dependencies]
dimos-module = { path = "../sdk/rust/dimos-module" }
story-messages-messages = { path = "../build/dimos/cargo-packages/story_messages" }
tokio = { version = "1", features = ["rt-multi-thread", "macros"] }
tracing = "0.1"

[patch.crates-io]
dimos-generated-messages = { path = "../build/dimos/cargo-packages/dimos_generated" }
```

Here `sdk/` is an unpacked SDK review artifact. `build/dimos/cargo-packages/`
contains the custom crate and its message dependencies, each with a single owner.
The patch ensures the SDK resolves the same built-in crate as the custom message.
This local source bundle works before registry publication.

```sh skip
cargo build --manifest-path rust/Cargo.toml --offline
```

Rust dependencies must be cached for `--offline`; otherwise perform an explicit
normal Cargo dependency setup first. No ROS installation is required for either
native language. CDR/schema compatibility is not a ROS node build integration.

## Connect and run the three languages

Declare the native executables' ports in Python:

```python skip
from dimos.core.native_module import NativeModule
from dimos.core.stream import In, Out
from story_messages.story_msgs.msg import DeviceReading

class CppProcessor(NativeModule):
    reading: In[DeviceReading]
    processed: Out[DeviceReading]

class RustProcessor(NativeModule):
    processed: In[DeviceReading]
    checked: Out[DeviceReading]
```

The coordinator supplies topics and transport configuration to native workers.
For the finite acceptance example, `Exchange` publishes one input and checks the
reply; the module chain itself uses the normal blueprint API:

```python skip
from dimos.core.coordination.blueprints import autoconnect

application = autoconnect(
    Exchange.blueprint(),
    ReadingProcessor.blueprint(),
    CppProcessor.blueprint(executable=str(cpp.resolve()), stdin_config=True),
    RustProcessor.blueprint(executable=str(rust.resolve()), stdin_config=True),
)
```

The snippets above show the module API. The finite `Exchange` harness is part of
[the complete example](/examples/message-project); copy that example directory
from the same proposal HEAD if you want to run this exact acceptance command.
The message wheel alone does not install the harness, native SDK or CMake consumer
project. After preparing the SDK, building both processors and installing the
custom Python wheel, run from the example directory:

```sh skip
python demo_blueprint.py --transport zenoh
```

It starts a loopback-only Zenoh router, runs actual Python and native workers,
validates the complete returned CDR value, and stops all owned processes:

```text
PASS: 20.5 -> Python 21.5 -> C++ 22.5 -> Rust 23.5; Header and sequence preserved
```

`--transport lcm` uses the same modules on a multicast-configured host. The harness
reports system tuning requirements without applying them. Host multicast and
hardware acceptance remain separate from the offline Zenoh check.

The source files are in [the complete example](/examples/message-project).
CI checks that the embedded file blocks match those sources, compiles the native
processors and runs the coordinator example. Installation/coordinator blocks use
`skip` in the generic Markdown runner because dedicated isolated/native tests own
their environments and cleanup; they are not substitute pseudocode.

## Distribute, record and evolve

Distribute the custom wheel/sdist, CMake archive and Cargo source bundle together
with matching dependency versions. Local installation does not require publishing
to a registry. Standard types retain their original Python class, C++ declaration
and Rust crate identity across package boundaries.

Message providers export owned types plus the full schema closure. MCAP embeds
`cdr` bytes and complete `ros2msg` schemas, so a viewer does not need your custom
package installed. Typed replay needs the matching package. Source timestamps
remain `header.stamp.sec`/`nanosec`; helpers handle geometry and image operations
outside generated value types.

Changing a wire layout requires rebuilding and distributing matching packages to
producers and typed consumers. This proposal intentionally breaks the old LCM
message API/wire format. LCM remains a transport; mixed old/new typed deployments
and transparent legacy-recording decoding are not supported.

For the other workflow, see [built-in development](/docs/development/messages-in-repository.md). Both share [message contracts](/docs/development/message-reference.md).
