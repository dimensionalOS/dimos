# Define and build an external message package

Built-in definitions arrive with the installed dimOS wheel. Custom messages
belong to your own package; building them does not rebuild the dimOS runtime.
Use ordinary Python, CMake and Cargo builds. There is no `dimos build` step.

## Define the message once

Create `interfaces/telemetry_msgs/msg/DeviceReading.msg`:

```text
std_msgs/Header header
uint32 sequence
float64 value
string label
```

Configure `pyproject.toml`:

```toml
[build-system]
requires = ["dimos[message-build]==0.0.14"]
build-backend = "dimos.message_codegen.backend"

[project]
name = "acme-telemetry"
version = "0.1.0"

[tool.dimos.messages]
languages = ["python", "cpp", "rust"]

[tool.dimos.messages.dependencies]
"dimos.messages" = "0.0.14"
```

The custom table is passive configuration. Pip invokes the backend selected by
`[build-system]`; that backend reads the table. CMake and Cargo hooks read the
same project plan, dependency closure and cache. Defaults discover
`interfaces/<package>/msg/*.msg` and derive `acme_telemetry` from the project name.
`source`, `module` and `python-module` are optional overrides. A dependency key
identifies an installed schema provider; its exact version must match the owner.
Here the owner is the main dimOS distribution, not a second copy of Header.

The build extra supplies Python tooling. Source builds use PEP 517 isolation;
provide matching dimOS wheels and their build dependencies to pip. Pip does not
read another checkout's `[tool.uv.sources]`. For offline builds use a complete
wheelhouse with `PIP_NO_INDEX=1` and `PIP_FIND_LINKS=/path/to/wheels`.

## Install and use from Python

```sh
pip install .
# Editable development:
pip install -e .
```

Only Python output is built by these commands, even when C++ and Rust are enabled
in the project. No CMake, Cargo or C++ compiler is invoked. A wheel installs
without a compiler. After changing a definition, reinstall editable output and
restart existing Python processes.

```python
from acme_telemetry.telemetry_msgs.msg import DeviceReading
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header
from dimos_cdrgen.registry import decode, encode

message = DeviceReading(Header(Time(17, 23), "map"), 7, 2.5, "sensor")
restored = decode(encode(message), DeviceReading)
assert restored == message
assert type(restored.header) is Header
```

Python exports aliases/stubs for rosbags values and complete schemas. These are
plain values, not the old rich `dimos.msgs` wrappers or native Python extensions.
Imports do not fetch definitions or invoke project builds. The installed provider
retains full dependency schemas while importing dependency-owned Python classes.

## Configure the native toolchain once

Use the same Python environment with the message-build extra, plus your normal
CMake/C++ and Rust/Cargo toolchains. C++ source preparation downloads only the
pinned support sources declared by `dimos-cdrgen`; `DIMOS_OFFLINE=1` requires the
corresponding cache. No OS packages or compilers are installed by a build hook.
Native builds currently target the host, not cross compilation.

```sh
export CMAKE_PREFIX_PATH="$(python -m dimos.message_codegen.native cmake-prefix)"
python -m dimos.message_codegen.native configure-cargo
```

These export installed SDK locations. The Cargo command writes standard local
patches in `.cargo/config.toml` for the build hook and matching dependency source
artifacts. It generates no message types and preserves unrelated Cargo settings.
Keep that machine-specific file out of Git. Python-only dependency wheels need a
matching Rust source artifact before they can be used by native consumers.

## Build and use from C++

Place this `CMakeLists.txt` beside the pyproject:

```cmake
cmake_minimum_required(VERSION 3.20)
project(acme_consumer LANGUAGES CXX)
find_package(DimosMessages CONFIG REQUIRED)
dimos_message_project()
add_executable(consumer cpp/consumer.cpp)
target_link_libraries(consumer PRIVATE acme_telemetry::messages)
```

The configure hook resolves the declared owners and prepares cached generated
headers/typesupport libraries. The imported target exports the include and link
requirements, including Header's original declaration.

```cpp
#include <telemetry_msgs/msg/device_reading.hpp>
#include <std_msgs/msg/header.hpp>

int main() {
    telemetry_msgs::msg::DeviceReading message;
    std_msgs::msg::Header header;
    header.frame_id = "map";
    message.header = header;
    message.sequence = 7;
    message.value = 2.5;
    message.label = "sensor";
}
```

```sh
cmake -S . -B build/cpp
cmake --build build/cpp
```

## Build and use from Rust

The message crate uses the same native identity with the `-messages` suffix.
Its Cargo version must equal the Python project version. `Cargo.toml` declares
normal language dependencies; matching built-in versions stay explicit:

```toml
[package]
name = "acme-telemetry-messages"
version = "0.1.0"
edition = "2024"
[dependencies]
dimos-generated-messages = "=0.0.14"
serde = { version = "1", features = ["derive"] }
serde-big-array = "=0.5.1"
heapless = { version = "=0.8.0", features = ["serde"] }
[build-dependencies]
dimos-message-build = "=0.1.0"
```

`build.rs` and `src/lib.rs` are small, stable integration hooks:

```rust
// build.rs
fn main() {
    dimos_message_build::generate(".").expect("build messages");
}
```

```rust
// src/lib.rs
include!(concat!(env!("OUT_DIR"), "/messages.rs"));
```

Normal Cargo compilation calls the shared project plan and generates Rust
bindings in `OUT_DIR`. An ordinary consumer imports the resulting value:

```rust
use acme_telemetry_messages::telemetry_msgs::msg::DeviceReading;

fn process(message: &mut DeviceReading) {
    message.value += 1.0;
}
```

```sh
cargo build
```

The short `package::msg::Type` exports are aliases to ros2msg's structs. Imported
fields remain the original dependency crate's types. The backend rejects owner,
version and schema conflicts; C++ and Rust do not generate another Header.
No ROS installation is required. This establishes message/wire compatibility,
not a ROS node build integration.

## Distribute and verify the package

Publish Python wheels and native source artifacts from your own CI. Python
source distributions retain definitions and project metadata so pip can rebuild
without pre-generated files. A project with Cargo hooks also carries those
source inputs in its installed provider for downstream native builds. Schemas
and license resources remain available for complete MCAP schema embedding.
Runtime type-hash routing enforcement is a separate concern.

The executable example in `examples/external-messages` uses these exact hooks.
It sends one CDR value through Python, C++ and Rust file codecs and back, checking
Header construction/assignment and final values. These are explicit codec calls;
SDK pub/sub and coordinator wiring belong to the opt-in transport integration.
Its ordinary build steps are:

```sh
pip install .
python -I demo_reading.py write python.cdr
cmake -S . -B build/cpp && cmake --build build/cpp
build/cpp/telemetry_relay python.cdr cpp.cdr
cargo build --example relay
./target/debug/examples/relay cpp.cdr rust.cdr
python -I demo_reading.py verify rust.cdr
```
