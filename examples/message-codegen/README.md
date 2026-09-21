# A message without an upstream PR

`demo_msgs/msg/Telemetry.msg` contains a standard ROS2 header, a custom nested
reading, a position, defaults, fixed and variable arrays, and a bounded string.
This example generates Python, C++, and Rust from that definition without ROS.
The native processes print the fields they receive and each adds its own hop.

## Build dependencies

Use Python 3.12+, a C++17 compiler, CMake 3.20+, and Rust 1.92+. Install Python
build/test dependencies into the checkout's virtual environment:

```bash
uv venv .venv --python 3.12
uv pip install --python .venv/bin/python pybind11==3.0.1 rosbags==0.11.0 pytest pytest-asyncio pytest-env numpy lcm-dimos-fork
```

Install Fast CDR 2.4.0 into a local build prefix. This setup step downloads source;
message generation and application runtime do not access the network:

```bash
mkdir -p build/message-codegen
curl -fL https://github.com/eProsima/Fast-CDR/archive/refs/tags/v2.4.0.tar.gz \
  -o build/message-codegen/fastcdr.tar.gz
echo '79d8466107dd6b7d1defe961c4aa31735038937cf9dd1175cf6b0da0df2209ab  build/message-codegen/fastcdr.tar.gz' | sha256sum -c -
tar -xzf build/message-codegen/fastcdr.tar.gz -C build/message-codegen
cmake -S build/message-codegen/Fast-CDR-2.4.0 \
  -B build/message-codegen/fastcdr-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/build/message-codegen/install" \
  -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build build/message-codegen/fastcdr-build -j 2
cmake --install build/message-codegen/fastcdr-build
```

## Generate and build all three languages

Run from the repository root:

```bash
.venv/bin/python -m dimos.message_codegen.generate \
  --package-root examples/message-codegen \
  --output build/message-codegen/demo
cmake -S build/message-codegen/demo/cpp -B build/message-codegen/demo/cpp/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/build/message-codegen/install" \
  -DPython_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build/message-codegen/demo/cpp/build -j 2
c++ -std=c++17 -Ibuild/message-codegen/install/include \
  -Ibuild/message-codegen/demo/cpp examples/message-codegen/relay.cpp \
  build/message-codegen/install/lib/libfastcdr.a \
  -o build/message-codegen/demo/cpp-relay
mkdir -p build/message-codegen/demo/rust/src/bin
cp examples/message-codegen/relay.rs build/message-codegen/demo/rust/src/bin/relay.rs
cargo build --manifest-path build/message-codegen/demo/rust/Cargo.toml
```

Cargo resolves build dependencies on the first build. Once cached, use `--offline`
to demonstrate that generation/builds do not fetch message definitions or ROS.

## Run and inspect

```bash
PYTHONPATH=build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_relay.py --build build/message-codegen/demo
PYTHONPATH=build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_conformance.py --build build/message-codegen/demo
.venv/bin/pytest dimos/message_codegen/test_definitions.py --noconftest -o addopts='' -q
```

The relay starts with sequence `40`, label `start`, and hops `[1]`. C++ increments
the sequence and adds hop `2`; Rust increments it again and adds hop `3`. Python
prints sequence `42`, label `start/cpp/rust`, and hops `[1, 2, 3]`. The terminal
also shows the nested temperature, position, and fixed-array values in each native
process. Binary artifacts remain under `build/message-codegen/demo/evidence/`.

The conformance demo prints the nine native encoder/decoder combinations, checks
both byte orders against an independent ROS2 codec, and exercises rejected
payloads. It is a local conformance check, not a replacement for the planned ROS2
Jazzy CI reference job.

No background processes remain after either command. To clean up, remove the
example's ignored `build/message-codegen/demo` directory; removing the broader
`build/message-codegen` directory also removes the native toolchain installation.

## Run the complete stage

The same scripts drive local verification and CI:

```bash
bash scripts/setup_message_codegen.sh
bash scripts/test_message_codegen.sh
```

The second command builds every bundled and demo message, runs the tests and
relay, prints image/point-cloud buffer timings, and exchanges raw LCM payloads
between Python and Rust, including a fragmented 1 MiB payload. It requires local
UDP multicast. Output and payloads are retained in the demo's `evidence/` folder.
The independent Jazzy job builds the same demo definitions with ROS generators
inside a container; ROS is absent from the standalone build and application.

## Python sequence and buffer behavior

Primitive sequences are live: `message.payload.append(7)` and indexed assignment
update the message. Nested message fields are live too. Elements of a sequence of
messages are values: edit a retrieved element and assign it back to persist the
change. This avoids dangling references when the sequence resizes.

`image.data.view()` returns a read-only NumPy view retaining the message owner.
While any view exists, operations that replace nested storage or resize a sequence
on that owner raise `BufferError`. Scalar and in-place element updates remain
possible. `image.data.copy()` returns an independent, writable NumPy array.
The terminal buffer demo reports local borrow/copy/encode/decode medians; these
are reproducible measurements, not platform-independent performance guarantees.

## Current stage scope

This stage provides standalone generation and codecs. DimOS runtime consumers
still use the old types; their coordinated replacement follows in the stack.

## Stage 2: an installed external application

After the stage-1 setup, run:

```bash
uv pip install --python .venv/bin/python setuptools wheel
bash scripts/test_message_packages.sh
```

This copies the demo definitions into an ignored external project, adds
`string application_note "added-locally"` to its own Telemetry definition,
and invokes the same generator with `--package`. It builds a wheel **from the
sdist**, installs the wheel in a fresh virtual environment, installs the CMake
package, and builds a separate Rust consumer from the packaged `.crate` archive.
The terminal prints:

```text
Installed Python package sends new field: added-locally
Installed C++ package received: added-locally
Packaged Rust crate received: added-locally/cpp
Installed Python package receives: added-locally/cpp/rust
```

The application runs with Python's isolated flag and no `PYTHONPATH`. Its wheel
contains its own extension, schema dependency closure, upstream licenses, and
`dimos.messages` provider. It needs neither DimOS nor ROS to encode/decode.
No background services are started. Outputs and the captured terminal demo live
under `build/message-codegen/external-app/`; remove that directory to clean up.

For your own package, use:

```bash
python -m dimos.message_codegen.generate \
  --package-root path/to/interfaces --type my_msgs/msg/Reading \
  --python-module my_robot_messages --version 0.1.0 --package --output build/messages
CMAKE_PREFIX_PATH=/path/to/fastcdr uv build build/messages/python
cmake -S build/messages/cpp -B build/messages/cmake \
  -DDIMOS_BUILD_PYTHON=OFF -DCMAKE_PREFIX_PATH=/path/to/fastcdr \
  -DCMAKE_INSTALL_PREFIX=/path/to/message-prefix
cmake --install build/messages/cmake
cargo package --manifest-path build/messages/rust/Cargo.toml
```

The source package includes the pinned generator and definition inputs; source
builds regenerate locally. Fast CDR must be installed at build time, using the
setup script or an equivalent pinned installation. Wheels statically link it.
Consumers use `find_package(my_robot_messages CONFIG REQUIRED)` and link
`my_robot_messages::messages`, or add the generated Rust crate through Cargo.
Python schema providers are discovered through the standard `dimos.messages`
entry-point group. Definition discovery does not import the native extension;
type discovery validates conflicting qualified definitions before loading types.
Generation, installation from built artifacts, and runtime require no schema
service or runtime download. Dependency setup can use the network; cached Cargo
builds and local wheel installation are exercised offline by the demo.
