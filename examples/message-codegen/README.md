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

## Stage 3: one recording for Foxglove and Rerun

After building the stage-1 demo, install the viewer-check dependencies and run:

```bash
uv pip install --python .venv/bin/python mcap==1.4.0 rerun-sdk==0.32.0 pillow
bash scripts/test_message_mcap.sh
```

Node.js 22 and npm are required for the independent Foxglove decoder check. The
script installs its locked packages with `npm ci`. It writes
`build/message-codegen/viewers/demo.mcap`: 30 frames each of raw RGB Image, PNG
CompressedImage, PoseStamped, TFMessage, and nested custom Telemetry. Source
timestamps start at 2023-11-14 22:13:20 UTC and advance by 100 ms. Reception
timestamps are 1 ms later. Each channel contains ordinary encapsulated CDR;
the MCAP chunks use Zstandard compression.

The checks read the embedded definitions using Foxglove's official libraries
and the native Rerun 0.32.0 importer. Rerun must produce image, encoded-image,
pose, and transform components plus a structured custom telemetry column. It
must import all 30 rows per topic. This catches unresolved schema dependencies
that can otherwise silently omit custom data. CI uploads the MCAP, converted
RRD, and decoder evidence as `message-mcap-evidence`.

To inspect the file in Rerun directly:

```bash
RERUN_ANALYTICS_ENABLED=false .venv/bin/rerun build/message-codegen/viewers/demo.mcap
```

For a browser-hosted viewer, add `--serve-web --bind 127.0.0.1 --port auto` and
open the URL printed by Rerun. No ROS or `demo_msgs` installation is needed by
the viewer. Raw and compressed images should show the same changing gradient.
Select `/robot/pose` in Streams to inspect the map-frame position, and select
`/telemetry` to inspect the nested message fields. The CLI conversion explicitly
enables `ros2msg` and `ros2_reflection`; the saved `demo.rrd` is also available
for examining these decoder results.

In Foxglove, sign in, open the **same** local `demo.mcap`, add an Image panel for
`/camera/image`, a 3D panel for `/robot/pose` with fixed frame `map`, and a Raw
Messages panel for `/telemetry`. Scrub from first to last frame: sequence should
change from 0 to 29 and `reading.temperature` from 20.0 to 22.9. The compressed
image topic should display the same pixels. Foxglove's web app currently
requires authentication; library decoding is automated, but application UI
acceptance remains pending until an authenticated viewer is available.

Stop the Rerun process with Ctrl-C and close its browser tab. The script starts
no persistent services. Remove `build/message-codegen/viewers/` to clean up
recordings and `examples/message-codegen/viewer-checker/node_modules/` to clean
up the Node dependencies.

## Runtime cutover foundations

The generated set also includes the in-tree `dimos_msgs` definitions. To inspect
weighted segments and trajectory duration fields without a running robot:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_message_helpers.py
```

The source timestamp survives CDR at full nanosecond precision. Segment endpoints
and weights are explicit fields, and controller-facing floating seconds are an
explicit helper conversion from ROS Duration. The generated classes carry data
and codecs; helpers do not add methods or compatibility properties to them.
This is preparation for stage 4, whose runtime consumer cutover is still in
progress.

The typed transport demo exercises DimOS's actual transport classes. It sends a
custom weighted-segment message and a 921,600-byte RGB image through LCM, two
separate Zenoh sessions connected by loopback TCP, and CPU shared memory:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_pubsub.py
```

Use `--backend lcm`, `--backend zenoh`, or `--backend shm` to select one path.
The demo chooses temporary ports and topics and closes its sessions, sockets,
threads, and shared-memory handles on exit. It prints the received custom field,
pixel count, and exact source timestamp. This is a transport demonstration;
the complete three-language blueprint and live viewer cutover remain stage-4
acceptance work. LCM limits its physical channel name, including the type suffix,
to 63 UTF-8 bytes; overlong names fail explicitly before sending.

### Browser schema decoding during the runtime cutover

After building the generated Python module, regenerate the shared vectors:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python -m dimos.web.relay_bridge.gen_cdr_fixtures
cd web
deno install
cd sdk
deno task fixture --host 127.0.0.1
```

Open `http://127.0.0.1:5174/demo_cdr.html`. Select `custom_segments` to inspect the
nested endpoints, weight, and exact timestamp. Select `image` to see the decoded
pixels, and switch byte order to verify both generated encodings. Each selection
uses the SDK's actual decoder registry and the complete schema from its manifest
record. The viewer has no generated message bindings. Stop Vite with Ctrl-C.
This is a fixture-based browser demonstration; live three-language blueprint
integration remains a separate acceptance gate.

Automated checks for this part of the cutover:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/pytest \
  dimos/core/test_cdr_transport.py \
  dimos/protocol/pubsub/impl/webrtc/test_transport.py \
  dimos/web/test_cdr_codec.py dimos/web/test_codecs.py dimos/web/test_cockpit.py \
  dimos/web/relay_bridge/test_map_codecs.py \
  dimos/web/relay_bridge/test_costmap_encoding.py \
  dimos/msgs/test_generated_views.py -o addopts='' -q
cd web/sdk
deno task test
deno task check
cd ../cockpit
deno task test
deno task check
```

Generated messages keep their ROS fields; conversion operations live in helpers.
For example, image arrays require an explicit encoding and header:

```python
from dimos.msgs.image import image_from_array, image_view
from dimos.msgs.time import header_now

message = image_from_array(pixels, encoding="rgb8", header=header_now("camera"))
borrowed = image_view(message)  # read-only, retains its owner; honors row padding and byte order
editable = borrowed.copy()     # an explicit independent mutable copy
```

`image_from_array` copies the supplied pixels and validates their dtype and shape.
It accepts non-contiguous inputs. `occupancy_view` provides the corresponding
read-only grid view using `info.width` and `info.height`. Pose consumers access
`message.pose.position`; planar heading is `yaw(message.pose.orientation)` from
`dimos.msgs.geometry`. Creating a zero velocity command is simply generated
`Twist()`, including in watchdog and stop paths.

### Native SDK relay during the runtime cutover

With the C++ SDK's LCM, Zenoh C/C++, JSON, and PFR build dependencies installed,
build the generated package and native relays:

```bash
bash scripts/setup_message_codegen.sh
bash scripts/install_native_messages.sh
cmake -S native/cpp -B build/native-cpp -DDIMOS_NATIVE_BUILD_TESTS=ON \
  -DCMAKE_PREFIX_PATH="$PWD/build/message-codegen/install"
cmake --build build/native-cpp -j 2
ctest --test-dir build/native-cpp --output-on-failure
cmake -S examples/native-modules/cpp -B build/native-cpp-examples \
  -DCMAKE_PREFIX_PATH="$PWD/build/message-codegen/install"
cmake --build build/native-cpp-examples -j 2
cargo build -p dimos-native-module-examples
cargo test -p dimos-module --lib
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_native.py
```

The generated Python extension comes from the initial generation/build demo.
The `native` job in `.github/workflows/ci.yml` lists the pinned Zenoh archives
and system dependencies. Add their installation prefix to `CMAKE_PREFIX_PATH`
and their pkg-config directory to `PKG_CONFIG_PATH` when installed locally.

The demo starts actual SDK subprocesses, supplies their stdin launch configuration,
and sends a custom `LineSegments3D` and a 640×480 RGB image through
Python → C++ → Rust → Python, first over LCM and then over loopback Zenoh TCP.
The segment weight changes from 4 to 5 to 6. The full image and exact source
nanoseconds must survive both relays. LCM fragments the image along each hop.
Use `--backend lcm` or `--backend zenoh` for one transport. Temporary topics and
ports isolate runs; subprocesses and sessions close automatically, including on
failure. Native logs go to `build/message-codegen/demo/evidence/native-*.log`.
This exercises the SDK launch protocol; the full coordinator blueprint and live
Rerun integration are still separate acceptance work.

The in-tree Rust message dependency generates only inside Cargo's `OUT_DIR`.
Its source package bundles the parser, generator, schemas, and licenses. Cargo
requires Python 3.10+ during the build (`DIMOS_CODEGEN_PYTHON` can select it);
the resulting Rust binaries need neither Python nor ROS. Verify the actual source
archive with `cargo package -p dimos-generated-messages --allow-dirty --offline`.

### Point-cloud layouts

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_pointcloud.py
```

This displays an organized cloud with field padding, row padding, and a two-value
`tags` field, in both byte orders. The helper borrows a read-only structured view;
`pointcloud_xyz` explicitly copies XYZ coordinates. The demo edits a separate
copy and checks that the borrowed source remains unchanged. No processes or
viewer windows remain after it exits.

### C++ lidar message construction

The native C++ test build also produces a hardware-free demonstration of the
production lidar cloud builder:

```bash
cmake --build build/native-cpp -j 2
build/native-cpp/tests/dimos_lidar_cdr_demo
```

It prints the decoded XYZI point count, frame, timestamp, and intensity for both
CDR byte orders. It also verifies negative timestamp normalization and rejects
invalid timestamps and negative cloud sizes. The executable exits on its own.

### External messages through native SDK ports

After the external packaging demo and the native SDK dependency setup above:

```bash
bash scripts/test_message_packages.sh
bash scripts/test_external_native_messages.sh
```

The first command adds `application_note` to a local copy of `Telemetry.msg` and
builds separate Python/CMake/Cargo packages. The second builds C++ and Rust SDK
modules against those packages and exchanges the extended message through
Python → C++ → Rust → Python over LCM and Zenoh. The displayed note becomes
`added-locally/cpp-native/rust-native`; every other field and the source timestamp
must remain unchanged. It uses the generated Rust `Telemetry::encode` and
`Telemetry::decode` directly in port declarations. No SDK-specific codec adapter
or change to the built-in definition registry is needed.

Generated Rust codec entry points return `std::io::Result`: invalid values at
encode time report `InvalidInput`, while malformed CDR reports `InvalidData`.
The external consumer test checks both errors against the SDK's function types.
Subprocesses and sessions close on exit. Logs and visible results are under
`build/message-codegen/external-native/evidence`.

### Python/Rust TF through the module coordinator

Build the Rust examples, then run the TF blueprint using either backend:

```bash
cargo build --release -p dimos-native-module-examples
DIMOS_TRANSPORT=zenoh PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/native-modules/rust_tf.py
# Repeat with DIMOS_TRANSPORT=lcm on a host configured for multicast.
```

The Python worker publishes `a → b → c` using generated `TransformStamped`
messages, and the Rust broadcaster publishes `c → d`. The Rust listener prints
`x=1.5` and changing `y=cos(t), z=sin(t)` for the composed `a → d` transform.
The pinned ROS quaternion definition defaults `w=1`; generated types preserve
that identity rotation, and this example also sets it explicitly. Stop with Ctrl-C; the coordinator
stops both native processes and its Python workers.

The automated check runs the same blueprint, verifies four changing composed
transforms per backend, and checks clean shutdown. It requires the normal DimOS
runtime dependencies (including Open3D). Pytest disables interactive host system
configuration, and the LCM run reserves a temporary local UDP port:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/pytest \
  examples/native-modules/test_rust_tf.py -m native_e2e -o addopts='' -v
```

### Three-language module coordinator blueprint

After building the native relays above, run the complete worker-based graph:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_blueprint.py --transport zenoh
# On a multicast-configured host:
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_blueprint.py --transport lcm
```

```text
Python worker → C++ native worker → Rust native worker → Python worker
   weight n          n + 1               n + 2          validate fields
   RGB image  ──────────────────────────────────────── validate all pixels
```

Each of five samples changes the custom weight, endpoint, image pixels, and exact
source timestamp. The final Python worker validates the full encoded messages;
the terminal reports each result. `--samples N` changes the sample count. Native
processes and workers stop automatically after the finite run, including failures.

The Zenoh demo starts a temporary loopback router and connects every worker to
its explicit endpoint, with multicast discovery disabled. The topics and module
names have a unique namespace. The LCM run uses `LCM_DEFAULT_URL`, so set it to an
isolated multicast port when other local blueprints are running. As with the TF
demo, normal DimOS runtime dependencies are required. Automated checks choose a
temporary LCM port and run the same command on each transport:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/pytest \
  examples/message-codegen/test_blueprint.py -m native_e2e -o addopts='' -v
```

This finite demo exercises coordinator deployment, remapping, native startup,
and generated streams. Live Rerun/browser integration remains a later addition.

### Generated TF values and geometry helpers

The live TF buffer and recorded-stream lookup return generated
`geometry_msgs/msg/TransformStamped` values. Read the parent frame and timestamp
from `value.header`, the child from `value.child_frame_id`, and the translation
and quaternion from `value.transform`. The separate helpers in
`dimos.msgs.geometry` compose, invert, or convert those values:

```python
from dimos.msgs.geometry import compose_transforms, inverse_transform, pose_from_transform

world_to_tool = compose_transforms(world_to_arm, arm_to_tool)
tool_to_world = inverse_transform(world_to_tool)
world_pose = pose_from_transform(world_to_tool)
```

Composition requires matching intermediate frame names and preserves the first
edge's exact source stamp. Inversion swaps frames and preserves the stamp. The
TF time index uses integer nanoseconds and owns snapshots of received transforms;
mutating a received value or a lookup result cannot change buffered history.
Float-second lookup arguments remain for existing timing APIs.

The Python/Rust TF demo above now logs the composed transform from both Python
and Rust. Its automated check verifies both results. Robot-specific publishers
and downstream consumers are being converted to these fields as part of the
remaining runtime cutover.
