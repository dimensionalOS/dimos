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

The TF demo's fixed `b → c` edge is published by the production
`StaticTfPublisher` in its own worker. This exercises periodic generated
`TFMessage` publication alongside the dynamic Python and Rust edges. Static mount
implementations return `list[TransformStamped]`; `frames_to_edge_transforms`
constructs them from `(child, parent, xyz, fixed-axis-rpy)` entries. A `None` parent
marks the root and produces no edge. The publisher supplies current integer
nanosecond stamps on every cycle.

### Camera calibration and webcam messages

`dimos.msgs.camera_info` constructs generated `CameraInfo` values from pinhole
intrinsics, field of view, or a ROS calibration YAML file. Construction requires
an explicit `Header`; message defaults do not invent a capture time. YAML loading
checks the matrix dimensions and finite values. The generated fields are lowercase
`k`, `d`, `r`, and `p`. `intrinsic_matrix(info)` returns an independent mutable
NumPy copy.

The existing helper demo now loads the packaged Go2 calibration, encodes and
decodes it with CDR, and prints its dimensions, distortion model, matrix,
coefficients, and exact source timestamp:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_message_helpers.py
```

The webcam path emits generated RGB images; stereo slicing happens on pixels
before message construction. Camera metadata and both TF edges share one exact
stamp without mutating the configured calibration or mount. Hardware-free tests
feed known BGR pixels into the capture boundary and verify the decoded RGB crops.

### Simulation lidar shared memory (no engine required)

Run the actual simulation lidar producer and consumer in separate processes:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_mujoco_lidar_shm.py
```

The demo prints decoded XYZ coordinates, frame `world`, and the exact timestamp
`1700000000123456789`. It writes generated PointCloud2 CDR bytes through the
production shared-memory buffer and cleans up its regions on exit. It does not
launch MuJoCo or require robot hardware. Live simulation and Go2 pose/TF consumers
remain separate acceptance checks.

### Robot pose and TF conversion (no hardware required)

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_robot_tf.py
```

This runs the WebRTC device-pose converter and Go2's TF-chain builder on a sample
packet, round-trips generated TFMessage through CDR, and prints the three edges
and their composed camera frame. The robot-local frames receive `robot0/`; the
source parent stays `odom`. Timestamp `1746565669448350564` survives unchanged.
The live WebRTC connection deliberately supplies a host-arrival header instead,
matching its previous timing policy with integer-nanosecond precision.

### Generated-message storage and module replay

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_storage_replay.py
```

The demo records custom weighted segments to SQLite and a ROS2-profile MCAP,
reopens both by their stored type/schema, and replays SQLite through a real
module's `Out` port. It prints weights `4, 5, 6` and consecutive source nanoseconds
starting at `1700000000123456789`. Temporary recordings are removed automatically.
This checks Python storage and replay.

### Record an external custom message with the Rust recorder

First complete the external-package demo above, including its locally added
`application_note` field. Build the recorder once, then record that external type
over both transports using the same executable:

```bash
cargo build -p dimos-memory-recorder --locked
PYTHONPATH=.:build/message-codegen/demo/cpp/build:build/message-codegen/external-app/venv/lib/python3.12/site-packages \
  .venv/bin/python examples/message-codegen/demo_native_recording.py
```

Adjust the external virtualenv's Python version in the path when necessary.
The demo prints `locally-added-field-0` through `locally-added-field-2` and exact
payload stamps on LCM and Zenoh. It stops each recorder and closes its transport
resources, retaining `external-native-recording-{lcm,zenoh}.mcap` and `.log` files
under `build/message-codegen/demo/evidence/` for inspection. Use `--output` to
choose another artifact directory and `--executable` for a packaged recorder.

With the viewer-checker dependencies installed as described above, independently
decode both recordings from their embedded schemas:

```bash
node examples/message-codegen/viewer-checker/check-native-recording.mjs \
  build/message-codegen/demo/evidence/external-native-recording-lcm.mcap
node examples/message-codegen/viewer-checker/check-native-recording.mjs \
  build/message-codegen/demo/evidence/external-native-recording-zenoh.mcap
```

The Foxglove decoder requires no custom message package. Arbitrary custom payloads
are preserved byte-for-byte; their MCAP publish time uses reception time because
the recorder has no compiled knowledge of their timestamp layout. Recognized
standard stamped messages use exact source nanoseconds. Delete the four retained
MCAP/log files when finished reviewing them. These terminal checks complement
the separate Foxglove and Rerun UI acceptance demos.

### Optional ROS2 bridge (isolated verification)

DimOS generation and runtime remain ROS-free. To check the existing optional
bridge against ROS2 Jazzy after building the standalone Python binding, use the
independent reference container:

```bash
docker build -f .github/docker/message-codegen-reference.Dockerfile \
  -t dimos-cdr-jazzy-reference .github/docker
docker run --rm -v "$PWD:/source:ro" dimos-cdr-jazzy-reference '
  set -e
  source /opt/ros/jazzy/setup.bash
  export PYTHONPATH=/source:/source/build/message-codegen/demo/cpp/build:$PYTHONPATH
  cd /source
  python3 -m pytest -o addopts="" --noconftest --import-mode=importlib -p no:cacheprovider \
    dimos/protocol/pubsub/impl/test_rospubsub_conversion.py \
    dimos/protocol/pubsub/impl/test_rospubsub.py -q
  python3 examples/message-codegen/demo_ros_bridge.py
'
```

The demo prints three poses (`x=0, 1, 2`) and RGB pixels (`[0,128,255]`,
`[1,128,254]`, `[2,128,253]`) received through real ROS publishers/subscribers.
Source nanoseconds `123456789` through `123456791` survive unchanged. Nodes,
timers, subscriptions, and the container are cleaned up on exit. With ROS
already installed and sourced, run the same Python demo directly.

The bridge uses ROS's serialization/type support and generated DimOS CDR codecs.
It requires matching ROS message packages only in the bridge environment; it
does not translate through LCM, rename fields, or load legacy message classes.
The independent Jazzy CI job runs these checks using the binding produced by
the preceding ROS-free build.

### Galaxea feedback without hardware or ROS

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_galaxea_messages.py
```

This feeds synthetic chassis-speed samples into the actual Galaxea connection
callback and subscribes to its generated pose, odometry, and TF output streams.
It prints three changing positions and the `odom -> base_link -> lidar_chassis_left_link`
chain, with exact source nanoseconds on every edge. It requires neither ROS nor
a robot; it cleans up subscriptions and the module on exit. The neighboring
`test_connection_messages.py` also checks joint-state aggregation, command array
splitting, tracking-speed sentinels, and malformed-command rejection.

### Whole-body transport feedback and commands

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_whole_body_transport.py
```

This connects the actual whole-body adapter to synthetic robot-side publishers
and a command subscriber, first over LCM and then through two explicit loopback
Zenoh sessions. It prints feedback positions `[0.25, -0.5]`, commanded positions
`[0.75, -0.25]`, and the hardware-facing WXYZ quaternion. The demo checks command
gains/torques and generated headers, then closes transports, subscriptions, and
sessions. `test_whole_body_transport.py` runs the same exchange automatically.

### G1 whole-body feedback and coordinator joint streams

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_g1_messages.py
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_coordinator_joints.py
```

The G1 demo feeds three synthetic 29-motor snapshots through the actual output
ports. It prints changing joint positions, the converted ROS XYZW orientation,
and consecutive exact source nanoseconds shared by joint and IMU messages. It
does not initialize the Unitree SDK or connect to a robot.

The coordinator demo runs two mock arms and prints aggregate and per-arm joint
streams, verifying names, positions, ROS header frames, CDR round trips, and one
shared tick stamp. Its existing internal snapshot clock remains in floating
seconds and is explicitly converted at the message boundary. Both demos close
subscriptions and stop their modules on exit.

### Base-motion transport commands, odometry, and stop

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_base_transport.py
```

The actual base adapter receives generated PoseStamped odometry over LCM and
then independent loopback Zenoh sessions. It prints planar position `(2, -1)`
and yaw `pi/2`, sends velocity `(0.5, -0.2, 0.3)` after enabling, then verifies
a generated zero Twist arrives when disabled. It closes the adapter and all
transport resources. `test_base_transport.py` runs this exchange automatically.

### Generated joint feedback in manipulation planning

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_planning_joints.py
```

This decodes a generated coordinator JointState, selects the left-arm joints in
canonical order, normalizes a differently ordered target, and validates both
against declared joint limits. It prints selected positions `[-0.2, 0.3]`, target
`[0.1, 0.5]`, and the preserved source stamp `1700000000123456789`. No robot,
planning backend, ROS installation, or persistent artifact is required.


### Planner backends and robot self exclusion

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_planning_backends.py
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_self_filter.py
```

The backend demo requires the manipulation dependencies (including Drake). It
creates a temporary two-joint URDF, plans between generated JointState values,
interpolates five waypoints, and checks every CDR-decoded waypoint against the
real Drake world. The printed shoulder positions run from -0.4 through 0 to 0.4.
This is an unobstructed joint-path demo; it does not exercise obstacle avoidance
or the still-pending generated trajectory/pose cutover.

The self-filter demo requires yourdfpy. It moves a modeled cube through x=0, 1,
and 2, showing that the cube's return disappears while a distant point remains.
Both the retained cloud and the world-frame clear mask round-trip through CDR
with their original integer nanosecond timestamps. The demos remove temporary
models and dispose module resources on exit; neither needs robot hardware or ROS.


### Generated trajectories and execution status

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_trajectory.py
```

The trapezoid generator produces 51 ROS2 JointTrajectoryPoint values for two
joints. The demo sends the generated JointTrajectory over LCM and then over
independent loopback Zenoh sessions, preserving source nanoseconds
`1700000000123456789`. A real JointTrajectoryTask executes it against synthetic
feedback; printed commands progress from `[0, 0]` to `[0.3, -0.2]`. Generated
`dimos_msgs/msg/TrajectoryStatus` crosses the return channel and reports
`COMPLETED`, progress 1, and zero remaining duration. Each transport and session
closes on exit. `test_trajectory.py` runs the exchange as a subprocess test.

Import trajectories from `dimos_generated.trajectory_msgs.msg`; waypoint times
use `builtin_interfaces/Duration`. Use `duration_from_seconds` and `to_seconds`
from `dimos.msgs.time` at floating-time boundaries. `sample_trajectory` and
`trajectory_duration` in `dimos.msgs.trajectory` provide interpolation and total
time outside the generated message classes. Absent velocities mean zero
feed-forward for joint-position execution. Generated status state fields are
integers with constants on TrajectoryStatus; compare them with `==`.

The old handwritten JointTrajectory, TrajectoryPoint, and TrajectoryStatus
classes are removed. Generated sequences return value copies for message
entries: edit a point and assign it back with `trajectory.points[index] = point`
when changing a waypoint.

### Generated poses and grasp proposals

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_planning_backends.py
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_grasp_proposals.py
```

The Drake demo now prints each waypoint's forward-kinematics tool position and
its CDR TF edge. Derived poses retain the joint sample's exact timestamp; the
first printed stamp is `1700000000.123456789`. The synthetic URDF is removed
when the demo exits.

The grasp demo needs neither hardware nor a GPU. Four generated PointCloud2
points cross a CDR round trip into the heuristic provider. The generated
`dimos_msgs/msg/GraspCandidateArray` crosses another round trip and prints a
ranked grasp at `[0.5, 0.0, 0.15]`, a downward approach axis, score `1.0`, and the
unchanged source timestamp `1700000000123456789` nanoseconds. The provider stops
on exit.

Use nested ROS fields (`stamped.pose.position`, `stamped.header.stamp`) and the
functions in `dimos.msgs.geometry` for pose matrices, Euler angles, and local
translation. A pose-matrix conversion rejects non-finite or non-rigid geometry.
Generated configuration poses keep a zero timestamp unless a source supplies
one. Access grasp entries through `.candidates`; the former handwritten pickle
codecs have been removed. Grasp consumers validate scores and geometry before
commanding motion.

For base motion, run:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build \
  .venv/bin/python examples/message-codegen/demo_path_following.py
```

This demo decodes a generated 41-waypoint Path, runs the path follower against
DimOS's base simulator, and round-trips each Twist command through CDR. It prints
`arrived` and writes `build/message-codegen/demo/evidence/path-following.svg`
with the reference and executed paths. Remove that generated SVG to clean up.
Space's SVG and Rerun renderers now read generated poses, points, paths, detection
boxes, and occupancy grids directly. Both apply the occupancy origin rotation;
headless tests exercise the actual Rerun SDK as well as SVG output.

### Generated cloud-to-map and inflation demo

Run after building the generated Python messages above (the normal mapping environment
also requires NumPy, SciPy, Numba, and the visualization dependencies):

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_occupancy.py
```

The demo sends 1,200 synthetic points through CDR, projects them into an occupancy
map, inflates the obstacle by 0.2 m, and round-trips the generated grid through CDR.
It prints the occupied-cell counts and checks the exact source header. Open
`build/message-codegen/demo/evidence/occupancy.svg` to inspect the resulting map.
The demo needs no robot, ROS installation, viewer login, or running transport.
Remove that SVG to clean up its output.

### Generated map → native/Python planner → path demo

The normal native extension build provides A*. To build just that extension for
this demo, with the compiler and pybind11 already installed:

```bash
c++ -O3 -shared -std=c++17 -fPIC \
  $(.venv/bin/python -m pybind11 --includes) \
  dimos/navigation/replanning_a_star/min_cost_astar_cpp.cpp \
  -o dimos/navigation/replanning_a_star/min_cost_astar_ext$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_grid_path.py
```

Both planners route around the same synthetic wall using a CDR-decoded grid.
The demo resamples each result, sends it through generated Path CDR, checks exact
source headers and obstacle exclusion, and exports `grid-path-python.svg` and
`grid-path-cpp.svg` under `build/message-codegen/demo/evidence/`. Open the SVGs to
compare the routes. The native extension is required; the demo fails explicitly
if it is missing. Remove the two SVGs to clean up the demo output.

### Run the navigation module over both transports

After building generated messages and the native A* extension above:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_navigation.py
```

This starts the real `CostMapper` and `ReplanningAStarPlanner` modules and a
synthetic drive loop. A generated CDR cloud is accumulated by the CPU voxel
mapper, then sent to CostMapper over LCM and separate loopback Zenoh sessions.
The resulting generated OccupancyGrid feeds navigation alongside generated
Odometry and PointStamped inputs. The drive loop consumes generated Twist
commands and sends new odometry until the module publishes a generated arrival
Bool. It checks the published Path's exact source header and the final stop
command. The simulation advances 0.1 seconds per control command at an accelerated
100 Hz wall-clock rate; it needs no robot or ROS installation.

The terminal prints source/voxel counts, costmap dimensions, final position,
command count, and source timestamp.
Open `build/message-codegen/demo/evidence/navigation-lcm.svg` and
`navigation-zenoh.svg` to inspect the planned route. Module threads, subscriptions,
and transport sessions stop automatically; remove the two SVGs to clean up output.
`test_navigation.py` runs this same demo as a bounded subprocess E2E check.

### Basic follower and exact odometry history

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_odometry_history.py
```

This hardware-free, fixed-step simulation passes a generated CDR path through the
basic follower's control step, applies its generated Twist commands, and checks
its generated arrival Bool. Each simulated Odometry sample passes through CDR
and the real OdometryHist handler. The demo checks consecutive nanosecond stamps
and true-height Rerun geometry, then exports the reference and traveled paths to
`build/message-codegen/demo/evidence/odometry-history.svg`. Open that file for
visual review; remove it to clean up the demo output. This exercises control
steps directly; it does not start a robot or the follower's timed control thread.

### Generated frontier goals

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_frontiers.py
```

The demo decodes an occupancy map from CDR, detects the frontier between its
known and unknown halves, and round-trips the selected pose through CDR while
checking the exact source header. It prints the selected coordinates and writes
`build/message-codegen/demo/evidence/frontiers.svg`. Open that SVG to inspect
the selected goal; remove it to clean up. No robot, ROS installation, or viewer
login is required.

### Generated patrol goals

After the generation/build setup above, run:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_patrol.py
```

The demo prints five goals each for random, coverage, and frontier patrol, and
writes `build/message-codegen/demo/evidence/patrol-{random,coverage,frontier}.svg`.
Each goal crosses a CDR encode/decode boundary and retains the map's exact header.
The red markers show selected goals; this demo does not simulate travel between
them. No robot or ROS installation is required. Remove those three SVG files to
clean up the demo outputs.

### Generated movement arbitration

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_movement.py
```

This terminal demo forwards a stamped click through CDR, shows teleop overriding
navigation, then shows navigation resuming after cooldown. It uses in-process
module callbacks and CDR output round trips; it does not actuate a robot or
exercise network discovery. No output files require cleanup.

### Generated visual-servo commands

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_visual_servo.py
```

This terminal demo decodes generated CameraInfo and prints CDR-decoded velocity
commands for four synthetic detections: far away, at target distance, too close,
and right of center. It needs no camera, robot, or ROS installation and writes no
files. It exercises the 2D controller, not the complete person-follow pipeline.

### Generated detection-cloud filters

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_cloud_filters.py
```

This terminal demo filters a synthetic cloud containing a dense object and one
isolated low point. Height and radius filters each retain 100 of 101 points;
results cross CDR boundaries and preserve the source header. It requires Open3D
from the development environment, no sensor or ROS, and writes no files.

### Generated depth-to-detection projection

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_detection_projection.py
```

The terminal demo projects sixteen depth pixels into a translated world frame,
then prints the generated CDR centroid and exact source timestamp. It uses
synthetic depth and needs no sensor, robot, or ROS installation. No files are
written. This exercises projection, not model inference or the person-follow
module lifecycle.

### Generated image detection annotation

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_detection_bbox.py
```

Open `build/message-codegen/demo/evidence/detection-bbox.png` to inspect the
synthetic target's bounding box and label. The terminal prints CDR-decoded track
and class identifiers and checks the exact image header. Remove the PNG to clean
up. This uses synthetic pixels, with no model download or sensor required.

### Generated vision-model image boundary

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_vl_images.py
```

This offline terminal demo resizes a generated image and converts a fixed model
response into a CDR detection array. It prints the dimensions and verifies the
source header. It does not call an API or run model inference, and writes no files.

### Person-follow control thread with generated messages

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_person_follow.py
```

This hardware-free demo starts the actual person-follow control thread with
fixed tracker output. It prints a CDR-decoded forward command, requests stopping,
checks the final zero command, and joins the thread. Model creation and tracker
inference are substituted; this is not a model-accuracy or robot demo. It needs
the optional Torch/Hydra imports but downloads no weights and writes no files.

### Generated 3D marker messages

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_marker_messages.py
```

This terminal demo serializes two synthetic marker detections and prints their
CDR-decoded dictionary-qualified identities and world positions. It verifies the
exact source timestamp. It performs no marker inference or camera acquisition
and writes no files.

### Detect an ArUco marker from a generated image

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_aruco_detection.py
```

This demo renders marker 7, runs OpenCV ArUco detection and pose estimation,
composes a translated world pose, and decodes the generated CDR detection. It
writes `build/message-codegen/demo/evidence/aruco-detection.png` with the detected
bbox and label. Expected world coordinates are x approximately 2 m and z
approximately 0.402 m. No camera or model weights are needed; delete the PNG to
clean up.

### Generated bounding-box navigation goal

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_bbox_navigation.py
```

This terminal demo sends generated calibration and detections over LCM, then
prints a CDR-decoded goal (2.00, -0.40, -0.40) with the original detection header.
It retains the module's existing forward/left/up coordinate convention and does
not perform TF conversion. It needs no robot and writes no files.

### Holonomic tracking through CDR

After building the generated demo bindings, run:

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_holonomic_tracking.py
```

The terminal shows a synthetic robot accelerating toward `(2, 1)` and settling
at the target. Reference poses, measured poses, and limited velocity commands
round-trip through generated CDR codecs each tick. This exercises the tracking
law and limiter, without a transport, path planner, hardware, or ROS install.
No processes or output files require cleanup.

### Threaded holonomic path following

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_holonomic_path.py
```

The real follower thread drives a synthetic robot along a one-metre path,
prints decoded velocity and position, reports arrival, and publishes zero
velocity. Path, odometry, and commands use generated CDR round-trips. The demo
closes its thread and subscriptions automatically; it needs no robot or ROS.
This is an in-process control demo, not network transport verification.

The holonomic path demo also exercises `DanLocalPlanner`'s core: it smooths and
resamples the incoming CDR path and visibly suppresses a duplicate replan inside
the configured commit window before starting the follower.

### Native MLS planning from generated clouds

With the generated demo bindings already built, install the native Python
planner into the checkout environment and run the terrain demo:

```bash
uv pip install --python .venv/bin/python 'maturin>=1,<2'
source .venv/bin/activate
maturin develop --release --uv -m dimos/navigation/nav_3d/mls_planner/rust/py/Cargo.toml
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_mls_transformer.py
```

The demo prints a TF-derived starting pose, then sends a CDR-decoded synthetic
floor to the native MLS planner and prints the CDR-decoded path waypoints and
exact source timestamp. It requires no robot, model download, or ROS install.
It runs in one process and writes no output artifacts; native build products
remain in Cargo's ignored target directory.

The MLS terrain demo also writes
`build/message-codegen/demo/evidence/mls-planner.rrd`, containing the floor,
raised path nodes, and connecting segments through the planner's Rerun adapters.
Open it with `rerun build/message-codegen/demo/evidence/mls-planner.rrd`.
For headless verification, use `rerun rrd verify` on that file. The recording is
an ignored build artifact and may be deleted after review.

### Recorded navigation evaluation

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_navigation_recording.py
```

The demo writes generated lidar and odometry messages to temporary SQLite
streams, reads them through the navigation evaluator, and prints sensor-to-world
point registration and accumulated trajectory distance. It removes the temporary
database automatically. No recording download or hardware is required.

The terrain demo also exercises native ray tracing and records its local map,
TF edges, and colormap annotations. Install its additional native extension with:

```bash
maturin develop --release --uv -m dimos/mapping/ray_tracing/rust/py/Cargo.toml
```

The recorded-navigation demo now also invokes the actual native ray-tracing CLI
on its generated SQLite lidar/TF streams. Install the ray-tracing extension as
above. It writes `build/message-codegen/demo/evidence/raytrace-cli.rrd`; inspect
that file with Rerun or check it with `rerun rrd verify`. The temporary SQLite
source is still removed automatically.

The terrain recording's TF hierarchy is now logged through the live Rerun
bridge's generated-message callback, including its frame axes. This demo invokes
the callback directly and does not start a network subscription or viewer.

### Generated camera messages through the bridge

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_camera_bridge.py
rerun rrd verify build/message-codegen/demo/evidence/camera-bridge.rrd
```

Open `camera-bridge.rrd` in Rerun to inspect a red/green image, its compressed
copy, calibration, and two depth images that both represent one metre. The demo
uses generated CDR messages and the bridge callback; it does not start a network
subscription. Its only output is the ignored RRD file.

The camera demo also logs a generated 3D box labelled `demo-box id=4` in the
camera's optical frame, exercising the same bridge used for image dispatch.

The terrain demo sends the generated terrain cloud through the shared bridge,
which renders explicit height colors. The same adapter preserves standard packed
RGB colors when supplied by a cloud producer.

The terrain demo's `world/planned_path` entity now comes from the shared bridge's
generated Path adapter. It is displayed 0.5m above the original route for clarity;
the path message retains its original coordinates.

The terrain recording also includes `world/occupancy`, a small generated grid
with free, occupied, unknown, and intermediate-cost cells. The shared adapter
places its textured plane using the grid's origin pose.

### Live LCM/Zenoh-to-Rerun bridge

```bash
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_live_bridge.py --transport lcm
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_live_bridge.py --transport zenoh
```

This starts the real bridge headlessly on an available local gRPC port, publishes
a generated image over the selected transport, waits for decoded/rendered delivery, and writes
`build/message-codegen/demo/evidence/live-bridge-lcm.rrd`. It closes the subscriptions
and transport automatically and never opens a window. A bounded subprocess test
runs the demo and verifies the recording with Rerun. Multicast loopback must be
available in the test environment.

The Zenoh variant writes `live-bridge-zenoh.rrd`. It uses separate publisher and
subscriber sessions connected explicitly over loopback TCP with scouting and
gossip disabled. This verifies the transport-to-bridge route without claiming
default peer-discovery reliability. Both variants have bounded subprocess tests.

Run the viewer control protocol without a GUI:

```sh
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_viewer_controls.py
```

This opens a local WebSocket, sends a click, velocity, and stop, and prints the
verified generated CDR values. It requires no robot or ROS installation.

Inspect command-center JSON and a generated click goal without a GUI:

```sh
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_command_center.py
```

The demo exercises registered command-center handlers directly and prints the
state prepared for the browser, including a compressed costmap.

Inspect a VLM request image without credentials or model inference:

```sh
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_vlm_image.py
```

The demo writes `build/message-codegen/demo/evidence/vlm-request.jpg` from a
generated CDR image through the agent's request builder, using a stubbed model.
