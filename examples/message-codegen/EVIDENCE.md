# Stage 1 local verification

Captured on 2026-09-21 on Linux x86_64 with Python 3.12, GCC 16, and Rust 1.98.
These are terminal demos intended for human inspection, executed automatically.
The ROS reference runs independently in a Jazzy container.

The full build generated 142 message types. Focused verification after adding
the all-types oracle passed **22 tests**. The raw transport passed nine unit tests
and one doctest. `cargo build --offline` also passed after dependency setup.

## relay

```text
Python sends: sequence=40 label=start hops=[1]
C++ received: frame=map sequence=40 label=start temperature=21.5 x=4.25 axes=[1,2,3]
Rust received: frame=map sequence=41 label=start/cpp temperature=21.5 x=4.25 axes=[1.0, 2.0, 3.0]
Python receives: sequence=42 label=start/cpp/rust hops=[1, 2, 3]
Nested fields, fixed arrays, bytes, defaults, and edits survived. Evidence: build/message-codegen/demo/evidence
```

## conformance

```text
LE Python -> Python: fields and bytes match
LE Python -> C++   : fields and bytes match
LE Python -> Rust  : fields and bytes match
LE C++    -> Python: fields and bytes match
LE C++    -> C++   : fields and bytes match
LE C++    -> Rust  : fields and bytes match
LE Rust   -> Python: fields and bytes match
LE Rust   -> C++   : fields and bytes match
LE Rust   -> Rust  : fields and bytes match
BE Python -> Python: fields and bytes match
BE Python -> C++   : fields and bytes match
BE Python -> Rust  : fields and bytes match
BE C++    -> Python: fields and bytes match
BE C++    -> C++   : fields and bytes match
BE C++    -> Rust  : fields and bytes match
BE Rust   -> Python: fields and bytes match
BE Rust   -> C++   : fields and bytes match
BE Rust   -> Rust  : fields and bytes match
Python/C++/Rust reject truncated
Python/C++/Rust reject unsupported representation
Python/C++/Rust reject trailing bytes
Python/C++/Rust reject missing header
All nine encoder/decoder combinations and both input byte orders match the independent ROS2 codec.
```

## buffers

```text
640x480 RGB image: 921,600 bytes, shared read-only view; resize rejected
  local median (microseconds): borrow=4.2, copy=31.6, encode=71.4, decode=74.2
100,000-point buffer: 1,200,000 bytes, shared read-only view; resize rejected
  local median (microseconds): borrow=5.7, copy=47.0, encode=119.3, decode=128.2
Borrowed image remains readable after its message variable is released.
```

## transport

```text
Python -> native Rust -> Python: 128 raw bytes match
Python -> native Rust -> Python: 65536 raw bytes match
Python -> native Rust -> Python: 1048576 raw bytes match
```

## jazzy-reference

```text
ROS2 Jazzy decoded python-le.cdr: every field matches
ROS2 Jazzy decoded python-be.cdr: every field matches
ROS2 Jazzy decoded C++.cdr: every field matches
ROS2 Jazzy decoded Rust.cdr: every field matches
ROS2 Jazzy confirms declared defaults, nested fields, bounds, arrays, Unicode, and both byte orders.
```

Reproduce with the commands in [README.md](README.md). CI retains fresh output
and binary payloads as artifacts; generated sources and build products are ignored.

## Stage 2 installed application

Built a Python wheel from its sdist, installed CMake packages, and compiled a
separate consumer from the Cargo archive. The Python consumer ran with `-I` in a
fresh environment containing only the external message wheel.

```text
Installed Python package sends new field: added-locally
Installed C++ package received: added-locally
Packaged Rust crate received: added-locally/cpp
Installed Python package receives: added-locally/cpp/rust
A locally added field crossed three installed native packages; no DimOS source change or upstream PR.
```

The full DimOS wheel also built with web assets. Its sdist passed the repository
content check (1,928 entries; 10.7 MB). A fresh environment with both wheels
discovered 141 types and accepted a built-in Point inside external Telemetry.
Two separately installed CMake packages can include the same standard Point
without duplicate definitions. Focused tests now pass 34 cases.

## Stage 3 automated recording checks

`bash scripts/test_message_mcap.sh` passed on 2026-09-21 with Rerun 0.32.0,
MCAP 1.4.0, and the locked Foxglove JavaScript packages. The artifact contains
150 messages and is 3,123,201 bytes, including all schema dependencies.

```text
4 passed
Foxglove libraries decoded 30 /camera/image messages from embedded definitions.
Foxglove libraries decoded 30 /camera/compressed messages from embedded definitions.
Foxglove libraries decoded 30 /robot/pose messages from embedded definitions.
Foxglove libraries decoded 30 /telemetry messages from embedded definitions.
Foxglove libraries decoded 30 /tf messages from embedded definitions.
Rerun imported 30 /telemetry rows as demo_msgs.msg.Telemetry:message.
Rerun imported 30 /camera/image rows as Image:buffer.
Rerun imported 30 /camera/compressed rows as EncodedImage:blob.
Rerun imported 30 /robot/pose rows as InstancePoses3D:translations.
Rerun imported 30 /tf rows as Transform3D:translation.
Custom telemetry is a structured Arrow value with nested fields.
```

The stage-1 conformance/buffer/raw-transport demos and stage-2 installed-package
demo were rerun after correcting dependency section names to `package/Type`.
The Rerun browser viewer displayed both images and exposed custom Telemetry's
`sequence: 29`, `reading.temperature: 22.9`, and `hops: [1, 2, 3]` from the MCAP.
Local screenshots are in `build/message-codegen/viewers/`.

Full viewer acceptance is still pending. Foxglove's application redirected to
sign-in in the isolated browser; no credentials were entered. Its decoding
libraries pass independently, but that does not establish application UI
acceptance. Rerun's direct-file screen capture also needs final confirmation
of pose visualization and complete semantic playback before task 3.5 is closed.

## Runtime transports and browser cutover (2026-09-21)

The runtime work is still in progress. This evidence covers typed transports,
web schema decoding, and the migrated built-in web adapters; it does not establish
that all DimOS consumers or native SDKs have completed the cutover.

- `demo_pubsub.py`: generated `LineSegments3D` and 921,600-byte `Image` exchanged
  over raw LCM, separate loopback TCP Zenoh sessions, and CPU shared memory.
  Decoded bytes and integer nanosecond timestamps matched on every path.
- Typed pub/sub pattern, core CDR transport, and CLI checks: 67 passed. A glob
  subscription regression was fixed: `/sensor/*` must match the logical topic
  before the slash-separated `#package/msg/Type` suffix.
- WebRTC plus core transport checks: 34 passed. Explicit data-frame channel/type
  routing distinguishes `Point` and `Vector3` despite their identical CDR layouts.
  Truncated, trailing, raw/unframed, foreign-type, foreign-channel, and wrong-codec
  frames are dropped; the next valid frame is delivered.
- Python web codecs: 48 passed. Cockpit authoring plus generated array/helper
  checks: 94 passed. Costmap and map codec checks: 17 passed.
- Browser SDK: 322 passed. Cockpit: 154 passed. Both TypeScript checks passed.
  The CDR decoder uses pinned Foxglove parser/serialization libraries. Fixtures
  cover standard and custom messages in both byte orders, raw images, fixed and
  dynamic arrays, empty messages, Unicode, and integers beyond JavaScript's safe
  number range. Former LCM codecs and their browser fixtures were removed.
- Chromium opened `demo_cdr.html` and displayed the generated custom message's
  endpoints, weight 4, and `stamp.nanosec = 500000123`. Switching to the image
  and big-endian CDR displayed both pixels. Reading the rendered canvas yielded
  RGBA `[0, 1, 2, 255, 253, 254, 255, 255]` as expected.
  Screenshots: ignored `build/message-codegen/viewers/cdr-browser-custom.png`
  and `cdr-browser-image.png`. Browser recording:
  `/tmp/dimos-cdr-browser-harness/agent-workspace/recordings/session-20260921-192504`
  (6 frames). The task-owned browser and Vite server were stopped afterward.

The combined Python runtime/web subset passed 318 tests before adding the explicit
WebRTC sequence regression; that WebRTC file then passed all 29 tests. The final
SDK and cockpit runs remained 322 and 154 tests, with both type checks passing.
Mypy passed on 12 changed runtime/helper/codec modules. The external packaging
relay and installed typing check also passed after making the bundled generator
a regular package so an installed DimOS cannot shadow it.

The subsequent relay consumer migration passed the entire `dimos/web` suite
plus the generated image/grid helper tests: **677 passed in 35.59 s**. This
includes forkserver blueprint deployment, real relay session/transport tests,
image and map delivery, cached-map replay after unsubscribing, relay respawn,
teleoperation stop/watchdog behavior, and lease handover. The watchdog's old
`Twist.zero()` call was replaced with the generated zero-valued `Twist()`.
The new array-to-image helper also passed strided input, independent copy,
explicit encoding, and big-endian depth tests.

### Native SDK CDR relay

The C++ SDK suite passed all **76 tests** using real generated messages in its
default-codec tests. The Rust SDK passed **143 tests**, including TF, both CDR
byte orders, malformed input, and propagation of publish encoding errors without
enqueuing a payload. C++ and Rust ping/pong examples and the custom/image relays
built successfully. The generated Rust crate also passed Cargo source-package
verification: a build from the packaged archive, with only Python's standard
library used for generation and no preexisting generated source.

The native demo used the real SDK processes, generated message packages, stdin
launch protocol, and Python typed transports:

```text
lcm: Python weight=4 → C++ weight=5 → Rust weight=6 → Python verified
lcm: 921,600 image bytes and source nanoseconds=1700000000123456789 match
zenoh: Python weight=4 → C++ weight=5 → Rust weight=6 → Python verified
zenoh: 921,600 image bytes and source nanoseconds=1700000000123456789 match
```

Full output and process logs are under `build/message-codegen/demo/evidence`.
This does not complete the native consumer, Nix build matrix, coordinator
blueprint, or live Rerun migration; stage 4 remains open.

### Lidar and mapping message consumers

After switching their generated fields and codecs, the Livox Rust suite passed
24 tests, voxel mapping passed 60, and the MLS planner passed 94. The planner's
new regression checks that graph endpoints and cost survive CDR in explicit
`LineSegments3D` fields. The SDK suite passed 146 tests after extracting the shared
point-cloud reader, including padded organized rows, mixed float32/float64 XYZ,
both byte orders, malformed layouts, and nonfinite-coordinate filtering.

The Python structured point-cloud helper passed 21 tests; together with the
Livox configuration tests, the focused Python run passed 26. The helper and
three native Python module declarations passed mypy with the generated stubs.
`demo_pointcloud.py` displayed both byte orders with organized shape `(2, 1)`,
row/point strides `(32, 20)`, and preserved two-value tags. Editing its explicit
copy left the borrowed source unchanged. Output is recorded in
`build/message-codegen/demo/evidence/pointcloud-layouts.txt`.

The Livox live loopback E2E also passed against the released CDR binaries
(`test_live_loopback_handshake_and_stream`, 3.03 s). It exercised the virtual
Mid-360 handshake, actual UDP sensor traffic, native CDR publication, Python
schema decoding, a 5 m point ring, per-point deskew offsets, and gravity IMU.
The physical-sensor recording test remains deselected; no robot was required.

### Remaining lidar and camera native adapters

Point-LIO's Rust suite passed 7 tests after moving to generated CDR. Its point
reader now preserves deskew offsets for padded organized rows in either byte
order. RealSense's Rust suite passed 3 tests, including the generated `ImuInfo`
round trip that replaces its handwritten fingerprint and LCM encoder; its Python
configuration suite also passed 3. The hardware-free RGBD benchmark built and
encoded 407,040, 101,760, and 45,280 points at pixel strides 1, 2, and 3.
These are synthetic conversion checks, not a physical-camera acceptance test.

The dimSLAM Rust adapter passed 7 tests, covering generated odometry/cloud
conversion, exact and negative timestamps, depth byte-order normalization with
row padding, BGR-to-RGB conversion, and invalid layouts/backend dimensions.
The Python camera/lidar declarations and geometry helper passed mypy.
The new odometry-to-TF helper passed zero, negative, and nanosecond-precise source
timestamp cases and preserves the message's declared parent/child frames.

The C++ SDK suite passed 77 tests including the production lidar cloud builder.
Its standalone `dimos_lidar_cdr_demo` visibly decoded two XYZI points in each CDR
byte order, preserving frame `lidar`, timestamp `1s + 250000000ns`, and intensity
`0.5`. Output is in `build/message-codegen/demo/evidence/lidar-cpp.txt`.

Both full C++ lidar Nix builds completed successfully with the generated CMake
package: Point-LIO and FAST-LIO. Their flake inputs/locks and CMake configurations
no longer fetch `dimos-lcm` headers. The generated package itself built from the
pinned source definitions in Nix. The root Rust workspace passed `cargo check
--workspace --locked`; the independently built RealSense and dimSLAM crates
passed the tests reported above.

Clippy passed with warnings denied for the SDK and migrated root-workspace
modules, and independently for RealSense and dimSLAM. Generated explicit Default
implementations retain a targeted lint annotation: the same emitter supports
nonzero `.msg` defaults and fixed arrays longer than 32 elements.

### Independently generated messages in the native SDKs

The external package relay was rebuilt after aligning generated Rust codec errors
with SDK port signatures. Both the installed file relay and the actual native
SDK relay passed. The external application changed only its local `.msg` copy;
its Rust ports used the external type's generated methods directly:

```text
lcm: external Python → native C++ → native Rust → Python: added-locally/cpp-native/rust-native
zenoh: external Python → native C++ → native Rust → Python: added-locally/cpp-native/rust-native
SDK codecs accepted the external generated type directly; no handwritten codec or upstream schema PR.
```

All other fields and the exact timestamp matched after both native processes.
The external Rust consumer test also verified SDK-compatible function signatures,
`InvalidInput` on bounded-field encode failure, and `InvalidData` on truncated
CDR. The built-in custom-message/image SDK relay still passed on both transports,
and the Rust SDK suite still passed all 146 tests. Mypy passed for both runtime
demo entry points using their separately generated type stubs.

### TF through the real module coordinator

The generated-message Python/Rust TF example now runs through
`ModuleCoordinator.build`, Python worker deployment, native process startup,
and typed stream wiring. Both LCM and Zenoh checks passed, each collecting four
composed `a → d` transforms with `x=1.5`, `y² + z²=1`, and changing positions.
Both runs returned normally after interruption and stopped their native workers.
The test logs are captured in pytest's temporary directory; the test summary is
`build/message-codegen/rust-tf-blueprint-tests.log`.

This verifies coordinator wiring beyond the direct native SDK relay. It does not
complete the three-language blueprint with live viewers required by task 4.10.

### Raw LCM fragment validation

The retained Rust raw transport passed 18 unit tests and its compile doctest after
adding bounded reassembly. Checks cover reordered/duplicate fragments, conflicting
metadata, invalid offsets/counts, overlapping or missing bytes, invalid channels,
expiry, aggregate payload limits, and continuing reception after malformed UDP
packets. Clippy passed with warnings denied.

The Python ↔ Rust raw transport demo still exchanged 128, 65,536, and 1,048,576
bytes exactly. After rebuilding the Rust native examples, the three-language SDK
relay passed on LCM and Zenoh again, preserving 921,600 image bytes and timestamp
`1700000000123456789`. Transport limits are documented in the raw crate README.

### Three-language coordinator blueprint

`demo_blueprint.py` now deploys the Python producer/verifier and C++/Rust relays
through the actual module coordinator. Its parametrized E2E test passed on both
LCM and Zenoh, checking three distinct samples each. Two additional standalone
Zenoh launches each verified five samples. Every sample changes its custom
segment fields, 921,600 RGB image bytes, and exact source nanoseconds; Python
compares the complete returned messages against the expected native edits.
Mypy and Ruff passed for the demo.

The Zenoh demo uses a temporary loopback router and explicit client endpoints.
Initial runs using default multicast peer discovery intermittently returned
neither stream despite successful native startup. Explicit routing passed the
repeated launches, but this does not establish the cause or fix default peer
discovery. Keep that finding open for the final runtime checks. Logs are
`build/message-codegen/three-language-blueprint-{tests,zenoh,zenoh-repeat}.log`.

This completes the three-language coordinator portion of the human demo;
live Rerun/browser image and pose integration is still required for task 4.10.

### Generated Python TF and recorded-stream lookup

The live TF buffer and `StreamTF` now use generated `TransformStamped` and
`TFMessage` values. Composition/inversion/Euler conversion live in geometry
helpers backed by the existing SciPy dependency, outside generated classes.
The time index keys transforms by integer nanoseconds, snapshots incoming data,
and returns independent lookup values.

The migrated TF suite and geometry helper suite passed 68 checks together:
live/in-memory/SQLite lookup parity, composition/inversion, nearest-time and
retention behavior, exact/negative timestamps, distinct adjacent nanoseconds,
copy independence, and invalid rotations/frames. Mypy passed on the TF buffer,
recorded-stream lookup, geometry helpers, and native TF demo. The coordinator
TF check passed on both LCM and Zenoh, now verifying Python and Rust composition
of the same chain.

Correction to the earlier quaternion diagnosis: the pinned Jazzy
`geometry_msgs/msg/Quaternion.msg` defines `w=1`; the generated default is an
identity rotation. A zero quaternion must be constructed explicitly to test
rejection. The initial missing TF replies were not evidence of a zero default.
Robot-specific publishers and other legacy convenience-class consumers remain
to be updated; this is not completion of task 4.2 or 4.6.

### Static publishers and robot mount callers

`StaticTfPublisher` and its frame-tree helper now use generated stamped transforms
and exact `time.time_ns()` stamps. The Go2/Mid-360, Alfred URDF, RealSense, and
Mid-360/RealSense mount declarations use those values. Go2's Zenoh mount tree and
odometry-to-TF callback were converted as well; its camera/video consumers still
need their separate runtime migration.

Spot's URDF mount composition and optical-frame rotation now use the geometry
helpers. Its SDK odometry boundary publishes generated Odometry and a matching
TFMessage, and replay's TF/odometry declarations use the generated types. A
synthetic SDK-value test checks negative source time, pose, velocity, frame names,
and CDR round trips without importing or connecting to the robot SDK.

Eleven mount/helper/Spot boundary checks passed. The cumulative TF demo now
includes the production periodic `StaticTfPublisher` for `b → c`; Python publishes
`a → b`, Rust publishes `c → d`, and both language buffers compose `a → d`.
Both LCM and Zenoh E2E checks passed, including shutdown. This is hardware-free
message/geometry verification, not physical robot or camera acceptance.

### Camera calibration and webcam cutover

Calibration construction/YAML loading now returns generated CameraInfo values.
The Go2 YAML helper was moved out of the connection module so the Zenoh-side
module no longer imports the robot SDK just to obtain calibration. Go2's periodic
calibration publisher uses integer nanoseconds and leaves its stored template
unchanged. The broader Go2 connection still has legacy image/cloud/pose paths;
its two generated perception-interface type mismatches remain to be resolved by
that consumer migration.

The webcam and CameraModule use generated Image, CameraInfo, and TFMessage values.
Stereo crops preserve RGB pixels and exact source headers. Metadata uses a shared
stamp, declared frame names, and independent copies of configured mounts. Image
sharpness is now an explicit helper used by the existing quality-barrier operator.

Forty-four camera/helper/view tests passed, plus the focused Go2 publication test.
Checks cover the actual 1280x720 equidistant calibration, CDR round trips, invalid
matrix dimensions/intrinsics/FOV, synthetic BGR capture and stereo crops, metadata
frame/stamp consistency, and sharpness ordering across visual encodings. Mypy
passed on eight changed helper/camera/calibration/demo modules. Physical capture
and the complete Go2 stack were not run.

`demo_message_helpers.py` visibly prints the decoded calibration matrix and four
distortion coefficients with source timestamp `1700000000123456789`; output is
`build/message-codegen/demo/evidence/message-helpers.txt`.

### Go2 sensor values and simulation lidar IPC

The WebRTC video/lidar conversion now constructs generated Image/PointCloud2
values directly. Hardware-free tests exercise the actual reactive conversion
using synthetic driver events, checking RGB pixels, XYZ coordinates, declared
frames, exact host-arrival stamps, and CDR round trips. The new XYZ factory
preserves organized shape, copies source storage, and rejects finite coordinates
that cannot fit float32. Missing points remain NaN/infinity with `is_dense=False`.

MuJoCo video/calibration construction and its lidar shared-memory boundary use
generated messages. Lidar IPC now carries CDR instead of pickle. Odd/even sequence
checks discard an observed in-progress or changed write before decoding. The
standalone `demo_mujoco_lidar_shm.py` passed with separate producer/consumer
processes and printed two expected XYZ points and timestamp
`1700000000123456789`. No engine or robot was launched.

The focused cloud, WebRTC conversion, connection lifecycle, and shared-memory
suite passed **51 tests**, with one hardware-recording test deselected. Log:
`build/message-codegen/go2-sensor-cutover-tests.log`. Mypy passed on eight changed
production modules. Pytest ran with `-o addopts=''` because this local environment
lacks the configured xdist/timeout/coverage plugins. Repository conftest emitted
its existing subprocess ResourceWarning; the standalone IPC demo exited cleanly.

This resolves the Go2 image/cloud interface type mismatches recorded above.
Go2/WebRTC/MuJoCo pose and TF callers still need their generated-value cutover;
DimSim's external browser producer and complete live simulation remain unverified.

### Go2 and simulation pose/TF values

WebRTC odometry and TF streams now return generated PoseStamped and
TransformStamped. The raw converter preserves the device header; the live
connection explicitly replaces it with a host-arrival header, preserving the
existing timing policy without a float timestamp round trip. Go2's publication
copies the received pose before overriding its configured parent frame. Its
mount chain and TFMessage retain that pose's exact stamp.

MuJoCo's pose boundary maps simulator WXYZ quaternions into generated XYZW fields.
DimSim's TF builder and G1 simulation ports/mounts also use generated values. G1's
camera metadata is restamped per publication without mutating the template.
The Unitree legacy Odometry subclass still serves unconverted recording-fixture
callers; it has no new fallback role and remains scheduled for deletion.

The focused geometry, device conversion, Go2/WebRTC, MuJoCo lifecycle/pose, and
simulation TF suites passed **43 tests**. Simulator startup dependencies are
stubbed in lifecycle/pose tests; shared-memory odometry uses real regions. No
engine or robot was run. Eight changed production modules passed mypy.
Logs: `build/message-codegen/unitree-pose-tests.log` and
`build/message-codegen/unitree-simulation-tf-tests.log`.

The human-facing `demo_robot_tf.py` passed and printed the decoded namespaced
camera chain with source timestamp `1746565669448350564`. Captured output:
`build/message-codegen/demo/evidence/robot-tf.txt`.

The broader Go2 blueprint topology test could not collect because PyTorch is
absent from this environment. Its generated-pose fixture has been updated, but
that test is not counted as passing. Full blueprint/runtime acceptance remains
open, as do recorder and legacy-fixture cutovers.

The cumulative Python/Rust coordinator TF demo was rerun on both LCM and Zenoh:
**2 E2E tests passed** in 8.68 seconds, including changing transform lookups and
clean worker shutdown. Log: `build/message-codegen/unitree-pose-tf-e2e.log`.
