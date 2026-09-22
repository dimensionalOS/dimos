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

### Python storage codecs and generated-message replay

Generated messages now select CDR instead of falling through to pickle. SQLite
persists/reconstructs `cdr` or `lz4+cdr`; old `lcm`, `lz4+lcm`, and private `jpeg`
codec IDs are rejected. The obsolete Python LCM and JPEG storage codecs are
removed. Raw/depth Image values remain lossless; compressed images are explicit
CompressedImage values stored through the same CDR codec.

The generic MCAP reader resolves installed types from `cdr`/`ros2msg` schema names.
Unknown schemas remain raw bytes, and Python payload-module metadata does not
trigger imports. Duplicate compatible channels on a topic aggregate counts;
conflicting schemas/timing policies and stream-name collisions are rejected.
Generic dataset opening now uses McapStore, leaving the robot-specific DDS preset
available for callers that explicitly choose it.

TransportRecorder now accepts generated-message contracts and reads source time
from Header. A zero source stamp is preserved; unstamped values use receipt time.
Observation pose conversion accepts generated nested PoseStamped/TransformStamped
and returns generated values. Replay ports use the recorded generated class
directly; the old rich-subclass-to-base-type substitution is removed.

**47 focused tests passed**, covering exact values/bytes after SQLite reopening,
raw/depth/compressed images, codec reconstruction and obsolete-ID rejection,
MCAP schema discovery/time ordering, pose metadata, recorder timestamps, and real
module replay. Log: `build/message-codegen/storage-cdr-tests.log`.
Tests ran with `--noconftest` because the memory conftest eagerly imports optional
CLIP/PyTorch dependencies. Two broader checks were deselected after their imports
failed: Rust-recorder delegation needs PyTorch, and the full Rerun blueprint needs
Numba. They are not counted as passing. Seven changed production modules passed
mypy. Checking the demo's full import graph additionally hits the installed MCAP
writer's `from .__init__ import __version__`, which mypy reports as duplicate
module names; the demo itself was executed successfully.

`demo_storage_replay.py` passed: SQLite and MCAP both returned weights `[4, 5, 6]`,
and module ports replayed stamps `1700000000123456789` through
`1700000000123456791`. Output is
`build/message-codegen/demo/evidence/storage-replay.txt`. Temporary files and module
resources were cleaned up. The Rust recorder and remaining legacy fixtures still
need conversion; this does not close stage 4 or stage 5 acceptance.

### Native recorder configuration dependency audit

The shared OnExisting policy moved to `dimos.memory.recording_policy`; its callers
now import it there. Native recorder configuration no longer imports the Python
embedding module just to obtain this enum. The previously blocked Rust-session
delegation test passes without PyTorch, and the policy/native configuration
modules pass mypy.

Running both native-recorder Python configuration suites now reaches their real
storage behavior: **23 passed, 9 failed**. Every failure requests the obsolete
`lcm` storage codec. This is an outstanding coordinated Python/Rust recorder
cutover, not an accepted regression result. Log:
`build/message-codegen/rust-recorder-config-audit.log`.

### Native recorder CDR and external schemas

The coordinated native recorder cutover resolves the nine obsolete-codec failures
above: **32 Python configuration checks, 16 Rust unit tests, five native process
E2E tests, and one external-package recording E2E test passed**. Production Python
configuration modules pass mypy; the Rust recorder passes Clippy with warnings
denied. Root workspace `cargo check --workspace --locked --offline` passes.

The recorder now receives complete generated schemas, writes ROS2-profile MCAP
with `cdr` channels and embedded `ros2msg` definitions, and uses chunk compression.
SQLite accepts `cdr` and `lz4+cdr`; obsolete LCM/JPEG wrappers are removed. Raw
image pixels remain exact. The root Cargo lockfile no longer depends on the
external `dimos-lcm` message repository; the in-tree raw LCM transport remains.

Recognized stamped messages retain integer source nanoseconds, including the E2E
MCAP value `1700000000123456789`. Zero stamps remain zero; invalid nanosecond
fields are rejected. MCAP rejects negative timestamps instead of silently
clamping them. SQLite converts timestamps to seconds only at its existing query
API boundary. Known ordinary message payloads preserve their original CDR bytes,
including byte order; TF arrays split into generated single-transform TFMessages.

`demo_native_recording.py` records three changing external Telemetry messages on
each transport with the same Rust binary. The locally added `application_note`
field survives along with every payload timestamp. The unknown custom timestamp
layout uses reception time for MCAP indexing. Foxglove's independent decoder
reads all six samples from embedded schemas without the custom package installed
in its environment. This is decoder evidence, not Foxglove/Rerun UI acceptance.

Review artifacts and transcripts:

- `build/message-codegen/demo/evidence/external-native-recording-{lcm,zenoh}.mcap`
  and matching `.log` files.
- `build/message-codegen/demo/evidence/native-recording.txt` and
  `native-recording-foxglove.txt`.
- `build/message-codegen/rust-recorder-cdr-tests.log`,
  `rust-recorder-config-cdr-tests.log`, `rust-recorder-cdr-e2e.log`, and
  `external-native-recording-e2e.log`.
- `build/message-codegen/rust-recorder-cdr-clippy.log` and
  `recorder-workspace-check.log`.

The Python recorder checks use `--noconftest` to avoid unrelated optional
CLIP/PyTorch imports. This leaves an unregistered `self_hosted` marker warning.
The host filesystem filled during the first workspace/Nix build attempts; those
attempts failed and are not counted as passing. Removing only this worktree's
reproducible compiler caches allowed the workspace check and executable build
to pass. The isolated Nix retry then passed, including all 16 Rust tests and
installation/fixup. An offline Nix build confirmed the completed output. The
installed Nix executable also passed the external custom-message demo on both
transports; its transcript is
`build/message-codegen/demo/evidence/nix-native-recording.txt`, with MCAP/log
artifacts in the adjacent `nix-native-recording/` directory. Full stage 4/5
acceptance, legacy consumers/fixtures, and live viewer demos remain outstanding.

### Optional ROS bridge uses CDR directly

Removed the LCM intermediate types, recursive field-copy conversion, renamed
time fields, complex-type allowlist, and old type-import caches. The bridge
resolves canonical `package/msg/Type` names and uses installed ROS serialization
with generated DimOS encode/decode. Mismatched message identities are rejected
even when their wire layouts match. Importing the conversion module remains
possible without ROS; actually requesting conversion reports the missing
optional dependency.

**25 checks passed** against the installed Jazzy type support in the isolated
reference container. Coverage includes seven real pub/sub cases, nested poses,
empty/nonempty TF arrays, raw/compressed image bytes, padded big-endian point
cloud data, camera calibration arrays, Unicode frames, exact epoch/zero/negative
stamps, invalid names, and wrong-type rejection. On the ROS-free host, four
contract checks pass and 21 ROS-dependent checks skip. The container's minimal
pytest reports three unknown optional-plugin configuration warnings.

`demo_ros_bridge.py` passed with three changing image/pose pairs on actual ROS
topics. Transcript: `build/message-codegen/demo/evidence/ros-bridge.txt`. The
local existing reference image needed `typing_extensions.py` mounted from the
host virtualenv; the updated reference Dockerfile explicitly installs that
dependency and pytest. The CI job now downloads the standalone Python binding
and runs the same bridge checks and demo; this is workflow wiring, not a claim
that the remote job has passed.

Galaxea depth/lidar/IMU and compressed-image callers now use generated sensor
types. Compressed images preserve their source format and header rather than
forcing JPEG and replacing zero timestamps with wall time. Other Galaxea
geometry/control consumers still need conversion; robot hardware was not used.
Mypy passes on the converter, Galaxea connection, and demo. The full runtime
cutover remains incomplete.

### Galaxea generated control and feedback messages

The R1Pro connection now uses generated MotorCommandArray, JointState, Twist,
PoseStamped, Odometry, and TFMessage alongside the previously converted sensor
streams. No legacy message imports remain in this connection module. Commands
use generated arrays directly and require 18 entries in every declared array
before publishing. ROS command conversion uses the CDR bridge; unsupported
base axes remain zero.

Wheel integration computes delta time from integer source nanoseconds and
publishes nested ROS2 pose/twist fields. Pose, odometry, and both TF edges retain
the same exact stamp. Joint-state aggregation retains the oldest segment stamp,
including zero, without converting through floating-point seconds or replacing
it with wall time. The publisher's interval wait now responds to its stop event.

**13 focused checks passed**, covering integration and clock jumps, exact
zero/epoch joint stamps, all 18 joint values, three command segments, tracking
velocity sentinels, and rejection of each malformed command array. Tests use
real generated messages and mock only the ROS/output boundaries. Mypy passes on
the connection and demo. Log: `build/message-codegen/galaxea-cdr-tests.log`.
The test launcher emitted a subprocess ResourceWarning; the reported process
was no longer live when checked after the run.

`demo_galaxea_messages.py` passed using the actual module output subscriptions.
It printed base x coordinates `0.100000`, `0.199980`, and `0.299900`, the complete
lidar TF chain, and exact stamps `1700000000223456789` through
`1700000000423456789`. Transcript:
`build/message-codegen/demo/evidence/galaxea-messages.txt`. No robot hardware or
ROS was required. This validates the connection's generated-message boundary;
remaining downstream whole-body/navigation consumers and hardware acceptance
are not claimed complete.

### Whole-body adapter generated streams

TransportWholeBodyAdapter now declares generated JointState, Imu, and
MotorCommandArray types. Commands receive an explicit wall-clock header. Five
focused tests pass for disconnected behavior, preservation of complete feedback
when a short frame arrives, and XYZW-to-WXYZ IMU conversion. The adapter and demo
pass mypy.

The hardware-free demo passed on LCM and independent loopback Zenoh sessions,
exchanging two-joint feedback, IMU, and full hybrid motor commands. Transcript:
`build/message-codegen/demo/evidence/whole-body-transport.txt`. The automatic
subprocess E2E check exercises the same assertions and cleanup. The initial demo
Zenoh factory incorrectly passed a leading slash to the raw transport; it now
uses the same `dimos/` topic mapping as the production transport factory.
No production topic compatibility path was added.

### G1 and shared coordinator joint-state cutover

G1WholeBodyConnection now uses generated MotorCommandArray, JointState, and Imu
messages. It constructs one explicit integer-nanosecond arrival header for each
feedback sample, shared by the joint and IMU messages. Unitree WXYZ is explicitly
copied to ROS XYZW. Every command array must contain all 29 joints before any
DDS command is written. Existing damping-first soft start, mode-machine, and
CRC behavior remain covered by the command tests.

The shared coordinator, tick-loop outputs, velocity/trajectory consumers, and
IK task joint states now use generated JointState. IK command snapshots use
explicit deep copies; generated sequence names are compared as lists. Internal
coordinator snapshot timestamps remain floating seconds, converted explicitly
to Header.stamp at publication. G1/GROOT and Galaxea coordinator transport maps
and declared joint ports now reference the same generated types. Other legacy
pose/trajectory consumers remain outstanding.

**12 G1 tests and all 369 tests in `dimos/control` passed.** The narrower 138-test
routing/joint-state suite and 60-test IK suite passed before broadening to the
full directory. Pinocchio 4.1.0, pin-pink 4.3.0, and QP dependencies were installed
from the local cache; ONNX Runtime 1.24.1 was installed to collect the GROOT task
tests. Mypy passes on 12 changed production/blueprint/demo modules.

Both human demos passed: G1 printed 29-joint samples with q0 `0.0`, `0.1`, `0.2`
and exact stamps `1700000000123456789` through `1700000000123456791`; the running
mock coordinator printed left `[0.1, 0.2]`, right `[0.3, 0.4]`, and the merged
four-joint state with matching tick stamps. Transcripts:

- `build/message-codegen/demo/evidence/g1-messages.txt`
- `build/message-codegen/demo/evidence/coordinator-joints.txt`
- `build/message-codegen/g1-wholebody-cdr-tests.log`
- `build/message-codegen/control-all-cdr-tests.log`

These checks exercise synthetic hardware boundaries and real coordinator ticks;
no physical G1 control or complete robot blueprint acceptance is claimed.

### Generated base-motion adapter and coordinator commands

TransportTwistAdapter now uses generated Twist/PoseStamped and reads nested
position/quaternion fields through the geometry yaw helper. The coordinator and
GROOT command contract, plus Go2/GROOT transport maps, use generated Twist.
Existing enable, stop, one/two/three-axis mapping, and odometry copy-isolation
behavior remain covered.

**373 checks passed** (four base-adapter cases and all 369 control tests), plus
one subprocess E2E that runs the base demo on both LCM and independent Zenoh
sessions. Mypy passes on six changed production/blueprint/demo modules.
The demo received odometry `[2.0, -1.0, pi/2]`, observed the generated velocity
command `(0.5, -0.2, 0.3)`, and observed a zero Twist after disable.

Logs: `build/message-codegen/base-control-cdr-tests.log`,
`base-transport-cdr-e2e.log`, and
`build/message-codegen/demo/evidence/base-transport.txt`. No hardware motion was
performed. Legacy benchmark/trajectory/pose consumers outside this adapter path
remain pending in the broader runtime cutover.

### Manipulation planning consumes generated JointState

Planning-group projection/target normalization, joint-space validation, planning
protocol/model declarations, and RobotStateMonitor now use generated JointState.
Projection and normalization explicitly copy the source Header instead of
inventing a new timestamp. RobotStateMonitor preserves that header when syncing
its reordered state into the world and invoking callbacks; arrival-time freshness
tracking remains separate. Generated sequence data is converted explicitly at
the NumPy joint-space boundary.

**47 tests passed** across planning groups, planning specs/model validation, and
the state monitor. Added checks cover negative/epoch source stamps, header copy
isolation, reordered positions/velocities, incomplete feedback, and stopped
monitor behavior. Six production/demo modules pass mypy. The yourdfpy 0.0.60
dependency was installed to run model-validation tests rather than omit them.

The hardware-free planning demo passed: generated CDR feedback selected left-arm
positions `[-0.2, 0.3]`, normalized target `[0.1, 0.5]`, validated joint limits,
and retained source nanoseconds `1700000000123456789`. Logs:
`build/message-codegen/planning-joints-cdr-tests.log` and
`build/message-codegen/demo/evidence/planning-joints.txt`.

Planner backend implementations, manipulation module entry points, and the
interactive manipulation visualizer still have legacy message consumers. These
checks do not claim the full manipulation stack is converted.


### Planner backend JointState and PointCloud2 self-filter cutover

All planning backend JointState imports now use generated values, including
Drake/Jacobian/Pink IK, RRT and RoboPlan, world synchronization, path utilities,
and trajectory parametrizer inputs. Dictionary constructors became explicit
keyword construction; the parametrizer copies generated inputs with deepcopy.
Array/list conversions are explicit at numerical and sequence API boundaries.
Pose and trajectory message types remain part of the pending cutover.

The robot self-filter now consumes generated PointCloud2 and TransformStamped.
A separate selection helper retains complete point records, including arbitrary
fields, field counts, point padding, and byte order. It removes row padding when
forming an unorganized result and preserves the source Header. Clear masks copy
the source stamp into a world-frame header. Transform matrices come from an
explicit geometry helper.

**253 tests passed with no skips** across the complete planning subtree and the
geometry/point-cloud helper suites. Installed pinned RoboPlan 0.6.0 and Drake
1.49.0 allowed actual backend tests to run. Added selection tests cover both byte
orders, custom array fields, point padding, header preservation, copy isolation,
empty output, and invalid masks. Nineteen production/demo modules pass mypy.

Both human-facing terminal demos passed: `demo_planning_backends.py` printed five
CDR-decoded, collision-checked joint waypoints in a real Drake world;
`demo_self_filter.py` printed three changing arm positions, one retained point
per capture, and the original nanosecond stamp on both outputs. Logs:
`build/message-codegen/planner-backends-cdr-tests.log`,
`planner-backends-mypy.log`, and
`build/message-codegen/demo/evidence/{planning-backends,self-filter}.txt`.

These local results do not claim that manipulation entry points, legacy poses
and trajectories, interactive visualizers, or the full repository are converted.


### Generated trajectories from planning through execution

Trajectory generators, parametrizers, previews, execution splitting, joint and
planar-base tasks, coordinator RPC contracts, and the legacy threaded controller
now use generated ROS2 JointTrajectory and JointTrajectoryPoint values. Waypoint
times are Duration fields; validation compares normalized integer nanoseconds,
including adjacent nanoseconds beyond floating-point resolution. Interpolation
and duration calculation live in `dimos.msgs.trajectory`. Position-only joint
commands are accepted with zero velocity feed-forward. Joint/base splitting
preserves all optional point fields and the original Header. Anchoring a running
trajectory also preserves its metadata and optional fields.

Execution feedback now uses generated `dimos_msgs/msg/TrajectoryStatus` with
Duration elapsed/remaining fields and a wall-clock Header. The internal enum
references generated wire constants, and wire state comparisons use equality.
The three handwritten trajectory codecs were deleted, with no legacy imports
remaining in Python source. The typed test spy uses the generated decode API and
coordinator tests use canonical schema names.

The remaining manipulation JointState consumers (module, SDK, operator, Viser,
and controllers) now use generated values, explicit copies, and list conversion
at sequence API boundaries. Model-state reordering preserves the incoming Header.
Snapshot reprs still expose joint values for interactive inspection.

Validation:

- **968 tests passed** across all manipulation and control tests, trajectory
  helper tests, the transport demo subprocess test, and LeRobot's host module
  contract tests. No skips. This includes real Drake/RoboPlan and Viser tests.
- **Three Galaxea planar-preview tests passed**, including moving its mock base
  through generated trajectory execution.
- **Five process-level coordinator E2E tests passed on Zenoh and the same five on
  LCM**: RPC startup, execution, published feedback, cancellation, and dual-arm
  execution. No coordinator or pytest processes remained afterward.
- **26 focused checks passed** after the final header-preservation and demo edits.
- **28 production/demo/test-support modules passed mypy.**
- The isolated LeRobot runtime's joint/trajectory inputs and corresponding tests
  were updated and syntax-checked. Its full runtime suite is still pending; the
  separate torch/lerobot environment is not installed here. Its legacy image API
  also remains part of the broader image-consumer cutover.

`demo_trajectory.py` passed on LCM and independent loopback Zenoh sessions. It
transmitted 51 generated points, preserved source stamp
`1700000000123456789`, printed changing commands from `[0, 0]` to `[0.3, -0.2]`,
and received generated COMPLETED status with zero remaining duration. The demo's
synthetic clock exercises the actual task without moving hardware.

Logs: `build/message-codegen/trajectory-cdr-tests.log`,
`trajectory-cdr-mypy.log`, `trajectory-planar-preview-tests.log`,
`trajectory-blueprint-tests.log` (Zenoh), `trajectory-blueprint-lcm-tests.log`,
`trajectory-final-focused-tests.log`, and
`build/message-codegen/demo/evidence/trajectory.txt`.

Local setup added the pinned optional xacro, Viser, MuJoCo, websocket-client, and
pytest plugins needed for the existing suites. These results do not establish
full-repository CI or completion of the remaining pose/image/viewer cutover.

### Generated pose, grasp, and path consumers (runtime cutover in progress)

Manipulation planning, FK/IK, preview, dispatch, the SDK, and Viser now use
nested generated Pose/PoseStamped values. Matrix/Euler/local-offset operations
live outside the generated classes. Robot state snapshots retain the source
Header atomically; FK-derived poses and their TF edges retain exact source
nanoseconds. Static mount publication copies its configured transform before
setting the publication time.

GraspCandidate and GraspCandidateArray now come from the generated DimOS schema
package. The two handwritten pickle codecs are removed. The heuristic and
isolated GraspGenX adapter accept generated clouds, preserve their source
headers, and return generated ranked proposals. Pick-and-place validates finite
scores and valid pose geometry before issuing a gripper or motion command.
The GraspGenX tests use a mocked inference backend, not a loaded GPU model.

Base path followers, benchmarking, and coordinator scalar/stamped-twist commands
use generated messages. SVG and Rerun scene rendering reads generated points,
poses, paths, detection boxes, and occupancy grids. The occupancy origin rotation
is applied in both renderers. Existing occupancy borrowing and block reduction
helpers remain in use; renderers validate physical resolution separately.

Human-facing demonstrations:

- `demo_planning_backends.py`: real Drake/RRT, five FK tool poses and CDR TF edges;
  the first source stamp is `1700000000.123456789`.
- `demo_grasp_proposals.py`: a four-point CDR cloud becomes a CDR ranked grasp at
  `[0.5, 0, 0.15]`, with downward approach and unchanged nanoseconds.
- `demo_path_following.py`: 41 generated path waypoints and CDR Twist commands
  drive the existing base simulator to `arrived` in 38 ticks. The SVG in
  `build/message-codegen/demo/evidence/path-following.svg` overlays reference
  and executed paths.

Validation logs are under `build/message-codegen/`: `pose-combined-tests.log`,
`pose-space-tests.log`, `pose-combined-mypy.log`, `pose-blueprint-lcm-tests.log`,
and `pose-blueprint-zenoh-tests.log`. The combined manipulation, control, geometry, grasp, object, and transform suite
passed 1080 tests. Both coordinator E2E runs passed all five tests. The
renderer/occupancy/view suite passed 61 tests, including actual
headless Rerun SDK logging and rotated-grid geometry.

This checkpoint is not an accepted stage-4 completion. The broad type check of
127 modules still reports four errors at legacy point-cloud boundaries: the
older grasp provider, the scene-to-pick cloud interface, and cloud-derived
occupancy rendering. Their producers must be converted; no compatibility reader
or fallback was added. The temporal object-registration suite also fails
collection because the optional Hydra/segmentation environment is absent.
Focused generated-object CDR and spatial-deduplication tests pass. Full viewer UI
acceptance and the remaining inventory retain their unchecked OpenSpec tasks.

### Generated cloud and occupancy renderers (runtime cutover in progress)

Cloud-to-map algorithms, inflation, gradient/Voronoi maps, obstacle operations,
and navigation-map composition now use generated PointCloud2/OccupancyGrid values.
Map images and path footprints use generated Image/Path values. Space accepts
CDR point clouds directly in both SVG and Rerun. Packed rgb/rgba conversion handles
FLOAT32 color bits, UINT32, both byte orders, and padded rows.

- `occupancy-renderer-tests.log`: 92 generated mapping, color, view, and renderer
  checks passed, including real headless Rerun SDK output and rotated-grid paths.
- `occupancy-fixtures-tests.log`: 10 existing image/XML fixture checks passed.
  Three gradient image fixtures were regenerated for the ROS float32 resolution:
  0.05 is stored as 0.05000000074505806. A direct comparison reproduced the old
  fixture exactly with float64 0.05 and identified 718 changed pixels with the
  declared float32 value. Seven other fixture comparisons passed unchanged.
- `occupancy-cutover-mypy.log`: all 15 checked production/demo modules passed.
- `demo_occupancy.py`: 1,200 CDR points, 44 occupied cells before inflation and
  208 afterward; exact source timestamp 1700000000123456789 ns retained. SVG
  output is `build/message-codegen/demo/evidence/occupancy.svg`.

These checks do not complete the broader navigation cutover. A* and path
resampling/masking callers and their fixtures still require conversion, as do
remaining perception cloud/image producers. The demo SVG was generated and
validated programmatically; human visual review is deferred.

### Generated A* and path processing (runtime cutover in progress)

A* consumes generated occupancy buffers in both Python and the compiled C++
extension and returns generated Path/PoseStamped values with the source header.
Resampling and path masks use nested ROS fields. Coordinate helpers apply the
full grid-origin transform. Sub-nanocell rotation roundoff no longer assigns an
exact cell corner to the preceding cell, and negative out-of-map positions are
rejected. Resampling writes pose edits back into generated sequences and merges
a numerically duplicated final point rather than reversing the last heading.

- `path-all-tests.log`: 38 checks passed, covering the complete occupancy test
  directory, recorded-map A*, native/Python parity, generated CDR paths, rotated
  grids, mask exclusion, source-header retention, and input isolation.
- Mypy passed on the four changed production modules and the demo.
- The native A* extension was compiled locally; generated-pipeline tests assert
  its availability before exercising the native branch.
- Six path-image fixtures were regenerated. On both recorded input maps, all
  161/107 generated A* waypoints exactly matched the old implementation when
  both were given the declared float32 resolution. Image expectations now also
  reflect generated path coordinates and the corrected final-point handling.
- `demo_grid_path.py`: both planners produced 41 waypoints, resampled to 122
  CDR-decoded poses, with exact source headers and no occupied mask cells.
  `grid-path-python.svg` and `grid-path-cpp.svg` are the human review artifacts.

Goal validation, higher-level global/local planners, and navigation module
wiring still require conversion; this evidence does not accept stage 4.

### Generated replanning navigation module (runtime cutover in progress)

Goal validation, global/local replanning, navigation maps, clearance masks,
position tracking, retry limiting, and the module's stream/spec declarations now
use generated messages. Odometry and clicked points convert explicitly to nested
PoseStamped values. Robot-footprint clearing copies and replaces the read-only
grid buffer. Clearance masks invalidate when map metadata changes.

- `navigation-tests.log`: 67 focused checks passed, including all replanning A*,
  occupancy, and geometry tests plus the two-transport subprocess navigation E2E.
- `navigation-mypy.log`: 16 production/demo modules passed.
- The arrival regression initially failed because an aligned robot at the first
  path point skipped directly to final rotation. It now checks the last path
  point, and the regression requires physical displacement before arrival.
- Cancellation joins the local control thread and sends generated zero Twist;
  its regression verifies the captured thread is stopped.
- Goal-validator fixture coordinates changed by one cell at the float32 map
  resolution. Running the old validator with that resolution reproduced all
  three new coordinates exactly; obstacle-search behavior is retained.
- `demo_navigation.py` passed on LCM and independent explicit-loopback Zenoh
  sessions: each run reached (2.815, 1.000) for a (3, 1) goal within the configured
  0.2 m tolerance, with 38 commands and 21 published path poses. Source timestamp
  1700000000123456789 ns survived map processing and path publication.
  `navigation-lcm.svg` and `navigation-zenoh.svg` are the review artifacts.

This verifies the navigation module with synthetic transport inputs, not a full
robot blueprint. Other navigation implementations, map producers, and perception
consumers still need conversion before stage 4 can be accepted.

### Generated voxel and cost-map producers (runtime cutover in progress)

CostMapper and the voxel accumulator now consume generated PointCloud2. Both
voxel implementations read declared fields through the shared point-cloud
helper; obsolete wrapper/Open3D conversion functions are removed. Generated
voxel output preserves the latest source stamp exactly, including zero stamps,
and initial-safe-radius changes replace an independent occupancy array after
releasing the borrowed view. The safety disc respects the map-origin rotation.

- `map-producer-combined-tests.log`: 88 checks passed across voxel/cost-map
  producers, cloud-to-occupancy, replanning navigation, occupancy operations,
  geometry, and the expanded two-transport subprocess E2E.
- `map-producer-mypy.log`: seven production/demo modules passed.
- Packed CPU and Open3D-on-CPU backends passed union/column-carving and missing
  point checks. CUDA hardware behavior was not exercised.
- The cumulative `demo_navigation.py` now sends 576 source points through CDR,
  accumulates 576 voxels, runs the real CostMapper module to produce a 62×62
  grid, and drives the navigation module to arrival on both LCM and independent
  explicit-loopback Zenoh sessions. Exact source nanoseconds survive the pipeline.
  Terminal output is in `map-navigation-demo.log`; existing navigation SVGs are
  regenerated by the expanded demo.

The VoxelGridMapper memory-stream wrapper's generated declarations are updated,
but runtime import remains unverified here: the existing memory module imports
optional PyTorch, which is absent. The old recorded voxel tests still depend on
LegacyPickleStore/Go2Moment fixtures; replacing those recordings remains part of
the full cutover. These limitations do not affect the verified direct voxel →
CostMapper → navigation transport demo, and do not complete stage 4.

### Generated basic follower and odometry history (runtime cutover in progress)

The basic path follower, public navigation protocol, and OdometryHist now use
generated messages. TF lookups retain the generated PoseStamped shape. History
uses integer nanoseconds for publication throttling and copies source headers
and nested poses into generated Path output. Its Rerun override constructs the
archetype directly, retaining the actual Z coordinate.

- `history-combined-tests.log`: 97 checks passed across follower/history, map
  producers, navigation, occupancy, geometry, and the two-transport module E2E.
- `history-mypy.log`: four production/demo modules passed.
- Nine focused follower/history tests cover TF mount composition and lookup
  throttling, generated velocity/arrival/empty-path behavior, one-nanosecond
  publication intervals at epoch time, copy isolation, bounded history, frame
  overrides, replay reset to zero, and Rerun path coordinates.
- `demo_odometry_history.py`: fixed-step simulation reached x=1.700 within the
  basic follower's 0.3 m tolerance, emitted zero Twist on arrival, and retained
  35 history poses stamped 1700000000123456789 through 1700000000123456823 ns.
  The generated Rerun line retains z=0.25 m. The SVG review artifact is
  `build/message-codegen/demo/evidence/odometry-history.svg`.

Exploration and patrol consumers still require generated-message conversion.
This checkpoint does not complete the runtime or viewer acceptance gates.

### Generated frontier exploration (runtime cutover in progress)

Wavefront exploration now consumes generated OccupancyGrid/PoseStamped/Bool and
publishes generated goals. Coordinate conversion uses the full map origin;
frontier centroids retain the map plane's height. Stop goals preserve the
current odometry header. Perimeters smaller than one cell no longer admit
empty frontier groups.

- `frontier-combined-tests.log`: 101 checks passed across frontier exploration,
  follower/history, map producers, navigation, occupancy, geometry, and the
  cumulative two-transport navigation E2E.
- `frontier-mypy.log`: the explorer and demo pass mypy.
- Four frontier regressions verify translated/rotated map centroids, original
  pose headers on stop, copy isolation, and an actual exploration-loop goal
  decoded from generated CDR with the map's exact frame and nanoseconds.
- `demo_frontiers.py`: selected (5.00, 4.75) m at the known/unknown boundary;
  stamp 1700000000123456789 ns retained. The review artifact is
  `build/message-codegen/demo/evidence/frontiers.svg`.

Patrol and other remaining consumers still need conversion. This checkpoint
does not complete the full exploration-to-robot blueprint or stage 4 acceptance.

### Generated patrol routing

The patrol module and all three routers now consume generated occupancy maps and
nested poses. Coordinate conversion uses the shared map-origin helpers, and
selected goals retain the map frame and integer timestamp. The existing office
coverage fixture now constructs generated clouds and maps directly.

- `build/message-codegen/patrol-tests.log`: **9 passed**, including six generated
  CDR cases covering all routers with ordinary and rotated/translated maps, plus
  all three existing office-map coverage tests.
- Mypy passed for 11 checked source files (patrol production code and demo).
- `build/message-codegen/patrol-demo.log`: all routers produced five CDR goals;
  three reviewable SVGs were written under `demo/evidence/patrol-*.svg`.
- This verifies goal selection and coverage calculations. It does not claim
  patrol lifecycle or hardware motion acceptance, or completion of stage 4.

### Generated movement manager

The click-to-goal relay and teleop/navigation velocity arbiter use generated
PointStamped, Twist, and Bool. Clicks retain their source header; cancellation
headers use integer wall-clock nanoseconds. Component scaling uses explicit
keyword fields. Existing cancellation behavior is unchanged.

- `build/message-codegen/movement-patrol-tests.log`: **14 passed**, comprising
  five movement tests with CDR-decoded outputs and nine patrol checks.
- Mypy passed for the movement module and its demo.
- `demo_movement.py` printed the preserved click timestamp, teleop suppression,
  and navigation resumption and completed its assertions.

### Generated 2D visual-servo controller

VisualServoing2D uses generated CameraInfo's lowercase `k` intrinsic matrix and
returns generated Twist values. Five parameterized checks passed for forward,
stationary, reverse, turning, and invalid-width detections after CDR decoding.
The terminal `demo_visual_servo.py` passed all four visible scenario assertions;
mypy passed for controller and demo. Person-follow and security-demo callers,
and their detection/cloud pipelines, remain part of the unfinished coordinated
runtime cutover; these controller checks do not establish their acceptance.

### Generated detection-cloud filters

Height, statistical, radius, and visibility filters now consume generated
PointCloud2, CameraInfo, and TransformStamped. Open3D performs geometric
selection; original point records are selected by index so custom fields are
not discarded when reconstructing the output. Four checks passed, verifying
CDR round trips, exact source headers, field definitions, and byte-equivalent
selected records carrying custom point IDs. Mypy passed for filters and demo.
`demo_cloud_filters.py` passed both visible 101-to-100-point scenarios.
Detection3DPC and its higher-level consumers still require conversion; these
results establish filter behavior, not acceptance of the person-follow pipeline.

### Generated 3D detection projection and target calculation

Detection3DPC and its batch projection wrapper now accept generated depth,
clouds, camera calibration, and stamped transforms. Depth projection preserves
the depth stamp and target frame; cloud projection selects original records.
Centroid poses use the cloud header. Open3D bounding calculations are explicit.
DetectionNavigation now consumes these generated values and produces generated
Twist commands using the shared quaternion helper.

Seven projection/filter checks passed, covering translated depth unprojection,
image/behind-camera rejection, bounding extents, robust target selection, CDR
velocity output, and field-preserving filters. Mypy passed for four production
modules. `demo_detection_projection.py` passed and printed the expected world
centroid (11.5, 1.5, 2.0) with stamp 1700000000123456789 ns. The 2D detection image
API and person-follow/security callers remain unfinished; no full pipeline or
model-inference acceptance is claimed.

### Generated 2D bounding-box image and wire boundary

Detection2DBBox now crops generated image views, annotates an independent BGR
copy, and emits generated vision_msgs/Detection2D with the original image header.
Class identifiers use the ROS string field on the wire. Eight combined generated
bbox/projection/filter checks passed. The bbox case checks crop pixels, RGB/BGR
conversion, source immutability, integer timestamp preservation, and decoded
identifiers. Mypy passed for bbox/base and the earlier projection demo.
`demo_detection_bbox.py` generated the annotated review PNG and passed its CDR
assertions. The model-backed existing bbox fixture was not run; its generated
image shape assertion was updated, and model/image fixture conversion remains
part of the remaining perception work.

### Generated detection collections

ImageDetections and ImageDetections2D use generated images and Detection2DArray;
array headers preserve the source image stamp and frame. Person/segmentation
constructors now derive their application-level floating timestamp from generated
headers. Shared `image_to_bgr` produces an independent drawing array and serves
both individual and collection annotation. The annotation demo now exercises the
collection's CDR array and rendered output.

Nine combined generated detection/projection/filter checks passed. Mypy passed
for the shared image helper, collection base, and bbox. Broader checking remains
incomplete: Torch typing is unavailable and the person class has pre-existing
unparameterized NumPy annotations. Model-backed inference and person-follow
lifecycle acceptance are still outstanding.
