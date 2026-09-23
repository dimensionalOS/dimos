# Navigation CDR verification

### Generated holonomic tracking and command limits

Converted the holonomic tracking law, command limiter, and trajectory samples
to generated Pose/Twist/Vector3. Quaternion yaw uses the shared geometry helper;
samples retain independent copies of caller-owned values.

Verification: five focused tests passed (frame rotation, feedforward, position
and yaw correction, one-sided damping, speed/acceleration bounds, copy isolation,
and CDR round-trip output). Mypy passed for all three production modules.
`demo_holonomic_tracking.py` ran 200 synthetic ticks and reached `(2, 1)` with
`0.00000002m` position error, printing intermediate decoded velocity commands.
This verifies the tracking core only; path-controller and live module conversion
remain part of the ongoing coordinated runtime cutover.

### Generated holonomic path follower and module

Converted PathDistancer, HolonomicPathController, and DanHolonomicTC to generated
nested PoseStamped/Path, Twist, and Bool messages. Removed their legacy message
imports and copy constructors. Velocity estimation subtracts integer nanosecond
timestamps before conversion and owns its previous-pose snapshot.

The three closed-loop follower cases (straight-line deceleration, right-angle
speed cap, initial rotation), two run-envelope cases, and four module lifecycle
cases passed. Module input test transport now CDR-round-trips each value. A
separate regression passed for a one-nanosecond interval at epoch 1700000000
and mutation of the caller's previous pose. Mypy passed for all three changed
production modules. The threaded `demo_holonomic_path.py` arrived at `(0.926, 0)`
within its 0.08m goal tolerance and printed a final zero command. Network
transport behavior is outside this focused demo's scope.

### Generated local-planner gating and smoothing

DanLocalPlanner and its replan gate now consume generated PointStamped,
PoseStamped, and Path. Seven tests passed with CDR-decoded inputs, covering
progress-based commits, fresh/stale goals, cancellation, empty-path stop,
disabled gating, and smoothing with exact sec/nanosec header preservation.
Mypy passed for the production module. The cumulative holonomic path demo now
runs this gate first: it prints 10 resampled poses and suppression of a duplicate
replan, then the threaded follower arrives at `(0.926, 0)` and publishes zero.

### Generated MLS Python boundary and TF start relay

MLSPlan now reads generated PointCloud2 via the shared layout-aware XYZ helper
and emits generated Path/PoseStamped with the cloud's exact Header. The start
relay consumes generated TFMessage and emits generated PoseStamped.

Built the actual Rust/PyO3 extension with maturin 1.13.3, release mode, offline
from cached dependencies. Installation succeeded after allowing uv's cache lock
outside the sandbox. Seven tests passed against that extension: terrain planning,
missing-pose skipping, robot-height offset, no-route output, TF chain composition,
missing-chain suppression, and retry throttling. Mypy passed for both production
modules. Tests check CDR path headers at `1700000000.123456789` and decoded relay
poses. No tests were skipped for a missing native backend.

`demo_mls_transformer.py` ran successfully: TF start `(-2, -2, 1)`, 3,600 terrain
points, 900 voxels, and 18 decoded path poses ending at `(2, 2, 0)`. The exact
cloud timestamp survived into the output path. This is a native planning and
serialization demo; it does not establish network or hardware performance.

### Generated MLS visualization adapters

Planner visualization now reads generated PointCloud2 fields through shared
layout helpers and renders explicit generated LineSegments3D endpoints/weights.
Three tests passed against actual Rerun archetypes: clearance filtering,
missing-intensity coloring, raised nodes, weighted edge colors, empty geometry,
and unchanged source messages after rendering. Mypy passed for the adapter.

The native MLS terrain demo successfully wrote `mls-planner.rrd` with terrain,
path nodes, and edges. `rerun rrd verify` accepted the recording. This is
headless recording verification; interactive viewer acceptance remains pending.

### Navigation evaluator on generated SQLite streams

Converted recording registration and suite frame counting to generated
PointCloud2/Odometry, nested pose fields, and shared point-cloud/pose helpers.
The evaluator's 14 tests passed after replacing its recording fixtures with
generated CDR messages. These include recorded frame alignment, rotation,
trajectory loading, suite generation, and causal/final evaluation paths.
Mypy passed for recording and case handling.

`demo_navigation_recording.py` wrote and reopened actual SQLite streams and
verified three sensor points rotated by 90 degrees and translated along the
recorded trajectory: `(0,1,0)`, `(1,1,0)`, `(2,1,0)`. Loaded odometry yielded
2m total distance. The temporary database was removed automatically. This
validates newly written recordings; historical dataset replacement remains a
separate outstanding part of the full migration.

### Offline planner rendering and ray-tracing boundary

Converted `plan_rrd` to generated cloud/TF types and shared point-cloud helpers.
Extracted named TF rendering and colormap annotations from legacy message classes
into `visualization/rerun/message_helpers.py`. Converted the upstream RayTraceMap
transformer to generated input/output clouds, preserving the source stamp and
removing the Open3D allocation previously used only to construct output messages.

Built and installed the actual ray-tracing Python extension offline. Five native
transformer tests and two offline-rendering tests passed; mypy passed for all
three production modules. The terrain demo now prints the generated ray-traced
map and writes it alongside TF edges, terrain, and the planned route. The updated
RRD passed `rerun rrd verify`. Full execution of `plan_rrd` against historical
recordings remains unverified until those datasets are replaced.

### Ray-tracing CLI and combined regression checks

Converted the remaining raytrace_rrd CLI to generated PointCloud2 fields and
nested TransformStamped values. The recorded-navigation demo now invokes the
actual CLI in a subprocess on newly written CDR lidar/TF SQLite streams. It
processed three frames, wrote `raytrace-cli.rrd`, and Rerun verified the file.
Mypy passed for the CLI after annotating the existing normal-length array.

The combined dannav, evaluator, MLS, and ray-tracing test selection passed all
75 tests, including both compiled native Python backends. A search found no
legacy geometry/nav/sensor/TF message imports in navigation or ray_tracing Python
files. This search establishes only that scoped import cleanup, not completion
of the repository-wide migration or historical dataset replacement.

### Generated TF dispatch in the live Rerun bridge

The bridge now recognizes generated TFMessage directly. Named transforms are
logged even with axes disabled; enabled axes use a separate generated-message
frame-tree renderer. Six bridge checks passed, including generated CDR dispatch
with axes enabled/disabled, reparenting cleanup, and existing bridge configuration
checks. Mypy passed for the extracted frame-tree renderer.

The terrain demo now sends its CDR TF message through the actual bridge callback
before recording terrain and the route; the resulting RRD passed verification.
This exercises dispatch without starting the viewer or a transport subscription.
Camera/image and other built-in bridge conversions remain outstanding.

### Generated camera and image bridge

Converted bridge image/calibration dispatch to generated CameraInfo, Image, and
CompressedImage with separate pinhole/image adapters. Calibration pairs by
header.frame_id in either arrival order. Depth units are explicit (16UC1 in
millimetres, 32FC1 in metres). JPEG/PNG compressed formats set the required
Rerun media type; unsupported formats fail explicitly.

Twelve camera/TF/config checks passed. A warning exposed missing compressed-image
media metadata in the initial run; fixed it and reran all six camera checks with
Rerun warnings treated as errors. That rule is now part of the test file. Mypy
passed for the adapters. The camera demo then logged raw/compressed color and
both depth representations without warnings, and its RRD passed verification.
Interactive viewing and bridge transport subscriptions remain unverified here.

### Generated 3D detections in the Rerun bridge

Added a separate generated Detection3DArray renderer and wired it into bridge
conversion. It preserves box centers, half sizes, orientations, class labels,
and detection IDs, including empty arrays. Updated the existing bridge fixture
to generated CDR messages. The two detection checks passed after fixing a
missing adapter import and an unintended early return caught by the dispatch
test. Nine camera/TF checks
also passed during this increment, and mypy passed for the adapter module.

The camera demo now sends a labeled generated detection through the bridge in
addition to images and calibration. Its final successful run produced an RRD
that passed verification. This is callback/serialization verification, with
interactive viewer inspection still outstanding.

### Generated point clouds through the shared bridge

Added generated PointCloud2 dispatch with shared XYZ/RGB layout helpers. The
adapter filters non-finite coordinates with the same mask for packed colors,
uses explicit height colors for colorless clouds, and never mutates the source.
The padded big-endian FLOAT32-packed-RGB test verifies color alignment, CDR
round-trip, frame attachment, and source-byte preservation. Empty and height-only
clouds are also covered. Both focused tests and the combined 13 generated
cloud/camera/TF/detection bridge tests passed; adapter mypy passed.

The terrain demo now sends its 3,600-point cloud through the bridge callback.
The native planning demo completed and the resulting RRD passed verification.
This remains headless callback verification, not interactive viewer acceptance.

### Generated navigation rendering in the bridge

Added generated PointStamped, PoseStamped, Odometry, and Path dispatch through a
separate adapter. Poses and odometry carry their declared parent frame in the
Rerun transform. Points and paths use bridge frame attachment; paths preserve the
existing 0.5m display lift without changing message coordinates. Empty paths emit
empty geometry for clearing.

Five focused tests passed with Rerun warnings as errors, covering each message
kind, CDR round-trips, coordinates, parent frames, source immutability, and empty
paths. Adapter mypy passed. The native terrain demo routed its planned path
through the shared bridge and its RRD passed verification. Network subscriptions
and interactive viewer acceptance remain outside this check.

### Combined bridge and viewer environment verification

Installed the exact repository-pinned `dimos-viewer==0.32.0a2` into the local
verification environment. Two initial viewer binary-discovery checks failed
because the package was absent; after installation and venv activation, all
29 generated adapter, bridge configuration, and viewer integration checks passed.
These viewer checks cover installation and mocked launch/lifecycle behavior,
not interactive visual acceptance. No dependency manifest changed.

Mypy passed for the bridge, message adapters, and frame tree after giving the TF
loop distinct variable names to avoid an incompatible archetype assignment.

### Generated occupancy grids through the bridge

Added generated OccupancyGrid dispatch to a separate textured-quad adapter.
The quad uses the complete origin pose, including rotation and elevation; its
texture preserves row orientation and default free/occupied/unknown colors.
Two focused tests passed with Rerun warnings as errors, checking a 90-degree
rotated grid, exact texture bytes, frame attachment, source immutability, empty
maps, and malformed dimensions. Bridge and adapter mypy passed.

The native terrain demo now sends a small generated occupancy grid through the
bridge alongside its cloud and path. The completed demo's RRD passed verification.
Interactive rendering acceptance is still outstanding.

### Live LCM bridge verification

Added a bounded headless demo that calls the real bridge start lifecycle,
creates its gRPC server, subscribes through typed LCM, publishes a generated
160x120 image on a unique topic, and verifies delivery after bridge rendering.
The demo passed, and Rerun verified its recorded output. The subprocess regression
also passed with 30-second bounds on the demo and RRD verification. No transport
or viewer startup behavior is mocked in this demo; no GUI was opened.

Bridge initialization now imports colormap registration from the separate
visualization helper instead of the legacy PointCloud2 class. All eight viewer
integration checks passed after this change. This establishes the LCM headless
path only; Zenoh and interactive visual acceptance remain outstanding.

### Live bridge on both transports

Extended the headless live bridge demo and regression to LCM and Zenoh. The
Zenoh variant uses two independent session pools and an explicit loopback TCP
connection. A first run rejected the demo's leading-slash LCM topic name;
corrected the fixture to a valid `dimos/vb/...` Zenoh key. Both bounded subprocess
tests then passed, including real bridge startup, CDR image delivery, shutdown,
and verification of each transport's separate RRD output.

Scouting/gossip are disabled in the explicit Zenoh fixture. This is not evidence
that the previously observed default discovery issue is fixed, nor is it
interactive viewer acceptance.

### Viewer control WebSocket

The viewer's click, velocity, and stop handlers now emit generated `PointStamped`
and `Twist`. Click timestamps convert integer milliseconds directly to ROS time;
nullable 2D click coordinates remain zero. No legacy message imports remain in
`dimos/visualization` Python sources.

```sh
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_viewer_controls.py
```

Observed: a real loopback WebSocket delivered a map click at `(1.5, 2.5, 0)` with
stamp `1700000000.123000000`, forward/yaw velocity `(0.5, 0.8)`, and a stop with all
six velocity components zero. All received values were encoded and decoded as
CDR. This exercises the control protocol without requiring a graphical viewer.
The six server tests passed; production-file mypy passed. The invalid-JSON test
now proves that a subsequent stop is delivered instead of relying on sleeps.

### Command-center generated message boundary

`WebsocketVisModule` now consumes generated pose/path/occupancy messages and
publishes generated click goals, exploration flags, and both velocity forms.
Costmap inflation and gradient use the shared occupancy view; JSON retains
compressed cells and now reports the origin quaternion's yaw. Local subscribers
receive velocity commands through the same publication path as configured
transports.

```sh
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_command_center.py
```

Observed: printed browser state for robot `(2, 3)`, path to `(5, 3)`, and a 2×2
compressed costmap; the click produced a CDR-round-tripped world goal `(5, 3, 0)`.
This demo invokes registered handlers directly; it does not claim browser or
Socket.IO network acceptance. Two focused tests passed, covering generated
commands, stamped velocity, pose/path state, exact decompressed cells, rotated
map origin, and unchanged input bytes. Production-file mypy passed.

### Shared interfaces and VLM image requests

Control and mapping protocols now reference generated Twist and OccupancyGrid.
The VLM agent, its RPC protocol, and stream tester reference generated Image;
model requests use the shared JPEG encoder and explicit base64 content instead
of methods on legacy values. The unused manipulator identity-transform factory
was removed along with its legacy imports.

```sh
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_vlm_image.py
```

Observed: `build/message-codegen/demo/evidence/vlm-request.jpg` is a 160×120 image
with red left and green right halves, also inspected directly. The demo exercises
CDR decode and the actual agent request assembly with a stubbed model; no external
inference is claimed. Twelve VLM-boundary and existing spec tests passed. Coverage
includes stream and RPC image requests, JPEG colors and dimensions, unchanged
input bytes, and the no-image response. Mypy passed for the six changed production
files after annotating the request content list.

### Navigation skill goals

The navigation skill's image/odometry streams and goal construction use generated
messages. Tagged locations now convert quaternion orientation to the Euler angles
required by RobotLocation, fixing the previous storage of quaternion xyz as
roll/pitch/yaw. Tagged and semantic goals use explicit generated pose fields.

```sh
PYTHONPATH=.:build/message-codegen/demo/cpp/build .venv/bin/python examples/message-codegen/demo_navigation_skill.py
```

Observed: a generated odometry pose at `(1, 2, 3)` was tagged as `desk`, retrieved,
and emitted as a CDR map goal retaining roll/pitch/yaw `(0.2, -0.3, π/2)`.
Memory and navigation RPCs were stubbed; no inference or robot motion occurred.
Two focused tests passed for the tagged orientation round trip and semantic-map
goal construction/rejection. Production mypy passed. Existing agent integration
test message annotations were converted; those inference tests were not run.
