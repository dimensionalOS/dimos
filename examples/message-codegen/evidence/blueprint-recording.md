# Native blueprint recording and replay

Verified locally on 2026-09-30 using the current runtime batch on top of
`f4a269ce4bc766a1ff876ed2b6f5ac432e6d3900`.

- 33 recorder, SQLite path, stamped-message and TF regression tests passed.
- Strict scoped mypy passed for RustRecorder and the SQLite replay helper.
- The actual Python -> C++ -> Rust -> Python Zenoh relay produced three
  custom LineSegments3D samples, three 640x480 RGB images, and three PoseStamped
  samples. Native edits incremented line weight twice; image bytes were exact.
- Rust recorder reported received=9, written=9, encode_errors=0 and exited cleanly.
- MCAP had CDR channels and embedded ros2msg schemas. Independent rosbags Jazzy
  decoding succeeded for every row; all three streams contained samples 0,1,2.
- After all producers stopped, DimOS ReplayModule emitted every recorded value
  with byte equality and per-stream order preserved.

The retained local artifact is
`build/message-codegen/demo/evidence/blueprint-current-zenoh-v3.mcap`; terminal
log is `/tmp/cdr-blueprint-recording-v3.log`. Earlier failed artifacts were
preserved. This demo uses installed built-in custom messages; arbitrary installed
external-message capture has separate acceptance requirements.

The first recorder run exposed inherited TF wiring being passed despite
`record_tf=False`. RustRecorder now filters native launch topics to declared
recording streams; regression tests cover TF enabled and disabled. Another run
exposed float-second replay treating 1 ns spaced epoch timestamps as static.
The demo now uses realistic 100 ms intervals while checking exact nanoseconds;
sub-float-resolution scheduling is not claimed.

No hardware ran and host tuning was check-only. Default-interface LCM, human
Foxglove/Rerun inspection, full dependency retirement and final installed-package
acceptance remain open. This evidence does not mark stages 5 or 6 complete.

## Follow-up regression batch

The published parent `f4a269ce4` codegen workflow 36763634296 succeeded. Main
workflow 36763630343 ended cancelled after ARM reported 6170 passed, 228 skipped
and one documentation-branding failure. Both offending spellings in
`docs/usage/lcm.md` are repaired; other Python jobs were cancelled, not passed.

Follow-up local checks: 15 Joy/Header/LineSegments3D/branding tests and 53
stamped-covariance/time tests passed. Migrated suites retain numeric payloads,
empty and large sequences, covariance patterns, copy independence, explicit
clock and datetime conversions, and add independent ROS CDR checks. Legacy
convenience constructors, custom string formatting, inheritance, Path-based
line packing and implicit timestamps are intentionally retired. Generated ROS
time overflow now reports an explicit ValueError; both int32 boundaries and
values immediately outside them are covered. Scoped mypy passed time.py.

## Future cross-stack compatibility note

Parent review on 2026-09-30 identified EpisodeStatus in the independent open
chain #3854 -> #3855 -> #3921 -> #3931 -> #3942. It is used by typed live Out,
collection recorder input/source timestamp extraction, offline episode boundaries
and TUI RPC. That type is absent from this CDR checkout. Its eventual integration
would need a coordinated generated CDR design if that proposal is adopted; a JSON substitution would not satisfy
the typed CDR recorder contract. The user has not selected the integration design.
No code or commits from that chain were imported or changed. This is an explicit
future compatibility note, not a regression introduced by this branch.
CDR is not a prerequisite for merging that separate stack: it targets current
main and remains independent. No merge is authorized for this CDR draft.

## Image and value-type retirement follow-up

Local verification after `0bac7f454`: 52 Path/basic-covariance checks, five
CameraInfo calibration checks, and 58 generated-image/compression/detection/heavy-import
checks passed. Strict mypy passed image.py, Rerun message_helpers.py and both
manual message tools. The original JPEG/JXL mean-error and compressed-size
thresholds, lossless PNG/JXL depth, codec effort comparison, camera calibration
values, Rerun box geometry/labels and path sequence behavior are retained.

PNG/JXL encoding and decoding live in external functions over generated values.
Source headers and 16UC1 depth units survive, including big-endian inputs. Raw
64FC1, 16SC1 and 32FC3 views remain supported; unsupported compression types are
rejected explicitly. The local venv was missing the existing lockfile dependency
imagecodecs 2026.6.26; its 27.5 MiB wheel was installed without changing project
dependencies or system packages.

Image file/selection tests use deterministic temporary pixels and real sharpness
calculation in virtual-time windows. The manual pickle tool retains only its
raw-array input path, converting to generated images after loading; no old typed
message decoding was added. Manual publishers and hardware tools were not run.

## Geometry and map follow-up

Local checks: 42 wrench/geometry, 12 odometry, 78 point-cloud/viewer and 32 occupancy
tests passed. Strict scoped mypy passed geometry.py, pointcloud.py, occupancy.py
and the Rerun helper. Force/torque arrays and axis-aligned finite cloud bounds
are external functions; generated types stay value containers. Point-field tests
retain intensity, zero offset_time, tag, line and all original overlap scenarios.
Occupancy tests retain unknown/free/occupied counts, threshold filtering, origin
yaw, coordinate conversion and obstacle-preserving reduction. Borrowed views
remain read-only; tests mutate declared message data explicitly. ROS float32
resolution is compared with numerical tolerances instead of assuming Python
float64 storage.

## Generated geometry regression continuation

The Vector3, Quaternion, Pose, Transform, and Twist suites now use generated
values and explicit external math. Non-unit Hamilton products/inverses, zero
errors, rotated local offsets, frame composition, independent ROS decoding,
and original numerical tolerances remain covered. Constructor coverage now
checks explicit nested fields, deep-copy isolation, covariance lengths and
layout, zero/default timestamps, exact nanoseconds, and rejection of retired
polymorphic fields. Stamped values are distinct from their payloads.

Offline message-directory validation: **614 passed** with only
`tf2_msgs/test_TFMessage_lcmpub.py` excluded. The attempted whole-directory run
also produced 614 passes, but that real multicast test failed host LCM self-test
and thread teardown; this remains an environment-dependent gate, not a pass.
No host routing/firewall configuration was changed. Strict scoped mypy for
`dimos/msgs/geometry.py` and repository hooks passed.

Published commit `0bac7f454bcd2f61766c0246e38a40016c62746c` has terminal success
for main CI 36765815202 and codegen CI 36765824915. Self-hosted lanes were
skipped. These results do not cover subsequent local commits until their own
exact-HEAD workflows complete.

## Python dependency retirement and remaining consumers

The 47 obsolete Python message implementation files have been removed after
migrating their remaining Python imports. The root project now declares raw
`lcm-dimos-fork` directly and no longer depends on `dimos-lcm`. The task venv's
installed `dimos-lcm` was also uninstalled: **624 offline message/voxel tests
passed without it**. The lock removes only `dimos-lcm` and its unused
`foxglove-websocket` dependency; no remaining package versions changed.
Re-resolution required restricting the existing optional `a750-control` wheel
to Python 3.12, its only published/locked ABI, instead of requesting it on 3.10/3.11.

The retired typed-pickle voxel regression fixture is now deterministic CDR
geometry, with exact voxel counts at three resolutions, repeated-ingestion
invariance, numerical coordinates, and range preservation. Raw PLY/image
occupancy fixtures remain supported. Full local test collection reached 6555
tests with 16 collection errors for absent optional dependencies (coacd, h5py,
open_clip, plotext, reportlab, requests_mock, trimesh, ultralytics, yourdfpy),
not obsolete message imports. This is not a full-suite pass.

Remaining examples and the optional LeRobot runtime use generated image,
pose, point-cloud and UInt32 button fields. The runtime copies image pixels
before retaining an observation. Its own scoped mypy configuration passes,
but its behavioral suite still requires the absent LeRobot environment.
The virtual robot's pure unicycle integration passed a numerical check;
the C++ controller compiled against the installed CMake message package and
raw LCM, without being run. The obsolete Lua and TypeScript LCM-codec demos
are retired explicitly; Lua generation is deferred and the browser CDR SDK
is the supported web path. Transform documentation passed 5 executable Python
blocks; eval documentation passed all 6 blocks (4 no-result, 2 with results).

Outstanding dependency gate: the M20 onboard DrDDS/Zenoh C++ bridge still
fetches/uses old message headers. The local environment lacks the vendor DrDDS
SDK, and its `sensor_msgs::msg`/`nav_msgs::msg` types collide with generated C++
type names. A verified vendor boundary/codec migration is still required;
no board build, deployment or robot operation has been performed. Historical
manual replay fixtures and final viewer/transport acceptance also remain open.

The shared Go2 moment fixture now takes an explicit CDR SQLite path; its tests
create generated recordings instead of resolving the old pickle dataset name.
The manual memory import/query tool now copies 1024 deterministic CDR samples
into an isolated destination. Its original >1000 counts, >10-second duration,
pagination, ordering, overlap, lazy decoding and pose assertions remain.
All **18 tool checks passed**, including cached CPU CLIP embedding/search with
model network access disabled; no new weight download was needed. These
synthetic image results validate the pipeline, not real-scene recognition.

The separate real detection fixture `unitree_go2_lidar_corrected` is absent
locally and its LFS archive is 1,212,727,745 bytes. Regenerating it while retaining
its semantic assertions is still an explicit outstanding fixture gate.

### DimSim standalone codec audit

The remaining browser/Deno `@dimos/msgs` consumers now use pinned Foxglove CDR
codecs and a standalone full-schema bundle exported from canonical generated
messages. RGB publishes raw RGBA Image, depth publishes 16UC1 Image, and lidar,
odometry and cmd_vel use PointCloud2/PoseStamped/Twist with ROS 2 channel names.
The existing LCM transport and LC02 WebSocket envelope remain. JPEG stays only
in eval/sidebar previews. No legacy decoding fallback was introduced.

Verified locally: Vite production build; complete Deno CLI type check with a
frozen lock preserving unrelated original dependency versions; three socket-free
codec tests (large image envelope, all velocity components, malformed input);
canonical-schema drift pytest; strict Python exporter typing; pre-commit checks.
Four Deno-encoded payloads (Twist, PoseStamped, Image and PointCloud2) decoded in
both generated Python and independent rosbags with exact values/source stamps;
a generated Python Twist decoded in Deno with exact command components.
Full browser/physics simulation acceptance remains unverified. These checks
neither operate hardware nor prove host multicast or end-to-end rendering.
