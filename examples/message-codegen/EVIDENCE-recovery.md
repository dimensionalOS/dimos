# Desktop recovery and continued CDR validation

Verified on 2026-09-30 in the dedicated `feat/cdr-runtime-cutover` worktree.
This is a progress checkpoint, not final migration acceptance.

## Preserved history

The recovered local tip `ae00dd3e8c` was clean. CC Desktop had three linear
successors, `87ad59d853`, `08cb4eb4ba`, and `be3832092d`. They were fetched and
fast-forwarded locally without changing the source machine or rewriting commits.
These migrate MCP observation images, keyboard commands and hosted map compression.

## Changes and visible demos

- `scripts/test_message_user_story.sh` generates an application-local
  `story_msgs/msg/DeviceReading`, builds Python/C++/Rust, and runs a real Python
  module handler followed by native CDR file consumers. Output returns sequence
  42, value 23.5, label `new-local-type/python/cpp/rust`, preserving nanoseconds.
- `demo_video_stats.py` converts browser JSON to the explicit generated
  `dimos_msgs/msg/VideoStats`, records and reopens SQLite, and displays the exact
  integer counter 4294967297. It removes its temporary database automatically.
- Video telemetry consumers now import the generated type. The old positional
  Joy carrier and its legacy codec methods are removed. Invalid unsigned metric
  values fail at the JSON boundary with a descriptive ValueError.
- The unused legacy `lcm_msg_type` discovery function is removed. Tests resolve
  generated types by `package/msg/Type`; old dotted names do not import codecs.
- `docs/development/messages.md` provides message-authoring, Python module,
  C++/Rust codec and native module, packaging and recording/replay user stories.
  Its runnable shell commands were executed and local source/doc links checked.

## Executed validation

Use the generated demo extension on PYTHONPATH and the activated project venv.
The restored native dependency prefix is `build/message-codegen/install`.

- Generated/built the complete demo definitions including the new telemetry type.
- 79 Python tests passed across generated video stats/type discovery, memory CDR
  codecs, MCAP store, Rust recorder configuration, MCP images, keyboard and map
  compression. Tests used `--noconftest --import-mode=importlib -o addopts=''`.
- 3 native recorder E2E tests passed over explicit local Zenoh TCP: Rust SQLite
  and MCAP artifacts read through Python, plus TF capture and Python replay.
  The two LCM CLI cases were deselected. Tests used the actual local Cargo-built
  `target/debug/dimos-memory-recorder`; no result from the separate main-branch
  MCAP compatibility thread is used as CDR evidence.
- `cargo test -p dimos-memory-recorder --lib`: 16 passed.
- `cargo test -p dimos-module --lib`: 146 passed.
- `cargo build -p dimos-native-module-examples --offline`: passed after fetching
  the lockfile dependencies during explicit setup/build.
- Ruff check/format, shell syntax and diff whitespace checks passed for this
  change. Mypy passed the two changed helper modules with generated typing on
  MYPYPATH. This is not a full-repository mypy claim.

Terminal and build logs remain ignored under
`build/message-codegen/demo/evidence/recovery-*`; user-story CDR files and its
terminal capture live under `build/message-codegen/user-story/evidence/`.
Standard setuptools egg-info was generated to expose the checkout's declared
`dimos.messages` entry point; simply putting an uninstalled extension on
PYTHONPATH does not register an installed provider.

## Continued hosted command cutover

Hosted Go2 command and telemetry boundaries now use generated TwistStamped,
Twist, PoseStamped and Bool, matching the converted Go2 driver. Drive ordering
uses integer source nanoseconds, including frames one nanosecond apart. Stale
and future rejection, finite-velocity validation, limits, idle suppression,
release-edge stopping and E-STOP remain covered by mocked driver tests. Invalid
ROS nanoseconds are rejected before telemetry processing. A runnable
`demo_go2_command_cdr.py` prints clamped drive and navigation fields and asserts
zero driver calls. Both production modules passed mypy with generated stubs.

The combined phone/Go2/hosted telemetry suite passed 53 tests. Separately, 135
generated geometry, time, buffer, point-cloud, trajectory, camera-info and
occupancy helper tests passed. Neither count represents the entire repository.
Unitree import dependencies were installed in the project venv for these mocked
regressions; no device connection, microphone or audio capture was started.

## CI environment and browser-fixture repairs

Both workflows were manually dispatched on `dd9a5e385df9b4966953c50e142bb695f38d3beb`.
The generator job passed its codec conformance, raw fragmented LCM exchange,
and transport unit tests before a helper demo failed because PyYAML was absent.
The workflow now explicitly provisions the lockfile's PyYAML 6.0.3. That helper
demo passed in a fresh minimal project venv.

Main CI's lint, docs and Rust binding jobs failed during editable installation
because their Fast CDR preflight was absent. They now run the same pinned setup
script already used by the test jobs. A real no-dependency editable install
succeeded in an isolated venv in 1m48s. All jobs that perform `uv sync` now have
an explicit CDR setup step. This validates installation, not the full jobs.

Web CI passed its Deno/SDK/cockpit checks and seven browser E2E cases; retired
PoseStamped inputs and an old replay database caused the remaining failures.
The typed-channel test now uses generated fields and expects the CDR schema
contract. Go2 cockpit/SDK tests use a deterministic 1.3 MB replay fixture with
three generated streams, 720 frames each, exact source nanoseconds and changing
RGB pixels. Images use lossless `lz4+cdr`; the old large LFS fetch is removed
from the web job. The full fixture was written and reopened locally with type,
stamp, frame-count and pixel assertions. Generated fixture mypy and Ruff passed.
61 focused schema/MCAP/teleop checks passed. Browser E2E and full CI acceptance
for these repairs remain to be verified on the next published commit.

## Exact-commit CI and native bake follow-up

At `1e7705ab3cf98edc35c72bd4c3bcceaa2d243b49`, manually dispatched
[message-codegen run 36691321793](https://github.com/dimensionalOS/dimos/actions/runs/36691321793)
completed successfully in both standalone and Jazzy-reference jobs. This includes
installed wheel/sdist/CMake/Cargo consumers and offline Foxglove/Rerun decoders.
The retained MCAP contains 150 messages: five generated channels, 30 each.
The viewer evidence is copied locally under ignored
`build/message-codegen/ci-evidence/1e7705ab3/`. This proves offline decoding,
not authenticated Foxglove UI acceptance.

[Main CI run 36691317959](https://github.com/dimensionalOS/dimos/actions/runs/36691317959)
completed with failures; its web browser job and native C++/Rust job succeeded.
Lint lacked generated typing stubs; the next batch generates them explicitly
and provisions pinned native-build typing tools. The emitter also had a real
loop-variable type collision, now fixed. Full-repository migration errors remain
to be resolved after these environment errors. Five changed production/demo
files passed targeted mypy; generated-interface positive/negative tests passed.

The native bake E2E exposed old registry type names and LCM test publishers
against a CDR decoder. All five native module registry entries now use the same
canonical names as their generated Python ports. Macro validation accepts the
canonical spelling. The actual Python-to-baked-Rust graph passed all three E2E
cases over loopback-only Zenoh, including real surface-map publication and
suppression, in 17.18 seconds. No hardware or networking settings were used.

Main Python CI collection lacked MCAP/rosbags test tools and offline planner
bindings; its next batch explicitly provisions those. Updating the whole uv
lockfile is currently blocked by the existing `a750-control==0.1.1` wheel
availability for supported Python 3.11. The runtime manifest and existing
lockfile were preserved; pinned supplemental CI tools are installed explicitly.

CameraMux uses generated images and separate array helpers. Its 21 tests cover
even dimensions, scaling, selection, FPS caps, latency strips, exact source
stamps and independent composite storage. The executable bounded camera demo
produces a 100x36 RGB composite and an 854-byte JPEG while preserving
`1700000000123456790ns`. No camera or encoder process is started.

## Remaining gates

The OpenSpec checklist remains unchanged because these slices do not complete
all stage-4, stage-5 or stage-6 requirements. Old wrappers and consumers remain
in other teleop, robot and perception paths. The public type replacement and
complete old-dependency removal still need coordinated migration.

The C++ SDK dependency blocker was resolved using an isolated project prefix
`build/native-deps/prefix`: official LCM 1.5.1 sources, pinned Zenoh C/C++ 1.10.0
release archives, and the already built Fast CDR prefix. Zenoh archives were
checked against the same SHA256 values used in CI. Their pkg-config prefix
was relocated within the ignored project directory; the actual unstable-API
compile probe passed. No system package or networking setting was changed.
The SDK built and all 77 CTest tests passed. C++ examples built, then
`demo_native.py --backend zenoh` exchanged fields across Python/C++/Rust and
verified a 921600-byte image with its exact source nanoseconds.

The phone browser command path now advertises generated schemas and dispatches
explicit channel/type CDR frames. Nine Python tests passed, including the real
FastAPI WebSocket endpoint, invalid frame rejection and control gain/yaw math.
A real in-app browser connected to `demo_phone_cdr.py`: five schema-driven
Foxglove CDR frames decoded in Python. The preview stopped its server without
starting a control loop or requesting sensor access. Production phone modules
passed mypy with generated stubs; Ruff check/format passed.

At published commit `dbf5583075154f9a174efe59a794ba5167e6592b`, all CodeQL
language analyses passed. Neither `ci.yml` nor `message-codegen.yml` has a
pull-request run for #4257 among the latest 100 runs. GitHub reports mergeability
unknown and exposes only the PR head ref; the missing merge ref may be relevant,
but a cause has not been established. The desktop browser was unsigned in. Existing authenticated GitHub CLI on
CC Desktop subsequently enabled manual dispatch without credential changes. LCM default
multicast self-test is blocked on this host; no networking settings were changed.
Full blueprint/viewer demo, final installed packaging matrix, authenticated
Foxglove UI evidence, and exact published-commit CI acceptance remain pending.
No hardware commands, model inference or model downloads were used.


## Generated-consumer cleanup checkpoint

The next local batch migrates model/detector/embedding image boundaries,
recording headers and TF payloads, report source-time calculations, G1 mount
composition, and generated video rendering. Public generated messages remain
plain schema values. OpenCV/Open3D/Rerun imports are deferred per the existing
codebase check, without suppressing that check. Old typed-transport test inputs
now use canonical generated types. Empty codegen namespace initializers are
removed; bundled standalone generator precedence still passes its existing test.

Affected offline helpers, WebSocket protocol, camera mux, reports and G1 math:
195 passed, one desktop viewer binary test excluded because `dimos-viewer` is
not installed locally. Transport/package/structure checks: 57 passed. Strict
checks passed for the migrated recorder/report, image/projection, detector
modules, person tracker, patrol consumer, time adapter, G1 and viewer helpers.
Model weights and hardware behavior are not validated by these checks.

The original timestamp self-hosted test accidentally acquired/extracted
`unitree_office_walk` during the explicit-file run (ignored data directory about
2 GB); the download was not requested in advance. Its backpressure/alignment
fixture now uses synthetic capture events, retaining its existing assertions.
All 21 timestamp/projection tests pass without replay assets. Existing cached
data is retained; subsequent tests avoid automatic large-asset acquisition.

Exact published `febed89cec435bc2b853b4d163f033a6fd7dc8f2` message-codegen run
36694469475 succeeded. Main CI run 36694465187 completed with ARM tests
5940 passed / 54 failed / 229 skipped / one teardown error; other Python jobs
were fail-fast cancelled. Native, Rust, Web, Linux and macOS builds succeeded.
Lint reported 188 errors in 61 files and md-babel failed on remaining old
examples. These results predate this cleanup batch and are not final acceptance.


## Offline RGB-D and namespace-entrypoint repair

Generated PointCloud2 helpers now project rectified RGB-D with scaled camera
intrinsics, integer-depth units, invalid-depth filtering and packed RGB fields.
Transforming a cloud retains declared fields, endian, organized dimensions and
all point/row padding; finite values beyond the target field range are rejected.
Support-plane preparation consumes these generated values before its existing
Open3D downsampling/RANSAC. Open3D fitting itself was not executed locally.
Image benchmark fixtures now construct generated RGB8 values.

Offline image views, calibration, geometry and point-cloud tests: **143 passed**.
Strict checks passed for point-cloud helpers, support-plane and benchmark data.
Standalone codegen tests with PYTHONPATH unset: **24 passed, one existing skip**.
The codegen and MCAP shell gates use python -m pytest so namespace-package
imports retain the checkout root. This fixes the observed collection failure of
published 6547f9c message-codegen run 36704834764 without reducing coverage.

Local full mypy: 178 errors in 78 files (1158 checked); the intersection with
original failing CI file paths is 90 errors in 28 files, down from 96. This
includes unavailable local dependency errors and is not CI acceptance.
Published 6547f9c main run 36704509445 has successful native, Rust, Web and
Linux/macOS builds; lint and md-babel failed, Python test jobs remain running.
WebXR control migration and model-backed memory-doc execution remain held for
specific approval after automatic review rejected those actions. Their existing
uncommitted work is preserved and excluded from this repair batch.


## Static maps, memory consumers and offline Spot viewer conversion

Static occupancy NPY/PNG loading now returns generated OccupancyGrid values,
with explicit header/map-load time, identity origin and ROS cell validation;
object-array loading is disabled. The MuJoCo scene loader and occupancy demo
use the standalone helper. No simulation/control loop or GUI was started.
OSM marker drawing and PNG saving convert generated pixels explicitly, retain
source metadata, and do not mutate cached tile images; its test mocks tile fetch.

SpatialMemory reads nested generated TF fields and RGB/BGR conversions.
TemporalMemory uses generated image/pose streams, quality selection and real
header stamps in its fixtures. Scene-staleness ignores row padding.
Inventory/localization use generated XYZ views and TF fields; source-point
selection retains RGB and other fields when oversized supports split. Native
Open3D clustering, plane fitting and oriented boxes are not locally validated.
Inventory's strict check reports unavailable local torch; no model was loaded.
Spot Rerun converters construct TF/calibration/capture-anchored depth archetypes
from generated messages; offline construction passed without a viewer or robot.

Combined selected offline regression: **195 passed, one external temporal-memory
integration class deselected**. Full local mypy: **161 errors in 73 files**;
original-CI-path intersection: **75 errors in 24 files**. No suppressions added.

Published c690ef58a439b377cb6d9bb8eeb9bdc60e0128f6 message-codegen run
36707053518 completed successfully for standalone and Jazzy reference.
Main run 36707056720 completed fail-fast cancelled: Native/Rust/Web succeeded,
lint reported 93 errors in 30 files, md-babel passed 126/128 blocks, ARM failed
and other Python matrix jobs were cancelled. This consumer batch follows that
run and still requires exact published-commit CI acceptance.


### Generated consumer contract repairs after 201a3053

- Exact published 201a3053 CI: standalone/Jazzy message-codegen run 36709681735 succeeded; main run 36709677922 terminated by fail-fast. Native C++/Rust builds and Web succeeded. Lint reported 70 errors in 21 files; Python 3.11 retry reported 40 failures and three errors. The main workflow is not green.
- Migrated VQA, image-file evaluation and replay benchmark fixtures to generated Image/CameraInfo/PointCloud2/CompressedImage. Image observation encoding lives in the application layer; generated messages remain plain values. JPEG benchmark codec preserves exact source headers, and transport cleanup now also runs on test failure.
- Moved the complete gradient contract test to the generated occupancy suite, preserving obstacle, distance and unknown-cell assertions. Replaced the pointcloud occupancy input with a deterministic synthetic cluster rather than legacy LCM asset loading.
- Explicit CPU voxel backend does not import Open3D; CUDA selection remains unchanged. Added a real accumulation/CDR roundtrip test checking exact nanoseconds, output frame and disposal.
- Local combined offline checks: 154 passed, 5 deselected (CPU voxel, generated occupancy, VQA, evaluation and Zenoh/pickle replay contracts). LCM multicast and large benchmark nodes were excluded; no network settings changed.
- Full memory voxel and Go2 marker blueprint collection is blocked locally by absent torch; legacy occupancy suite collection by absent numba; YOLOe collection by absent ultralytics. Those fixture migrations await CI and are not recorded as local passes.
- WebXR control/ESM changes and full memory documentation execution remain paused after automatic approval rejection. Eight related earlier work-in-progress files remain uncommitted. No model download, hardware execution or approval bypass occurred in this batch.


### Camera, scene and mapping consumers after aecb4d7e7

- Exact aecb4d7e7 message-codegen run 36712561619 succeeded. Main run 36712557915 terminated by fail-fast: Native C++/Rust builds, Rust (including E2E) and Web succeeded; lint reported 69 errors in 20 files. ARM Python reported 6037 passed, 229 skipped, four failures: two PGO generated PoseStamped conversions and two control-coordinator collection blueprint links. The latter remain related to held work.
- ZED/MuJoCo image, calibration and cloud output constructors now use generated messages. Calibration retimestamping copies all fields and nested data. Existing tensor Open3D voxel downsampling moved into a standalone helper with the small-cloud fast path; unsupported fields are rejected instead of lost. No camera, robot, simulation or RTSP connection was started.
- Scene Object storage, aggregation and interfaces use generated clouds/geometry. Native Open3D geometry/filtering remains an algorithm input; generated messages carry the wire values. Exact source cloud nanoseconds survive aggregation. RGB-D alignment uses the existing local TimestampedData adapter.
- World-belief visibility and frame selection use generated values and standalone matrix/distance helpers. PGO correction applies its existing correction matrix to generated Pose values, fixing the old constructor boundary; internal optimizer math still requires later legacy retirement. Map output and mock lidar replay use generated wire messages.
- Local regression set: 90 passed, one optional native Open3D test skipped. This covers calibration copies, generated cloud/object values, visibility inverse transforms, PGO correction order and pose-less passthrough. Production scoped strict mypy: 16 files clean.
- Installed only the repository-locked types-PyYAML 6.0.12.20250915 typing stub (20 KB wheel) into the existing venv; no global software or model assets. Mocked camera/scene suites that import absent mujoco/pyzed/ultralytics await CI; they are not claimed as local passes.
