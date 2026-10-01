# Remaining camera and plain export consumers

ZED TF publishing/tracking uses generated TransformStamped and TFMessage,
with exact stamped headers, extracted inverse/composition helpers and the
existing camera/optical frame conventions. Three offline callback tests
passed with only SDK responses mocked; no SDK Camera or start() was called.
GStreamer emits generated BGR Image values owning pixels after buffer unmap.
One mocked callback test passed, retaining invalid/pre-2000 timestamp rejection.
The original float-second/offset calculation is retained; this is not a claim
of arbitrary nanosecond preservation for that existing source path.

The camera calibration CLI subscribes to generated Image and uses the explicit
BGR helper. All 22 existing synthetic/mocked tests passed, including YAML
round-trips, raw topic delivery, fisheye calibration and timeout cleanup.
The fixed FlowBase mount is a generated Pose with identical numeric values.
Strict scoped mypy passed for all five changed production camera/CLI files.

Plain agent recording export accepts generated Image/PointCloud2, preserving
source frames and explicit lossless PNG/XYZ/RGB CSV/JSON output. Six existing
agent-boundary/export tests passed without starting an agent or models.
Strict scoped mypy passed for the export implementation.

## Approved control and documentation continuation

The combined camera, export, PGO, WebXR, episode-monitor, coordinator and mock
arm regression run passed 192 tests on Python 3.12.14. This includes GTSAM
optimization and the OpenArm bounded 100-step IK test; no physical robot was
connected. Scoped strict mypy passed all eight control production files.

WebXR serves generated PoseStamped/Joy schemas and accepts explicit CDR data
frames with channel/type metadata. Buttons remain a local bit-field helper;
wire ports publish generated UInt32. Six added checks cover schema advertisement
and rejecting mismatched types, wrong encodings, unknown channels, truncation
and trailing bytes before control handlers. Existing deadman, disconnect and
single-client assertions remain enabled. The actual browser loadCommandSchemas
and sendCommand functions ran in Node with the already installed, pinned
@foxglove/rosmsg 5.0.5 and @foxglove/rosmsg2-serialization 3.1.2 dependencies on
the trusted recovery machine. Python decoded the returned framed CDR values,
including exact nanoseconds, nested poses, analog axes, digital buttons and
sequence numbers. This is offline encoding evidence, not a headset/robot demo.
The browser imports these versions from esm.sh; network access is needed to load
the page dependencies on first use.

Memory index and plotting documentation passed md-babel-py 1.4.0: 9/9 and 6/6
non-skipped Python blocks respectively, using CPU execution and cached official
openai/clip-vit-base-patch32 weights. Both brightness-comparison blocks also
executed successfully. Run from docs/capabilities/memory so relative assets
land alongside the documents. Refreshed results and images describe a new
synthetic CDR recording, not the historical office recording. The old 187 MiB
recording is unnecessary for this walkthrough and was not downloaded. The CLIP
cache occupies about 1.2 GiB because the upstream loader fetched both .bin and
.safetensors weights (roughly 605 MB each). Skipped detector sections were not
executed; their historical screenshots are not new-format acceptance evidence.

The unchanged 100 ms Zenoh sync/async RPC assertion passed in six independent
cold pytest processes locally. Python 3.10's earlier CI failure remains
unexplained; no pre-warming, retry inside the test, or timeout relaxation was
applied. Main CI at e386163c56 terminated cancelled: Python 3.10 had 6316 passed,
82 skipped and this one RPC failure; Python 3.12 was cancelled without a final
test result. Codegen, lint, Web, Rust, native, ARM and Python 3.11 passed.
The next published commit requires its own terminal CI results.

Legacy consumers/dependency retirement, full clean build matrix, human viewer
and cumulative live demo gates remain open. Default-interface LCM multicast
and real hardware gates are not accepted by these offline checks.
