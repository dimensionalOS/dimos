# Fixture routing and generated-message regression coverage

Core/worker, TF, browser, detector, MCP and offline manipulation fixtures now
use generated ROS-shaped messages. The test spy encodes CDR and its topic names
use `package/msg/Type`. Browser paths preserve the same numeric points; map
click assertions inspect nested PointStamped fields. The offline IK fixtures
retain the original rotation composition and numeric acceptance thresholds.
G1 debug command tools use generated CDR without running a driver.

Verified locally:

- 12 core baked-host tests and 13 worker tests passed.
- The existing bidirectional TF IO integration test passed (48 unrelated tests
  deselected), using generated TransformStamped inside TFMessage.
- 35 isolated Space rendering tests passed.
- 109 offline CDR spy/browser-codec/framing tests passed.
- 15 offline Pi adapter tests passed after fixing the exact CI assertion below.
- All four executable blocks in `docs/usage/lcm.md` passed. The guide now
  describes generated CDR and raw LCM transport instead of rich LCM overlays.

The previous published HEAD `11bae55835e56c33f3940a9563cf15cfc81993b2` has a
successful message-codegen run 36760272088. Its main run 36760267791 completed
with an ARM failure: 6158 passed, 228 skipped and one failed. The failed Pi
recording-export test read `obs.data.position.x`; the fixture already stores
generated PoseStamped and requires `obs.data.pose.position.x`. This batch
preserves its selected-stream, timestamp and numeric-content assertions.
Other Python lanes and lint were cancelled, not passed. Rust, native, Web and
docs jobs passed. A new exact-HEAD full CI result is still required.

These checks did not execute browser UI, external inference, full simulator,
hardware, or old recording fixtures. Remaining rich-message implementations,
legacy tests/assets and the dimos-lcm dependency still require retirement;
stages 5–6 and human viewer acceptance remain open.
