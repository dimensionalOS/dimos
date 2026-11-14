# Topic adaptation on slow links

PLAN_2c adds a single SDK controller, per-viewer congestion-abort feedback,
and direct-video bitrate and in-place encoder changes. The implementation
follows the fixed ladders, thresholds and cooldowns in the plan. Measurement
gates below remain failures when missed; adaptation does not remove the
underlying QUIC congestion-control limit.

## Measurement

The harness's `V3-topic-adapt` runs one Chromium page with a synthetic 720p30
camera, 100,000-point cloud at 10 Hz, raw odometry at 30 Hz, reliable events
at 100 Hz, and guarded command receipts at 20 Hz. Camera priority is 3 and
cloud priority is 5. JPEG and direct H.264 use separate runs. N5 steps the
viewer leg to 25% of measured payload demand at 20 seconds and restores it
at 80 seconds. N2 and N6 retain the harness's cellular and lossy-WAN profiles.
Each mode also has a ten-minute unshaped loopback point.

Measured on 2026-11-05, on an AMD Ryzen 9 9950X (32 logical CPUs), Linux
`7.0.0-38-generic`, with Chromium and software `libopenh264` on one host.
The harness is local branch `paul/feat/topic-adaptation-scenario`, commit
`8831255`; no harness GitHub PR is planned. The dimOS implementation commits
are `eb0cc13379`, `e5c7835cc9` and `346dcc7dbf`, based on
`paul/feat/topic-video-sdk` at `06c0f71b38`.

The full trial began before those changes were committed; `run.json` honestly
records `eb0cc13379` plus a dirty working tree. Its loaded SDK bundle is
preserved as `measured-sdk.js`, SHA-256
`9ef9b83d68ddeb79951989f7029775e981c74aa782d6025c360f0ba7130210e4`.
The final branch additionally fixes retrying rejected automatic changes
without another stall signal and discarding queued old video packets at
reconfiguration. Those edge cases have focused tests; this trial is not a
performance measurement of those final corrections.

All paths below are under
`/home/p/pro/dimensional/dimos-websdk-benchmark/results/`:

- Full trial: `topic-adapt-full/run.json`, `topic-adapt-full/report.html`.
- Raw points, settings windows and worker logs:
  `topic-adapt-full/scenarios/V3-topic-adapt/`, with `jpeg-N5.json`,
  `jpeg-N2.json`, `jpeg-N6.json`, `jpeg-N0.json` and matching `h264-*` files.
- Per-route gate summary: `topic-adapt-full/adaptation-summary.json`.
  This is recomputed from immutable point records with the final harness
  scorer: it uses the worse of the two route p95s, rather than pooling a
  sparse cloud with many camera frames, and excludes unshaped direct RTP
  from shaped-media gates. The original run report remains available.
- Setup smoke: `topic-adapt-smoke3/report.html`; final rebuilt-branch smoke:
  `topic-adapt-final-smoke/report.html`.
- Guard status diagnostics: `topic-adapt-guard-diagnostic/` and
  `topic-adapt-guard-ready/`, including their `h264-N0.json` point records
  under `scenarios/V3-topic-adapt/`.
- Kernel impairment: `topic-adapt-manual2/manual.json`, with exact namespace
  and qdisc commands plus `provider.log` and `relay.log` alongside it.

The proxy shapes WebTransport, including JPEG and cloud, but cannot shape
direct RTP. Shaped H.264 rows therefore gate cloud media and control traffic;
direct-video media remains diagnostic. A separate manual provider namespace
run uses a kernel `tc netem` qdisc and has no automated performance gate.

## Results

All shaped points failed at least one required gate. No performance threshold
was relaxed. N5 demand was 60.99 Mbit/s for JPEG and 50.10 Mbit/s for H.264;
the caps were respectively 15.25 and 12.52 Mbit/s. N2 adds 75 ms one-way
delay, 25 ms jitter, 1% loss and a 10 Mbit/s cap. N6 adds 100 ms one-way
delay, 1% loss and a 100 Mbit/s cap.

| Path | Profile | Command receipt p95 | Camera / cloud display age p95 after settling | Event interruptions / gaps | Verdict |
|---|---|---:|---:|---:|---|
| JPEG | N5 | 96 ms | 2,590 / 4,351 ms | 0 / 0 | Failed: age, churn, restoration |
| JPEG | N2 | No receipts | 140,211 / 139,006 ms | 0 / 0 | Failed: receipts, age, settling, churn |
| JPEG | N6 | No receipts | 141,422 / 154,465 ms | 3 / 2,572 | Failed: receipts, age, reliable continuity, churn |
| H.264 | N5 | No receipts | Unshaped / 5,443 ms | 0 / 0 | Failed: receipts, cloud age, churn, recovery order |
| H.264 | N2 | No receipts | Unshaped / 151,777 ms | 1 / 2,004 | Failed: receipts, cloud age, reliable continuity, churn |
| H.264 | N6 | No receipts | Unshaped / 137,677 ms | 2 / 1,903 | Failed: receipts, cloud age, reliable continuity, churn |

Under JPEG N5, the cloud first stepped down 1.11 seconds after the cap,
before the camera. Recovery prioritized the camera. Neither path violated
its per-handle cooldown, but JPEG N5 reached five changes per handle in a
rolling 30-second settled window (limit two), and complete restoration took
63.69 seconds (limit 60). H.264 N5 first reduced the cloud after 1.55 seconds
and restored settings after 16.17 seconds, but its cloud recovered first
while the unshaped camera stayed at the request; the stated recovery-order
gate therefore did not pass.

The first trailing second at the required floor arrived after 1.00 seconds
for JPEG N5, 17.50 for JPEG N2, and 4.00 for JPEG N6; cloud-only H.264 values
were 1.00, 5.00 and 4.50 seconds. Crossing that floor once did not establish
sustained fresh delivery: the age and continuity failures above still apply.
N2/N6 ended at 1 Hz cloud with stride 8 and no optional fields, and at either
1 Hz JPEG (320×180, quality 38) or 5 Hz video (640×360, 200 kbit/s).

Missing command receipt series are failures, not zero latency. The probe
records `guard_not_ready` after the publisher disarmed and does not silently
re-arm it. N2's nominal added RTT is already 150 ms, above its specified
100 ms receipt gate; the original gate is retained. Similarly, the fixed
one-rung-per-ten-clean-seconds recovery rule cannot restore more than six
outstanding rungs in sixty seconds.

| Ten-minute N0 repeat | Camera FPS | Camera age p95 | Camera payload rate | Rung changes | Result |
|---|---:|---:|---:|---:|---|
| JPEG | 28.91 | 37 ms | 13.37 Mbit/s | 0 | Media and zero-change gates pass; 21 superseded guarded commands recorded |
| H.264 | 29.97 | 65 ms | 2.49 Mbit/s | 0 | Media and zero-change gates pass; command receipts failed |

Both N0 repeats had zero reliable interruptions/gaps. JPEG had 23 ms command
receipt p95. Its extra probe-error check remains failed for the superseded
commands, even though the specified media gates passed. H.264's guard was
already not ready during the measurement window, with no command receipts.
A focused follow-up retained publisher status across warmup and measured
`guard_expired` one second after readiness, while visible and focused. Arming
only after media negotiation completed reproduced the same failure. The final
harness retains that later arming and status history, with no automatic
re-arming. The cause of this expiry is unresolved; these records cannot
attribute every guard failure to link impairment or establish command latency
for those points. Repeating a long H.264 trial with the still-expiring guard
would not repair the missing command measurement.

These measurements do not establish the slow-link acceptance guarantees.
The fixed controller adapts, but this trial still suffers stale media and
loss of reliable/control progress on the lossy profiles. Transport behavior
and the conflicting timing gates need separate follow-up before claiming
those guarantees.

## Direct RTP namespace trial

An unprivileged user namespace owns two private network namespaces joined by
a veth pair. The provider, camera and screen worker occupy one side; the
relay and browser occupy the other. No host interfaces or qdiscs change.
A local control tunnel accommodates the relay's localhost ephemeral
certificate; RTP crosses the veth directly. The provider's egress gets
100 ms delay, 3% random loss and a 750 kbit/s cap at ten seconds, removed at
sixty seconds. The trial lasts two minutes.

Video acknowledged 2.5 → 1.5 → 0.875 Mbit/s, then 640×360 at 350 kbit/s,
with the first reduction 3.8 seconds after impairment. The severe shared
egress bottleneck also interrupted the source and exhausted the peer's ICE
retries. Requested settings returned 20.6 seconds after restoration, but
video did not resume: the final peer state was rejected with `ice_failed`.
This is a recorded diagnostic failure, with no automated media gate. Track
events and missing re-offer-gap endpoints remain in the raw record.

## Validation

- 260 web tests; 617 SDK tests; SDK type
  checking and production bundle build.
- 452 protocol/provider tests and 33 real-relay self-hosted tests.
- Four Chromium/Firefox browser cases against the rebuilt SDK, including
  unchanged track identity and continued frame callbacks after a sole-viewer
  settings change.
- Mypy: 1,276 source files; repository pre-commit checks. The global Deno
  formatting hook was replaced with a focused format/check of all 19 changed
  TypeScript files because the installed formatter rewrites unrelated HTML.
- Four new harness scoring tests pass. The complete harness unit suite has
  17 passes and one existing legacy-writer import failure:
  `webbench.robot` still imports `RELIABLE_MAX_UNACKED_BYTES` from the current
  `wt_client`, which no longer exports it. V3 does not use that adapter.

## Counter semantics

The current latest-stream sink can retain an already accepted stream until
its 500 ms cleanup even on a healthy link. Counting every cleanup as congestion
would continuously degrade loopback. `pong.aborted` therefore counts bound
aborts and expiry with unaccepted bytes, excluding normal accepted-stream
cleanup, per-key replacement and disposal. This matches the distinction
between aborted and expired sends on the legacy channel path. The real-relay
slow-viewer test verifies that blocked receive credit increases the counter
while control remains connected.
