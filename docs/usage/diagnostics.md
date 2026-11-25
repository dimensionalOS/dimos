# Runtime diagnostics

DimOS diagnostics records where time is spent and which input produced an output. It is opt-in, independent of recording robot datasets, and does not publish robot commands. The backend includes a local collector, CLI, read-only gateway API, Python transport/queue/control hooks, and Rust SDK hooks.

This is an instrumentation implementation, **not an end-to-end teleop qualification**. In particular, camera-driver timestamps do not establish physical exposure time, and adapter publication does not prove that an actuator applied a command. Source publication identity now survives the native gateway encoder/RTP metadata path and browser presentation callbacks. Missing coverage remains explicit.

## Component ownership

This capability is part of the existing platform packages. R1 Pro is one adapter
and a validation target. No separate diagnostics service or Desktop host fork is
required.

| Owner | Code | Responsibility |
| --- | --- | --- |
| DimOS runtime | `dimos/diagnostics/`, stream/worker hooks, native SDK | Message identity, stage timing, bounded incident capture, analysis and qualification. |
| Existing resource monitor | `dimos/core/resource_monitor/stats.py` | Process CPU, memory and IO sampling shared with `dtop`. Diagnostics retains the samples in its incident history. |
| Hardware sensors | `dimos/hardware/sensors/wifi/` | Radio observations, ICMP probes and interval host/interface rates. CLI collection and blueprint modules use these same collectors. |
| Mapping | `dimos/mapping/wifi/` | Optional timestamped join with odometry and an ordinary point-cloud overlay. It does not own transport or process instrumentation. |
| Shared native gateway and browser SDK | `dimensionalOS/zenoh-gateway` | Encode, pacing, delivery, decoding and browser frame identity. Any application can consume these hooks. |
| Existing Python gateway | `experimental/gateway/routes/diagnostics.py` | Read-only status, capabilities, topology, metrics and incident downloads under the normal gateway boundary. |
| Spatial Desktop app | `data/diagnostics.js`, `server/trace.ts` in its app repository | Render/XR observations and bounded uploads through the app's declared routes. It consumes the shared SDK instead of implementing another streaming stack. |

For the integration revisions, review boundaries and remaining qualification, see
[diagnostics integration](/docs/development/diagnostics-integration.md).

## Capture

For an already chosen blueprint, use the resolved run configuration. The coordinator
propagates it to local and remote workers and their native children:

```bash
dimos --diagnostics trace --diagnostics-run teleop-investigation-01 --diagnostics-dir /tmp/diagnostic-captures run BLUEPRINT
```

Leaving `--diagnostics-dir` empty uses each host's normal DimOS state directory.
An explicit path is local to each participating host. Do not use a Mac-only home path for
a Linux worker. Existing hosted runs may use their shared session ID when no explicit
diagnostics run ID is supplied. For standalone processes, set these variables before
launching them, using the same run ID and collection root as their host's workers:

```bash
export DIMOS_DIAGNOSTICS=trace
export DIMOS_DIAGNOSTICS_RUN=teleop-investigation-01
export DIMOS_DIAGNOSTICS_DIR="$PWD/diagnostic-captures"
# Launch the chosen blueprint normally after configuring diagnostics.
```

Modes are `off` (default), `metrics` (bounded counts/histograms, no raw event ring), and `trace` (metrics plus raw events). This does not attach to, restart, or take over a running robot.

For a disposable local test command:

```bash
dimos diagnostics capture --directory ./diagnostic-captures --seconds 60 --run-id example -- python demo.py
```

`capture` owns the supplied child: at the deadline it requests a diagnostics flush/stop, then terminates that child. Do not use it with a daemonizing command. For an ongoing blueprint, use the environment settings instead and control only its diagnostics:

```bash
dimos diagnostics status ./diagnostic-captures teleop-investigation-01
dimos diagnostics mark ./diagnostic-captures teleop-investigation-01 --reason hand-tracking-stalled
dimos diagnostics stop ./diagnostic-captures teleop-investigation-01
```

The last command stops collection, not the blueprint. Status moves through `collecting`, `stopping`, and `stopped`; the final status is written after the final incident save. Starting capture again currently requires a new process/run. These file-backed commands address the chosen host's capture root; run them on each host whose collector should stop or save a marker.

Python, native SDK and native gateway collectors share host reservations (256 MiB per configured host root, at most 64 MiB for any process) and a flock-protected 1 GiB incident retention budget. Use the same root for every collector on a host. Python reserves 16 MiB by default, with half available for the ring; native SDK reserves 16 MiB and gateway 32 MiB. Capture requests a 60-second pre-trigger / 30-second post-trigger window. The byte cap takes precedence: retained coverage, evictions, dropped events, and IO failures are explicit. Python metadata is copied into bounded JSON-only fields; application buffers are rejected. A bounded 128-event ingress queue never waits for disk or a remote collector.

All three collectors checkpoint every five seconds and preserve incident windows on a manual marker or excessive delay. Python also watches input freshness independently of new arrivals. Graceful shutdown flushes evidence; a crash can lose data since the checkpoint. Native modules require a rebuild against the instrumented SDK. Metrics mode retains bounded histograms, not raw payload/event rings.

## Inspect and evaluate

```bash
dimos diagnostics inspect ./diagnostic-captures/teleop-investigation-01
dimos diagnostics topology ./diagnostic-captures/teleop-investigation-01
dimos diagnostics capabilities
dimos diagnostics host --interface wlan0
```

When a blueprint includes `WifiMonitor` or `NetworkSurvey`, their existing
observations also enter the same run's incident capture. Original acquisition
and measurement-window timestamps are retained. No second probe loop is started.
Use [the survey composition](/docs/usage/wifi_coverage.md) to add the optional map overlay.
On that host, do not also start a CLI network monitor with the same probes unless
you intentionally want duplicate measurements. The CLI remains useful on hosts
that do not run the survey, or for additional station/AP counters.

Process history uses the same `ProcessStats` fields and sampling semantics as
`dtop`, including its platform-dependent zero defaults for unsupported counters;
zero IO/PSS alone is not evidence of no work. These samples appear as `host.process`
events in the incident bundle, and the latest sample is exposed by the metrics API.
`survey.measurement` events retain the existing typed survey observations.

The topology is a graph of hosts, processes, module endpoints, and transport channels. A bound subscription is distinguished from an observed publication. Matching topic names alone do not connect two processes: matching sample identities establish observed deliveries. A delivery edge explicitly leaves its physical route unresolved until route/interface/infrastructure evidence is available. The API does not invent access points, switches, tunnels, or relays.

Inspection reports stage percentiles, per-process/channel/task groups, queue delays, control rejection reasons, unmatched spans, observer loss, and sequence anomalies. A sequence gap means **unobserved delivery**; it does not by itself prove packet loss. Intentional latest-value supersession is recorded separately.

Export a single clock domain to a Perfetto-compatible trace:

```bash
dimos diagnostics export ./diagnostic-captures/teleop-investigation-01 trace.json --clock-domain 'domain-from-the-manifest'
```

Multiple unaligned clock domains cannot be combined into one exported timeline. The inspector never subtracts wall clocks to manufacture one-way latency.

Budget profiles explicitly name the stages to evaluate. For example, this is a **sample profile**, not a product latency guarantee:

```json
{
  "schema": "dimos.budgets/v1",
  "stages": {
    "control.ik_step": {"p99_ms": 10, "min_samples": 1000},
    "control.connection_enqueue": {"p99_ms": 5, "min_samples": 1000}
  }
}
```

```bash
dimos diagnostics evaluate ./diagnostic-captures/teleop-investigation-01 budgets.json
```

Exit codes are 0 for pass, 1 for a measured breach, and 2 for inconclusive evidence. A stage-budget pass is scoped to the specified stages; `teleop_qualification` remains `not_established`. Missing stages, too few samples, observer loss, or unmatched spans cannot silently pass.

Path profiles may specify `channels`; every required channel independently meets
its sample count and latency budget. Freshness profiles are checked separately per
process/clock, including missing observations at both ends of an active interval.
An explicit watch disable ends that interval; silence does not. `processes` can
require a particular headset observer, so a laptop cannot supply its coverage.

```bash
dimos diagnostics report ./diagnostic-captures/teleop-investigation-01 report.html --profile budgets.json
```

The resulting HTML is standalone and contains no external scripts or network calls.
It shows recorded network and process topology, measured link/radio evidence, stage
durations, observer coverage and selectable causal message histories. Unknown hops
remain unknown. Reports without a budget profile show observations, never a pass.

## Network and clock probes

The gateway's `GET /diagnostics/clock` exposes receive/send monotonic timestamps. Probe an explicitly selected gateway URL:

```bash
dimos diagnostics probe "$GATEWAY_URL/diagnostics/clock" incident-probes.jsonl --count 120 --interval 0.5 --interface wlan0
```

This is a low-rate HTTP application probe, not an airtime or saturation test. Its RTT includes proxy/HTTP dispatch, endpoint scheduling and network delay. Four timestamps bound clock offset without assuming symmetric paths. Mappings expire and their uncertainty grows with the configured drift bound. Same-boot clock domains can be compared directly; other domains require a valid mapping. If a mapping is missing or expired, the sample age is unknown.

Linux host snapshots include interface counters and available `ip`, `iw` and `tc` observations. Missing commands/permissions are reported. These endpoint counters do not isolate radio airtime, AP queueing, or a hidden relay. AP/controller telemetry and on-path timestamp adapters are still needed for that attribution. A ping or HTTP RTT must not be relabelled as “the Wi-Fi hop.”

## Instrumented boundaries

| Boundary | Current evidence |
| --- | --- |
| Python `Out.publish` | Publication identity and parent sample IDs |
| Concrete Python transports | Publish/callback duration, bound channels and delivery observation |
| Zenoh | Native trace attachment; legacy payload bytes unchanged |
| LCM, ROS, DDS, WebRTC, Cloudflare data channels | Local spans; explicit dedicated traced-carrier option for cross-process identity |
| Rx backpressure and module async handlers | Context handoff, queue age, explicit supersession, handler duration |
| Zenoh wildcard/latest subscription | Context survives its worker queue |
| Cartesian and two-hand IK | Cached input lineage, solver duration, target timeout / feedback limit / invalid result / solver failure reason |
| Joint trajectories | Cached trajectory input lineage, including partial-joint updates |
| Coordinator | Tick lateness, read/compute/arbitrate/route/write stages, selected command lineage and dispatch result |
| Rust SDK | Encode/enqueue, queue waiting, async handler, transport publish and Zenoh delivery spans; parent identity retained |

`control.dispatch` distinguishes adapter acceptance/enqueue from rejection and always has `hardware_acknowledged: false`. It cannot establish motor acceptance or physical response. IK/trajectory provenance is marked partial because measured-feedback, calibration-reference and all vendor-internal dependencies are not yet carried through the full causal graph. Native derived handlers retain context across input queues, output queues, task migration and cancellation. Custom native caches must explicitly retain their input context.

Native gateway captures also identify the last RTP packet leaving the application pacer and packets discarded there. These records join to a source frame only when process, pacer incarnation, SSRC and RTP timestamp match unambiguously. Pacer departure is not a NIC or radio transmission timestamp. Browser captures retain selected ICE endpoint observations and their generations; intermediate AP/backhaul hops remain unresolved.

No control limits, estop behavior, IK optimization, interpolation, recorder policy, or robot command wire formats are changed by these hooks.

## Explicit traced links

Where a carrier lacks usable metadata, both endpoints must opt into the same dedicated link:

```python
from dimos.diagnostics.factory import traced_link
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

transport = traced_link("lcm", "investigation", "operator-right-hand", PoseStamped)
```

Hosted broker channels are directional. Supply an existing `ProviderConfig` plus `publish_channel` and `receive_channel`; a robot publishes on `state_reliable_back` and receives on `state_reliable`, while the operator uses the reverse pair. The wrapper uses a dedicated message magic and run/link namespace on those channels. Both endpoints must negotiate the same type and encoding before application publication. This does not modify legacy command messages or create an operator session implicitly.

Hosted qualification uses a synthetic session created by the fixture, with a separate robot process and an operator using the broker's join/bridge/leave protocol. It requires `DIMOS_TEST_BROKER_URL`, `DIMOS_TEST_ROBOT_API_KEY`, `DIMOS_TEST_OPERATOR_TOKEN`, and `DIMOS_TEST_DEDICATED_ROBOT=1`. Use a dedicated test account. Missing credentials are skipped coverage and cause the CLI qualification command to exit 2. The manually dispatched `diagnostics qualification` workflow provides separate ROS/DDS and hosted jobs; these jobs must run successfully before those transports are qualified.

Existing ROS 2/LTTng captures can be imported with `dimos diagnostics import-ros-trace SOURCE OUTPUT --run-id RUN`. This requires the Babeltrace 2 Python bindings. Callback durations retain their process/thread/callback identity; executor tracepoints stay separate observations. CTF clocks remain unmapped to the application clock until an independently established mapping exists, and callback addresses are never treated as message identities. This follows the [ROS tracepoint definitions](https://github.com/ros2/ros2_tracing/tree/rolling/tracetools) and [Babeltrace reader API](https://babeltrace.org/docs/v2.0/python/bt2/examples.html).

The dedicated channel is derived from run/link IDs, never the legacy robot channel. The envelope verifies version, run, link, payload type and bounded size. It carries CBOR payload data in a string carrier. This adds copies/base64 overhead and is for explicitly selected diagnostic links, not automatic rewriting of every camera stream. Existing QoS/provider settings must be supplied explicitly. WebRTC needs an appropriate provider config; ROS and DDS need their optional runtimes. An explicit contract exchange validates schema, run, link and payload type before application delivery. Module wiring starts negotiations; module startup waits for compatibility before binding handlers. Standalone callers start both endpoints then call `wait_ready()` during setup. Command publication never waits for negotiation. ROS/DDS/WebRTC/Cloudflare carrier factories are implemented but require live environment qualification.

The identity tuple is `(run_id, publisher_instance, stream, sequence)`. A sample ID is the SHA-256 hex digest of its compact JSON array (`ensure_ascii=True`); payload hashes and timestamps are never used as identity. Publisher instances change on process restart. `parents` carries bounded causal links; `lineage` is `complete`, `partial`, or `unknown`.

For application-specific processing, retain `current_context()` alongside cached input, then use `publication(output_stream, parents=[...])` and `span(stage)` from `dimos.diagnostics.runtime`. Arbitrary executors or third-party queues need the same explicit handoff. Do not attach robot images, secret config, or unbounded payloads to event fields.

## Frontend API contract

These routes are under the existing DimOS gateway access boundary and are listed in its OpenAPI schema and `dimos.yaml`. They do not start a separate service or expose an unrestricted filesystem path:

| Method/path | Result |
| --- | --- |
| `GET /diagnostics/capabilities` | Implemented hooks, optional dependency availability, qualification gaps |
| `GET /diagnostics/clock` | Four-timestamp probe response |
| `GET /diagnostics/{run_id}/status` | Per-process observer state, loss, counters, histograms, resource counters and freshness timestamp |
| `GET /diagnostics/{run_id}/metrics` | Per-process counts/histograms, resource metrics, observer loss and status freshness |
| `GET /diagnostics/{run_id}/topology` | Typed graph plus raw per-process binding evidence |
| `GET /diagnostics/{run_id}/incidents` | Relative incident IDs and sizes |
| `GET /diagnostics/{run_id}/incidents/{process}/{incident}` | Bounded NDJSON incident download |
| `GET /diagnostics/{run_id}/incidents/{process}/{incident}/analysis` | Offline analysis of that incident |

Artifacts live at `<root>/<run>/<publisher-process>/`. The first NDJSON row is a `dimos.incident/v1` manifest; events use `dimos.trace/v1`. Native/Python telemetry stays on the machine. Opt-in browser tracing uploads bounded gzip batches to the existing authenticated Spatial Desktop route; one unacknowledged batch is retained and retried with the same ID. Uploads have a separate 80 KiB/s payload budget and do not run in XR rendering callbacks. Combine host artifacts offline under one run directory for cross-host inspection. The frontend should label stale/missing processes and coverage gaps instead of treating an empty response as healthy.

## Browser capture and exports

Run the matching gateway/Spatial Desktop diagnostics branches together. Add `?diagnosticsRun=<same-run-id>` to the Spatial Desktop URL to opt in. The browser negotiates the trace contract when opening command publishers; an older incompatible gateway rejects setup. Camera frames use source attachments and RTP timestamps scoped to the current subscription generation. Reconnects clear the old frame correlation cache. Repeated command transmission retains the original input identity and age.

`video.presented` is a browser callback, `video.texture_upload` an observed renderer texture update, and `video.xr_submit` the render submission following that update. These are software estimates with explicit correlation provenance, not optical display measurements. Hidden panels are not reported as submitting a new frame merely because they remain subscribed.

Use `dimos diagnostics network ROOT RUN --interface INTERFACE --probe-url URL/diagnostics/clock` for periodic endpoint routes, counters and low-rate clock probes without subscribing to robot streams. An optional `--infrastructure FILE` imports `dimos.infrastructure/v1` controller evidence with source/target IDs, association, exporter clock, export/observation timestamps and evidence provenance. Stale exports remain stale; endpoint counters do not identify a radio/AP/backhaul cause.

Add repeatable `--peer ADDRESS` options for concurrent endpoint RTT observations.
The observer also probes the current default gateway and records interface/route
changes. Linux collects `iw` link, station and survey fields, `ethtool` counters and
`tc` queue counters; macOS uses a cached CoreWLAN helper without scanning for networks.
Retry, traffic and airtime deltas never cross a counter reset or association change.
Drivers that omit station/survey counters report unavailable fields, not zero loss.

With access to a Linux access point, add `--ap-host USER@HOST --ap-interface IFACE`.
The read-only SSH collector uses existing SSH configuration and reads each client's
station counters separately. For managed controllers use the normalized infrastructure
import with exporter identity, observation time and clock provenance. Endpoint RTT,
station retry counts and AP downlink counters are distinct evidence; their presence
does not justify assigning an entire RTT spike to radio airtime.

For kernel socket observations, explicitly start `dimos diagnostics socket-echo ADDRESS` on a chosen test host; it prints its ephemeral port. `dimos diagnostics socket-probe ADDRESS PORT OUTPUT` measures that dedicated echo path. On 64-bit Linux it requests receive timestamps; other platforms report the unavailable capability. These observations are not application delivery acknowledgments or over-air timestamps.

`dimos diagnostics export BUNDLE OUTPUT --format otlp-json` generates an offline OTLP/HTTP JSON document. An explicit `--otlp-endpoint URL` sends it. Only spans with a recorded local wall/monotonic anchor are exported; omissions are reported. Wall-clock anchors are for visualization, not cross-host latency calculations.

## Qualification state

The implementation is under active qualification. Local regression coverage includes separate-process Zenoh/LCM fan-out, identical payloads, ordering/duplication, publisher restart, setup mismatch, Python→Rust→Python asynchronous parent lineage, actual WebRTC command delivery and video RTP metadata, and a shaped WebRTC link with an independently injected upstream delay. Additional tests cover matched-path budgets, missing parents/cameras, clock bounds, freezes, interrupted diagnostic uploads, host retention and exports.

ROS/DDS have also been exercised using real separate processes (Humble on isolated
domain173 and CycloneDDS11.1 locally). Humble's message-only callbacks remain supported;
native writer metadata is exposed only when the installed binding supplies it.
An explicit `wait_ready(expected_peers=...)` barrier is available when all fan-out
recipients must finish negotiation before a test starts publishing.

Hosted-provider qualification needs the dedicated credentials described above.
An unavailable dependency is skipped coverage, never a pass. A synthetic desktop
browser fixture does not establish headset optics, office RF attribution or the
60-minute R1 Pro/headset qualification.

The complete synthetic application fixture drives two independent arm inputs through
the actual native gateway, shared100Hz control loop and fake adapters, alongside three
H264 cameras through actual browser video/Three texture/render callbacks. It also
pauses rendering while reception continues and checks the fault attribution:

```bash
python -m dimos.diagnostics.demo_browser_qualification --gateway GATEWAY_CHECKOUT --desktop SPATIAL_CHECKOUT --directory NEW_OUTPUT_DIRECTORY --chrome CHROME_EXECUTABLE
```

Build the gateway's `diagnostic_browser` example first. All sockets are loopback-only,
discovery is disabled and no robot commands leave the fixture. Use `--mode off` or
`--mode trace` with `--steady-seconds 20` in separate processes for an overhead comparison.
That mode continuously drives both inputs at60Hz with three30Hz cameras and measures
the Python/gateway/Deno/Chrome process tree, browser-upload bytes, and matched input
selection-to-dispatch age via a separate fixture sidecar that runs in both modes.
Browser/Python clock offsets use four-timestamp bounds, with observed drift included.
Compare the p99 *bounds*, not just their midpoints; overlapping uncertainty cannot
certify the less-than1ms additional-command-age gate. The other proposed gates are
less than 2 additional host CPU percentage points and at most 100 KiB/s telemetry.

```bash
dimos diagnostics overhead OFF_DIRECTORY/result.json TRACE_DIRECTORY/result.json overhead.json
```

The same fixture provides isolated workload regressions, using `--steady-seconds 4`:

- `--decode-delay-ms 80` stalls only head-camera decoding and asserts that the
  head decode span grows while wrist decoding stays healthy.
- `--control-delay-ms 40` stalls one control task and asserts that task timing
  grows while camera decoding stays healthy.
- `--extra-viewers 1` adds a second actual browser viewer without claiming the
  operator lease, then requires separate three-camera render evidence for both.
- `--recorder-binary PATH` runs the existing Rust recorder with three synthetic
  image streams and a bounded temporary MCAP artifact. It never builds or replaces
  an installed recorder. The recorder and its child CPU are included in the workload.

Each run writes its inputs, workload checks and measured results to `result.json`
and its trace report to `report.html`. Compare equal workloads; the overhead command
rejects comparisons with different viewers, recorder state or injected delays.
The gateway's ignored `diagnostics` integration test exercises real WebRTC with
directional delay, measured injected packet drops/bursts and packet reordering.
Its `backpressure` tests cover finite bandwidth and slow receivers. Unit regressions
cover clock expiry, missing telemetry, failed uploads and fresh-input/IK failures.

The small reproducible Python benchmark is `python -m dimos.diagnostics.demo_overhead --output result.json`; run it separately in off/metrics/trace modes. It is a microbenchmark, not representative camera/robot qualification.
