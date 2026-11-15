# Desktop integration measurement history

> Historical results from 2026-10-31. The review found biased latency, CPU and buffer measurements. These reports do not qualify the corrected build. Artifact hashes below identify the old binaries only.

> The `demo_*.py` harnesses named below were deleted on 2026-11-03. The module lifecycle check lives on as `dimos/web/relay_bridge/test_topic_module_lifecycle_e2e.py`.

Implementation and release evidence for `PLAN_1.md`. The supported local
workloads pass their functional and performance gates. Packaged Linux x86-64
Desktop enables the SDK runtime by default on glibc 2.28 or newer. Explicit
rollout exceptions and the legacy rollback remain documented below. No
performance or platform parity is implied by a unit test.

## Baseline inputs

- dimOS: `d2fb73cb5b6943bb2ecfe41e817fe790901ee762`.
- Desktop: `5622e97053121aa741fd5daab4048d3c7ad5d870`; Rust dependencies
  `zenoh-gateway = 0.5.32`, `zenoh-dimos-codecs = 0.3.9`.
- Gateway checkout: `e770ae62201ec4ac6229b2a2473437acfaa009f2`.
- Desktop UI pins dimos-react `0.3.8`, which vendors dim-app `0.25.1`.
  Source changes must be made upstream and pinned, never in `node_modules`.
- Linux measurement host: AMD Ryzen 9 9950X, 32 logical CPUs, 123 GiB RAM,
  Ubuntu kernel `7.0.0-38-generic`. Each result records browser, workload,
  network, executable checksum, timings, and measurement limitations.

The event baseline harness is
`python -m dimos.web.relay_bridge.demo_desktop_baseline --desktop-binary
<built-desktop> --output <result.json>`. It starts a scratch Desktop home without
a dimOS installation, and uses synthetic JSON events on a dedicated Zenoh
endpoint. It does not connect to a robot. Baseline output belongs in a state or
artifact directory, outside the source tree.

### Event measurements (2026-10-31)

The source publishes five JSON events per 100 Hz tick (500 events/s), with
128 padding bytes, for ten seconds on unshaped loopback. Both browser runs
use actual HTTP documents; Playwright document interception changes Chromium's
local-network classification and is not a valid WebTransport platform test.

| Path | Browser | Delivered / sent | Reordered | p95 age | Service CPU, % of one core |
|---|---|---|---|---|---|
| Existing Desktop gateway | Chromium 149.0.7827.55 | 5000 / 5000 | 0 | 2.23 ms | 1.70 |
| Existing Desktop gateway | Firefox 151 | 5000 / 5000 | 0 | 2.81 ms | 1.50 |
| Raw SDK + isolated topic provider + compiled relay | Chromium 149.0.7827.55 | 5000 / 5000 | 0 | 1.49 ms | 9.80 |
| Raw SDK + isolated topic provider + compiled relay | Firefox 151 | 5000 / 5000 | 0 | 1.72 ms | 10.00 |

The declared SDK baseline exposed 21 distinct events through its 500 ms UI
snapshots. That is expected snapshot behavior, not evidence of transport loss.
The raw topic callback is separate and sees every delivered event.

The initial raw path passes this short event latency/completeness check, but
**fails the proposed CPU comparison**. CPU excludes browser/source processes:
gateway measurements cover Desktop, while raw-path measurements cover the
Python provider plus relay. A full Desktop comparison and profiling are still
required. Raw-path RSS rose from about 160 MB to 175 MB during these short runs;
no memory plateau has been established. These results do not authorize cutover.

Artifacts are in `~/.local/state/dimos/benchmarks/plan1/`: `events-*.json` and
`topics-*.json`. `demo_topic_baseline.py` runs the compiled relay and the pinned
runtime's Python from a temporary directory with no selected checkout and no
Deno/development dependency resolution. Both raw browser runs reported no
startup downloads. The compiled relay SHA-256 for this measurement is
`ae24ded36637e912b0a1745edbc44f9100fcb2bbe043320202952f1127c31bc5`.

The later 60-second comparison includes every browser descendant process,
with a two-second warm-up and the same 500 events/s source:

| Path | Events | Reordered | p95 age | Pipeline CPU, % of one core |
|---|---|---|---|---|
| Desktop gateway | 30,000 / 30,000 | 18 | 1004.49 ms | 3.90 |
| SDK, runtime manifest `9e93f984…` | 30,000 / 30,000 | 0 | 1.41 ms | 11.28 |

Artifacts: `events-pipeline-chromium-60s.json` and
`topics-pipeline-chromium-60s.json`. The SDK still fails the CPU gate; the
comparison means 10% relative to baseline, not ten percentage points. The
gateway's ordering/latency failures are also retained in the report.

Three quiet, matched 60-second pairs subsequently measured the 90 ms event
coalescing candidate, with 128-message drains and loopback-only 1500-byte QUIC
packets/32-packet bursts. Every published event still has its own frame and
callback; this does not sample the reliable stream. The order was gateway,
SDK, SDK, gateway, gateway, SDK, and all six results are retained in
`events-paired-local90-summary.json`. Arithmetic mean pipeline CPU was 4.0722%
of one core for the gateway and 4.5166% for the SDK: **10.91% higher, failing
the 10% relative gate**. SDK p95 ages were 92.09–92.14 ms, with all 30,000 events
per run delivered in order. The gateway delivered all events after draining,
but had 32–206 reorderings and p95 ages of 875–1244 ms. A bounded five-second
callback drain replaces the earlier fixed one-second wait; CPU includes this
drain and remains normalized to the 60-second source duration. Candidate
runtime manifest SHA-256:
`ce008cfc46e7b6e23f70547d6b4eabbfe67fe4c8b035e969c3d5bd977d736967`.
This narrow miss is not rounded into a pass.

The ACK-coalescing packaged runtime was then measured with three new matched pairs in
the same order and with the same arithmetic-mean criterion. It preserves
unused pacer credit across same-tick ACKs, coalesces outgoing packet builds
within an event-loop turn, and reads reliable QUIC credit at a queue limit or
delivery notification instead of for every offered event. Source export
`8d622c4224d5a065e4c69e97c4ee624ab792d074e56badd31471b0e461713f54`
was reproduced byte-for-byte; Nix runtime manifest SHA-256 is
`1a60b8ee45fe02daa54c1dd7ddb1a6f69e48c2d51f8d3d8ac20a4fdbbe79c2db`.

| Path | Mean pipeline CPU, % of one core | p95 event age across runs | Completeness/order |
|---|---:|---:|---|
| Desktop gateway | 4.1889 | 730–1165 ms | All 90,000 events; 95 reorderings |
| Packaged SDK runtime | 4.5722 | 92.04–92.13 ms | All 90,000 events; no reorderings |

The SDK's relative CPU increase is **9.15%**, passing the stated 10% mean gate
with limited margin. All six reports and their aggregate are retained as
`events-paired-ack90-*.json`. The earlier failed candidate remains failed;
this result applies to the identified packaged runtime and measured host.
Additional experiments with a one-millisecond relay flush and larger
congestion-window pacing bursts did not improve CPU and are not included.

The subsequent burst-sensitive policy waits up to 90 ms only after three
successive arrivals with gaps below 5 ms, retaining the burst classification
for 100 ms of local ingress time. Sparse events go through immediately. This
preserves the high-rate job-log CPU improvement without delaying ordinary
100 Hz telemetry. Its three quiet 60-second event repeats used the same
workload and the retained fresh gateway mean above: SDK CPU was 4.5333%,
4.6333%, and 4.4666%, averaging **4.5444%, or 8.49% above the gateway**. All
90,000 events arrived in order; p95 ages were 92.09–92.21 ms. The predeclared
arithmetic-mean criterion passes, with limited margin. Individual results
and the mean are retained in `events-burst3-*.json`.

This candidate's source export is
`ab50f471fad6d356f3134d0470a4d6f8b1a174116bcee5a3ce6d59610396d880`,
reproduced byte-for-byte. Its Nix runtime manifest SHA-256 is
`d406c01f970a3e5cc95beb4dd33f1bac5f8defa5b2f2cd9bca55b2624169cfc5`.
The intermediate two-arrival policy also passed mean event CPU (+9.81%) but
failed sparse-event latency in the mixed screen; those reports remain in
`events-burst-summary.json` and `media-sdk-burst-60s-odom-1`.

### Implemented foundations

- Protocol v8 has source registration, authorized dynamic routes, provider
  readiness/activation, cumulative reliable receipts, listing, and binary
  publication routing and receipts. TS/Python golden vectors include malformed
  topic inputs and capability announcements.
- The relay binds grants once per connection, authenticates source providers,
  shares matching subscriptions, assigns fresh routes/generations on source
  restart, filters dynamic discovery/stats, and denies empty-manifest access.
- The minimal provider opens `zenoh.Session` directly using the shared wire
  settings. It supports exact/wildcard raw reads, observed-topic pagination,
  bounded callbacks, and reliable ordered streams with real QUIC ACK accounting.
- The SDK exposes per-message `topics.subscribe`, readiness/status, independent
  updates, reconnect notifications, binary writes, and listing. Identical subscriptions share
  one route; declared stores retain their existing UI tick. ESM/types and the
  served single-file bundle both build.
- The provider receives bounded descriptor snapshots from the optional metadata
  module and liveliness tokens. Exact remapped transport keys survive listing;
  load/unload/reload refreshes metadata without adding payload subscriptions.
- Fixed JPEG, lossless depth, and quantized cloud profiles select fresh samples
  before using one of two shared transform slots. Browser decoding has a shared
  work/byte budget. Color views draw JPEG bitmaps or video tracks, close replaced
  bitmaps, and keep one pending frame. Clouds retain typed arrays and headers.
- Desktop supervises a relocatable Python/compiled-relay bundle independent of
  the selected robot checkout, extracts an embedded inventory-verified archive,
  and reaps both children on shutdown. Shell and explicitly opted-in installed
  apps receive separate ephemeral credentials through checked local bootstraps.
- dim-app chooses one connection per app and maps the baseline media presets to
  fixed SDK profiles. dimos-react exposes `sdkSource`; replay clocks and memory
  sources retain their existing contracts. Remote pages and unsupported browser
  engines keep the gateway, with `transport: "legacy"` as the rollback selection.
- Desktop's explicit discovery probe accepts Zenoh expressions, uses Zenoh's
  matcher, and caps duration at one second, simultaneous probes at four, and
  retained keys at 4096 per probe. The adapter uses it only when requested.

The standalone service is explicitly enabled through a private JSON config;
its `enabled` default is false. Desktop enables its managed service on the
supported host platform. Set `topics.enabled: false` in Desktop's configuration
to roll back globally; this selection survives save/reload and unrelated
settings changes. Development binaries without an embedded runtime, other
platforms and older glibc hosts retain the gateway default. App-level
`transport: "legacy"` remains available; apps must declare the SDK capability
to receive their privileged bootstrap.

### Reproducible runtime and custom-module verification

Desktop's Nix build consumes a checksummed source archive, a locked vendored JS
graph, Deno/denort 2.9.8, Python 3.12.13, and platform-specific wheel hashes for
Linux x86-64/AArch64 and macOS x86-64/ARM64. Its release workflow runs
`scripts/check_topic_runtime.ts` on each native target before uploading. That
probe installs from the executable into a clean home, reads a real wildcard
event using the embedded Python, verifies bootstrap caller checks, and imports
the native media extensions without a dimOS checkout. Only Linux has been run
locally; the other native CI checks are not claimed as passes.

`module-lifecycle-8/report.json` records a real forkserver coordinator and
Chromium subscribed before loading a custom module. Explicitly remapped topics
appear as known, become observed after publication, disappear on unload, and
return on reload without restarting the provider. Custom dataclass and mapping
values decode through safe CBOR with values 42.5 and 43.5, without a declared
Cockpit channel. The harness uses an isolated Zenoh router and client sessions.

### Representative load and outstanding gates

`demo_topic_media_load.py` renders actual 1280×720 RGB camera frames and
100,000-point clouds, together with reliable events and guarded commands. It
records source-to-draw ages, codec/queue timings, all service/browser CPU, RSS,
and source/wire payload bytes. Headless Chromium uses software WebGL; GPU scanout
is not measured. The initial ten-second SDK run delivered 29.5 camera FPS and
9.9 cloud Hz with p95 ages 50/133 ms; the equivalent gateway delivered 19.1 FPS
and about 9.5 Hz with p95 55/153 ms. These short runs do not establish release
throughput or memory gates.

The first completed 30-minute overload run, `media-overload-30m-2/report.json`,
**failed**. Two views, 60 FPS camera input, 40 Hz cloud input, subscription churn,
and 90 requested browser suspensions produced 28.1 FPS and 18.2 cloud Hz per view. There
were two event sequence gaps and a guarded publisher expired after about
21 minutes. Service RSS continued rising, from 501 to 698 MiB in successive
five-minute averages; browser RSS rose from 840 to 958 MiB. Application queues
stayed bounded and encoding stopped completely after all viewers closed.
Concurrent development work makes this run unsuitable as a CPU comparison.

That evidence prompted the Deno 2.9.8 upgrade, explicit aioquic HTTP/3 parser
cleanup, and compact closed-stream ID ranges that still reject late packets.
The source harness now uses Zenoh blocking congestion control for reliable
events, and records service RSS per process. These changes require a new
30-minute pass; the old failure is not reclassified.

`media-memory-5m-2/report.json` subsequently delivered 29.56 camera FPS and
19.77 cloud Hz per view, with no event gaps/reordering, skipped commands, or
runtime errors. Service RSS averaged 508.3, 508.4, and 508.2 MiB in its final
three minutes; encoding again stopped after disposal. This is only a five-minute
memory result. Inspection also showed that the requested lifecycle freezes had
not stopped JavaScript on the visible headless page. The harness now pauses the
renderer through CDP and verifies unchanged animation counters. The corrected
30-second case (`media-real-pause-30s-1`) kept healthy views at 29.6 FPS/19.8 Hz
with ordered events, while the paused viewer's bounded reliable route interrupted
and recovered. The full soak must use that verified suspension mechanism.

The verified suspension run `media-overload-30m-3/report.json` also **failed**:
healthy views averaged 10.4 camera FPS and 0.66 cloud Hz, and the guarded
publisher expired after about 667 seconds. All 179,626 healthy-view events
arrived in order. Service RSS did plateau (five-minute means 584.8, 635.5,
642.0, 645.6, 647.5, 648.0 MiB); browser RSS declined to about 740 MiB.
All 90 pauses were verified through unchanged renderer animation counters.
After disposal there were no routes, buffered bytes, or new encodes. These
memory and cleanup results do not excuse the delivery/control failure.

A quiet two-minute repeat (`media-final-pause-120s-1`) reproduced the cloud
failure without competing builds or tests. Per-second transport diagnostics
in `media-diagnostic-pause-150s-1` then showed cloud delivery stopping after
about 70 seconds despite ample QUIC congestion and flow-control credit.
Inspection identified aioquic 1.3's inconsistent packet-time floor and pacing
bucket calculation: a growing congestion window progressively reduced the
per-wake packet allowance. The local correction keeps its existing bounded
burst size consistent with the actual packet interval.

The quiet correction run `media-pacer-pause-180s-2/report.json` delivered 29.84
camera FPS and 19.65 cloud Hz per healthy view for three minutes, with p95 ages
51/128 ms. All 18,003 events were ordered and complete; 3,601 command receipts
completed with p95 9 ms and no runtime errors. Nine 15-second renderer pauses
were verified. End-of-run provider buffering was about 0.59 MiB, not the
growing cloud backlog seen before the correction. All routes/buffers and new
encoding work went to zero after disposal. Runtime manifest SHA-256:
`a75c6e852d2843d8fc8d20e4ec4f66bad12a5ed79f7a67f3a5eec87ab4a14f7d`.
This was a short sustained-delivery result; the completed 30-minute soak is
recorded below. The preceding attempt's debugger-pause timeout is a harness
failure, not a pass; the harness now executes an explicit breakpoint and saves
partial diagnostics on failure.

The 200 ms RTT / 1% loss / 100 Mbps trial
(`media-rtt200-loss1-1/report.json`) also failed media delivery and interrupted
the bounded reliable route. The simulated command output stopped and did not
resume automatically. Remote Desktop retains its legacy path; constrained-link
cutover requires the PLAN_2 adaptive video/congestion work. No configured
Cloudflare deployment was available for an RTC comparison.

The quiet 60-second mixed-screen comparison adds a real DOM odometry display
at 30 Hz to the 720p/30 camera, 100k-point/10 Hz cloud, 100 Hz reliable events,
and 20 Hz commands. With the packaged `1a60b8ee…` runtime, the SDK delivered
29.8 camera FPS, 10.0 cloud Hz, and 29.8 odometry updates/s, versus 11.2, 8.0,
and 17.5 for the gateway. Camera/cloud p95 display ages were 52/136 ms versus
68/201 ms; odometry was 35 ms versus 36 ms. Pipeline CPU was 87.65% of one core
versus 200.50%. The screen delivered about 74.2 Mbps of application payload,
within the declared 100 Mbps per-screen payload budget. This is unshaped
loopback evidence, not a measured remote-link result. All 6,001 events were
ordered, all 1,200 command receipts completed, and command p95 was 5 ms.

That run also exposed the unconditional event window's latency cost: sparse
100 Hz events had p95 86 ms, versus 12 ms for the gateway. The subsequent
burst-sensitive policy batches only dense arrivals and requires its own repeat.
Reports: `media-gateway-60s-odom-1` and `media-sdk-ack90-60s-odom-1`.

The actual job-log comparison uses a real temporary Nix build through Desktop's
backend, bus, `watchJob`, and App Store DOM. Both paths rendered all 30,000
lines in order. SDK p95 display age was 97 ms versus 1,951 ms; pipeline CPU was
60.24% of one core versus 93.53%. There were no browser errors. Reports are
`job-gateway-quiet-60s-1` and `job-sdk-ack90-quiet-60s-1`; the latter uses the
packaged runtime override with the real release executable. These results
also need confirmation after the burst-sensitive change.

The three-arrival candidate's quiet comparison uses the adapters' normal
1000 Hz raw-topic cap on **both** paths for the 30 Hz odometry source. The
earlier 30 Hz transport cap introduced unnecessary phase-dependent sampling.
The source sizes/rates, browsers and rendering are otherwise unchanged:

| Metric | Gateway | Packaged SDK candidate |
|---|---:|---:|
| Pipeline CPU, % of one core | 194.75 | 92.58 |
| Camera displayed FPS | 11.23 | 29.75 |
| Cloud displayed Hz | 8.07 | 9.97 |
| Odometry displayed Hz | 17.93 | 29.98 |
| Camera p95 age | 68 ms | 54 ms |
| Cloud p95 age | 200 ms | 136 ms |
| Odometry p95 age | 35 ms | 19 ms |
| Reliable event p95 age | 11 ms | 3 ms |

Both paths delivered all 6,001 events in order. All 1,200 SDK provider
acknowledgements completed, p95 4 ms, with no skipped commands or browser
errors. The gateway reports client put acceptance rather than provider
acknowledgement, so those timings are not interchangeable. SDK application
payload was about 74.2 Mbps, within the declared 100 Mbps screen budget.
Reports: `media-gateway-raw-odom-60s-1` and
`media-sdk-burst3-raw-odom-60s-1`.

The candidate also rendered all 30,000 actual job-log lines in order, p95
96.84 ms, at 63.75% pipeline CPU versus the retained 93.53% gateway run.
There were no browser errors. `job-sdk-burst3-quiet-60s-1` identifies the
release executable and explicit current-runtime override. These quiet local
event, screen and job-log comparisons pass their stated gates. The overload
and final embedded-release checks are recorded below.

Before the final 30-minute run, the overload memory budgets are fixed at
1 GiB aggregate service RSS and 1.5 GiB aggregate browser RSS. The last three
five-minute means must stay within a 5% band, while provider ingress and
outbound bytes remain within their separate 64 MiB limits. Healthy views must
keep at least 95% of the configured 30 FPS/20 Hz targets, with zero event gaps,
reordering or guard interruptions. All routes, queued bytes and new encoding
work must disappear after disposal. The workload includes two healthy views,
one repeatedly verified paused viewer, and subscription churn; a failure in
the paused viewer may interrupt that viewer's bounded reliable route.

`media-overload-30m-4` was deliberately stopped at 444 seconds to correct the
harness: it combined both healthy views' display counts, which cannot prove
the individual-view gate. At interruption it had no healthy event gaps,
reorderings or command errors, with 22 verified pauses; it is **not** a
completed soak. The replacement records per-view display counts/ages and a
small live progress snapshot. The packaged runtime and declared gates are
unchanged. The overload workload intentionally exceeds the representative
screen's 100 Mbps payload budget; its purpose is to verify bounded operation
and disposal on unshaped loopback.

The replacement `media-overload-30m-5/report.json` completed 1,800 seconds with
the same packaged runtime (`d406c01f…`). Every declared soak gate passes:

| Measurement | Result | Gate |
|---|---:|---:|
| Camera display, each healthy view | 29.79 FPS | At least 28.5 FPS |
| Cloud display, each healthy view | 19.89 Hz | At least 19 Hz |
| Camera / cloud p95 display age | 43 / 77 ms or better | Below 200 ms |
| Odometry / event / command acknowledgement p95 | 18 / 6 / 7 ms | Below 100 ms |
| Ordered healthy events / command acknowledgements | 180,001 / 36,001 | Zero gaps, reordering, errors or skipped commands |
| Verified 15-second viewer suspensions | 90 | Repeated suspension throughout the run |
| Maximum aggregate service / browser RSS | 534.1 / 1,001.1 MiB | 1,024 / 1,536 MiB |
| Last three five-minute RSS means, service / browser band | 0.084% / 2.259% | At most 5% |
| Maximum provider ingress / outbound buffering | 8.33 / 0.69 MiB | 64 MiB each |
| Routes, queued bytes and new encodes after disposal | All zero | All zero |

Service RSS means were 514.1, 514.5 and 514.2 MiB; browser means were 911.0,
914.0 and 931.6 MiB. Pipeline CPU was 351.5% of one core for this deliberate
overload; there is no equivalent gateway-soak CPU comparison. Each view received
about 123 Mbps of media payload at the doubled cloud target. The representative
10 Hz screen, rather than this overload, is subject to the 100 Mbps budget.
The paused viewer's bounded routes reported interruptions and recovered;
healthy views and commands continued. `gates.json` contains the gate evaluation,
reproduced with `python -m dimos.web.relay_bridge.demo_topic_media_gates
<report.json>`.

### Installed application verification (2026-10-31)

`demo_desktop_apps.py` builds the reviewed dimos-react source with its vendored
dim-app adapter, installs it as a static app into a fresh Desktop home, and uses
the real app HTTP response and its CSP. No request interception is used. The
app declares `uses: { "@dimos/sdk": ">=0.1 <0.2" }`; loading the SDK asset alone
does not authorize its bootstrap. The app CSP permits the managed relay's HTTPS
loopback address with an ephemeral port, so child restarts remain reachable.

The bundled Chromium 154 and external Firefox 151 smokes passed JPEG canvas drawing, a real Three.js
point-cloud draw, bit-exact 16-bit depth, preserved intensity/color, original
acquisition timestamps, readable custom-dataclass CBOR, 100 ordered frontend
events, binary publication receipts, listing, provider/relay kill recovery, and
child reaping. Temporary HTTP 503s during injected child failures are recorded
separately from unexpected errors. This tiny functional fixture is not a camera
or lidar throughput measurement. Chromium used software WebGL in headless mode.

Evidence: `~/.local/state/dimos/benchmarks/plan1/desktop-media-app-5/` contains
`report.json` and the rendered `app.png`; `desktop-media-app-firefox-1/` holds the
equivalent Firefox evidence. Inputs:

- Desktop executable SHA-256:
  `573b3eb9d6d7ba68aa504103b9eead967ac619cb5560a8f84f9f748973410157`.
- Runtime archive `media-runtime-0.1.0-linux-x86_64.tar.zst` SHA-256:
  `911c9cff92085b62fa5f16980191ac4bd66516f153e1233c6b9a6138ae9c2851`.
- Runtime manifest SHA-256:
  `aad8b50a10dfa4925da2339920618b5dae2a636829925af01ec15cf4129194bc`.
- Installed app bundle SHA-256:
  `0eea642807e861bbc914a49c3d65fb957ee5a9345416554bbad9a5f1705fb588`.

These are local review artifacts, not published release pins. The bundle pins
NumPy 2.3.5 and Pillow 12.2.0; neither is imported for raw startup. The standalone
media readers were implemented from dimOS's LCM schemas: the installed gateway
codec package's license does not grant implementation-copying rights.

### Final packaged release and rollback

The final Nix Linux release is
`~/.local/state/dimos/benchmarks/plan1/desktop-final-release/bin/dimos-desktop`.
It embeds the exact runtime used by the passing event, screen, job and soak
measurements. The source export was reproduced byte for byte after the final
repository checks; the reviewed dimos-react archive also reproduces its pin.

| Artifact | SHA-256 |
|---|---|
| Desktop executable | `76be4bfc406d700d29d53719a714eeebae144920707cbf6199f67ec65b575b5e` |
| Embedded runtime manifest | `d406c01f970a3e5cc95beb4dd33f1bac5f8defa5b2f2cd9bca55b2624169cfc5` |
| Runtime source archive | `ab50f471fad6d356f3134d0470a4d6f8b1a174116bcee5a3ce6d59610396d880` |
| Shared dimos-react source archive | `e27c1d23836b7ae1c8e6536e74f6667739aa0a04389154b3d68c6bfb4d82b1be` |

The native release probe omits the topic configuration on Linux x86-64 and
therefore tests the shipping default. With a clean home, no selected checkout
and restricted PATH, it read an authorized wildcard event and loaded native
media extensions in 1.6 seconds. Missing caller headers were rejected. All four
runtime bundles assembled; native execution here covers Linux x86-64 only.

`desktop-final-app-chromium` and `desktop-final-app-firefox` passed the complete
installed-app fixture on bundled Chromium 154 and external Firefox 151 using
this executable's embedded bundle. Both verified rendering, lossless depth,
custom data, 100 ordered events, binary receipts, guard expiry, focus loss,
stale-handle rejection, child restarts and reaping, with no unexpected errors.

`controller-final-embedded` ran the actual Controller and R1 watchdog with
simulated actuator output under 720p/30 camera and 100k-point/10 Hz cloud load.
Provider loss stopped output in 180 ms, and held input was not replayed. It
loaded no legacy client asset. This confirms the final artifact against the
earlier complete injected-failure matrix; it is not physical-hardware evidence.

`final-legacy-rollback-2.json` explicitly sets `topics.enabled: false`, asserts
that the managed runtime is disabled and has neither child PID, and receives
all 1,000 gateway events without gaps, reordering or browser errors. The earlier
short rollback run only established gateway connectivity because its harness
omitted the new setting; it does not establish runtime disablement.

The final default/configuration checks passed 29 config and five runtime Rust
tests, including explicit enable/disable persistence and platform selection.
dimOS's repository-wide pre-commit checks and its changed/new-file checks pass.
Desktop's edited TypeScript formatting passes. Rustfmt reports one existing
line-wrap difference in `src/server/mod.rs`, reproduced from the unmodified
baseline; no changed Rust code is implicated. These binaries and benchmark
reports are local review artifacts, not published release pins.
