# dimoscope benchmarks

The master benchmark reference for getting DimOS topics to a browser: **bus transports ·
delivery mechanisms × network · decode location · load & realistic robot streams · QoS · WebRTC ·
WebTransport under real packet loss**.

> **What changed, and why this doc grew.** The first benchmarks tested 4× PoseStamped @ 100 Hz ≈
> **28 kB/s** — where *every* mechanism looks identical and nothing is decided. **The interesting
> physics only appear at MB/s** (lidar, depth, camera). This doc benchmarks those, and that's where
> the real engineering trade-offs live.

See also: [data-path.md](./data-path.md) (the methodology narrative + the two-axes framing) ·
[README](./README.md) · [findings](./findings.md). Raw data: [`../bench/RESULTS.md`](../bench/RESULTS.md)
(transports), [`../bench/RESULTS-mechanisms.md`](../bench/RESULTS-mechanisms.md) and
`../bench/RESULTS-mechanisms-{lidar,dense,camera,mixed}.md` (mechanisms × stream × network).

## Reproduce (one self-contained command each, from `dimos/web/dimoscope/`)

| command | what it measures |
|---|---|
| `deno task test` | the SDK unit suite (metrics math, seq-loss, decode-axis, rate-limit) |
| `deno task bench` / `bench/run.sh all` | **bus transports** — Deno↔LCM · Python↔Zenoh · zenoh-ts |
| `deno task bench:matrix` | **mechanisms × network** (WS/SSE/poll × lan/wifi/4g/3g/2g) |
| `STREAM=lidar deno task bench:matrix` | a **heavy stream** (2 MB/s) × network — also `dense`/`camera`/`mixed` |
| `deno task bench:decode` | **decode location** (client-binary vs server-JSON) |
| `deno task bench:qos` | **QoS** — rate-limit · on-demand · prioritization |
| `deno task bench:webrtc` | **WebRTC DataChannel** e2e |
| `deno task bench:webtransport` | **WebTransport** (HTTP/3 / QUIC) e2e — datagrams (small) + streams (big) |
| `deno task bench:loss` | **WebTransport under real packet loss** (`netsim-udp`) — graceful datagram degradation |

## Methodology

- **Load**: `bench_source.py` (the headless twin of the `bench-load` blueprint) over LCM. Small
  (PoseStamped), medium (OccupancyGrid), large (Image of N bytes — a byte-accurate proxy for a
  lidar/depth/camera frame). Each carries `ts` (publish time) + a per-topic `seq`.
- **Latency = one-way** (`recvTs − srcTs`); publisher and client share a clock (single host).
- **`loss%` = seq gaps** (received vs the seq span). *Reliable for steady streams; under link
  **saturation** it inflates* (buffered/late frames widen the span), so at saturation read **hz +
  latency**, not loss%.
- **Bad network**: a zero-install TCP impairment proxy (`bench/netsim.ts`) shapes latency/jitter/
  bandwidth. Profiles: `lan` 0 ms · `wifi` 8 ms/±3 · `4g` 50 ms/±15/**12 Mbps** · `3g` 150 ms/±40/**2
  Mbps** · `2g` 400 ms/±120/**280 kbps** · `lossy` 80 ms/±80/5 Mbps. (TCP turns packet *loss* into
  retransmit→latency, so it's shaped as latency/bandwidth; for **real** UDP packet drops — what the QUIC
  datagram path actually faces — `bench/netsim-udp.ts` drops real datagrams in userspace, see §8.)

---

## 1. Bus transports — language/transport is *not* the bottleneck

4× PoseStamped, headless, end-to-end (publish→client). From `bench/RESULTS.md`:

| transport | hz | kB/s | p50 ms | p95 ms | max ms |
|---|--:|--:|--:|--:|--:|
| Deno↔LCM | 333 | 27.97 | 0.44 | 2.04 | 5.31 |
| Python↔Zenoh | 329 | 27.63 | 0.84 | 3.41 | 5.46 |
| zenoh-ts (direct) | 333 | 27.97 | 0.53 | 3.01 | 5.58 |

**Throughput parity, sub-ms p50 on all three.** The gateway is a byte-relay (the browser decodes via
the self-describing 8-byte hash), so it's I/O-bound, not CPU-bound — Python keeps pace with Deno, and
direct zenoh-ts keeps pace with both. In-browser the same scenario runs ~417 Hz / p50 ~1–2 ms.

## 2. Delivery mechanisms × network — the *light* stream (pose)

4× PoseStamped @ 100 Hz (~28 kB/s) over the LCM gateway, WS vs SSE vs HTTP-poll:

| profile | ws p50 | sse p50 | poll p50 | note |
|---|--:|--:|--:|---|
| lan | 1.7 | 1.7 | 2.0 | all equal — **don't choose a mechanism on LAN numbers** |
| 3g | 282 | 283 | 489 | poll's per-cycle RTT starts to hurt |
| 2g | 1461 | 1537 | **dead** | poll's request/response can't keep up; SSE's base64 costs ~33% more bytes |

On a clean link nothing differentiates; the mechanism only matters as the network degrades — and even
then, at 28 kB/s, all of them are "fine enough." **The decisive differences need real payload (§3).**

## 3. Realistic robot streams — where it actually bites ⭐

Real streams aren't 28 kB/s. Simulated via `bench_source`'s configurable Image (byte-accurate):

| stream | bytes/frame | rate | throughput |
|---|--:|--:|--:|
| **lidar** | ~200 KB | 10 Hz | **~1.85 MB/s** |
| **dense lidar / depth** | ~1 MB | 20 Hz | **~18 MB/s** |
| **camera (JPEG)** | ~550 KB | 20 Hz | **~9.8 MB/s** |
| **mixed** | pose 100 Hz + grid + lidar | — | concurrent fleet |

**lidar (~2 MB/s)** — `STREAM=lidar`:

| profile | mech | hz | kB/s | p50 | p99 |
|---|---|--:|--:|--:|--:|
| lan | ws | 9.5 | 1853 | 10.0 | 13.7 |
| lan | sse | 9.5 | 1853 | **16.6** | 18.8 |
| lan | poll | 9.5 | 1853 | 9.9 | 14.3 |
| 4g | ws | **1.25** | 244 | **2107** | 3390 |
| 4g | sse | 0.75 | 146 | 2025 | 2889 |
| 4g | poll | 0.25 | 49 | 877 | 877 |

**dense (~18 MB/s)** — `STREAM=dense`:

| profile | mech | hz | MB/s | p50 |
|---|---|--:|--:|--:|
| lan | ws | 18.3 | 17.8 | 19.8 |
| lan | sse | 18.5 | 18.0 | **41.6** |
| lan | poll | 18.5 | 18.0 | 18.6 |
| 4g | ws | 0.25 | 0.24 | 3803 |
| 4g | sse | **0** | **0** | — (dead) |
| 4g | poll | **0** | **0** | — (dead) |

**camera (~9.8 MB/s)**: LAN all three ~18 Hz/9.8 MB/s (SSE p50 29.5 vs WS 20.9); 4G → WS/SSE limp at
0.25 Hz/~2 s, poll dead. **mixed**: on 4G the 2 MB/s lidar saturates the link and the pose/telemetry
topics show **~99% loss** — the heavy stream starves the light ones.

**What the heavy streams reveal (that pose hid):**
1. **On LAN the mechanism finally matters** — SSE's base64 makes it **1.5–2× the latency** of WS at
   MB/s (29–42 ms vs 20 ms). On pose this was invisible.
2. **MB/s streams saturate cellular.** 2 MB/s lidar collapses to ~1 Hz / 2 s on 4G; 10–18 MB/s
   (camera/dense) is **impossible** on 4G — **SSE and poll die entirely**, WS merely limps.
3. **Mixed load → light streams suffer** when a heavy one saturates the pipe (→ QoS, §6).
4. **Conclusion:** heavy streams need a fat pipe (LAN/5G), **compression/decimation**, or the
   **media plane** (encode-at-source) — not a raw topic relay over cellular.

## 4. Throughput ceiling

The stack **sustains ~18 MB/s on localhost** (dense, all three mechanisms keep full rate). Beyond
that the gateway/WS becomes the limit — for higher (raw camera ≈ 55 MB/s) you encode at the source and
carry compressed video on the bus (the media plane), not raw frames.

## 5. Decode location — client-binary vs server-JSON

`deno task bench:decode` (4× PoseStamped + grid): server-side decode→JSON costs **2.64× the bytes** of
the binary self-describing relay, with worse p99 and gateway CPU — the rosbridge tax. **At MB-scale it's
catastrophic**: a lidar/point-cloud frame is a packed binary array; as JSON it balloons several-fold
*and* the gateway pays the deserialize. Keep decode on the client (the gateway stays a byte-relay).

## 6. QoS — the client knobs (`deno task bench:qos`)

| knob | result |
|---|---|
| **rate-limit** `setRateLimit(2)` on the 2 MB/s lidar | 1886 → **390 kB/s** (**4.83× less**), client-driven; the gateway downsamples that topic |
| **on-demand** (subscribe 1 of 4 topics) | 56 → 14 kB/s = **74.9% reduction** (zenoh-ts makes it true end-to-end) |
| **prioritization** (4G, mixed): cap the lidar so pose survives | pose throughput **79 → 298 Hz (~4×)** when the lidar is rate-limited |

The headline: **rate-limit the heavy stream and the light ones recover** — "pick a budget, keep
pose/teleop responsive." This is the only client QoS today (rate); Zenoh's reliability/durability QoS
is publisher-side, so a browser subscriber can't set it (it maps to gateway emulation — future work).

## 7. WebRTC DataChannel (e2e)

`deno task bench:webrtc`: ~**350 Hz** of `/bench/*` over a real unordered/`maxRetransmits:0`
DataChannel — binary frames (no base64), no TCP head-of-line blocking. Its win over WS shows up *under
loss* — measured for the sibling QUIC datagram path in §8.

## 8. WebTransport (HTTP/3 / QUIC) — datagrams + streams, and the loss payoff ⭐

`deno task bench:webtransport`: a headless aioquic client pulls ~**330 Hz** of `/bench/*` from
`servers/webtransport.py` (a re-transmitter in front of the gateway), **size-routing** each frame —
small (pose/imu, ≤1100 B) over **datagrams** (unreliable/unordered, never retransmitted), large (lidar)
over a per-frame **unidirectional stream** (reliable). Same `[f64 send-ms][LC02]` frames, same client
`frameToSample` decode. Clean link: datagram p95 ≈ 2 ms, stream p95 ≈ 9 ms. **Verified in real Chrome**
(`serverCertificateHashes` over the short-lived self-signed cert): 917 datagrams + 81 streams in 3 s.

**The payoff — `deno task bench:loss`** runs the probe through `bench/netsim-udp.ts`, a userspace UDP
relay that drops **real** datagrams (no sudo / no dummynet), at 10 ms ± 5 latency so the effect of *loss*
is isolated from latency. Pose-only (pure datagrams), per drop rate:

| drop | datagrams/s | dgP50 ms | dgP95 ms |
|---|--:|--:|--:|
| 0% (direct) | 299 | 0 | 0 |
| 5% | 279 | 11 | 16 |
| 10% | 271 | 11 | 16 |
| 20% | 237 | 11 | **16** |

**Delivered-datagram p95 stays flat (16 ms) as loss climbs to 20 %; throughput falls ~linearly with the
drop rate.** A lost datagram simply never arrives — no retransmit, no head-of-line blocking. A reliable,
ordered TCP stream (WebSocket) instead turns the same loss into retransmit→stall: §2's matrix shows WS
p95 at **1400–1900 ms** on impaired 3g/2g links. Flat 16 ms vs seconds — that gap is the whole reason
WebTransport (and WebRTC) exist for lossy/remote links.

> Honest caveat: `netsim-udp` drops real UDP datagrams; the TCP `netsim` can only *model* loss as jitter
> (a reliable stream never exposes raw drops). The asymmetry is the point — TCP converts loss to latency;
> QUIC datagrams don't. A fully symmetric raw-drop test on TCP too needs kernel shaping (dummynet/netem),
> the documented fallback.

---

## Takeaways — what to use when

| situation | use |
|---|---|
| small/medium topics (pose, odom, costmap), any network | **WebSocket binary** — simplest, duplex, fine everywhere |
| heavy streams (lidar/camera) on **LAN/5G** | **WebSocket binary** (SSE/poll add 1.5–2× latency at MB/s) |
| heavy streams on **cellular** | **compress/decimate** or the **media plane**; raw MB/s won't fit |
| protect light streams under load | **rate-limit the heavy ones** (QoS prioritization) |
| where to decode | **client-binary** — server-JSON is 2.64× at 28 kB/s, catastrophic at MB/s |
| lossy/remote links | **WebTransport** datagrams (small/fast) + streams (big) — no HoL; flat 16 ms p95 to 20% loss (§8) |

## Phase 2 (documented, not built)
**zenoh-ts reconnect + live discovery**, and a **symmetric raw-drop test on the TCP mechanisms** (kernel
shaping via dummynet/netem) so WS and WebTransport sit on one chart. WebTransport itself is now built and
measured (§8): aioquic server, headless probe, browser adapter (Chrome-verified), and the userspace UDP
loss bench.
