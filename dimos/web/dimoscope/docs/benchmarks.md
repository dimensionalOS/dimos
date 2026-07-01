# dimoscope benchmarks — the detailed numbers

The numbers behind [benchmark-report.md](./benchmark-report.md): **delivery mechanisms × stream ·
decode location · load & realistic robot streams · QoS · WebRTC · WebTransport under loss**. Everything
is measured in the real browser via `/bench.html` + the in-app Bench tab (both reuse the SDK's
`@dimos/topics/bench` core).

> **What decides a mechanism is payload.** 4× PoseStamped @ 100 Hz ≈ **28 kB/s** — where *every*
> mechanism looks identical and nothing is decided. **The interesting physics only appears at MB/s**
> (lidar, depth, camera). That's where the real engineering trade-offs live (§3).

## Reproduce

```bash
deno task serve         # the one service on :8080
deno task scope:bench   # data source → /bench/* (scenarios/bench.py)
deno task app
#   http://localhost:5173/bench.html   → full 5-mechanism sweep (all scenarios)
#   the in-app Bench tab                → live transport + QoS knobs (STREAM profiles, decode A/B)
#   ?gw=<vps>:8080                      → a real remote/lossy path (see real-wan.md)
deno task test          # the SDK unit suite (metrics math, seq-loss, decode-axis, rate-limit)
```

**Metrics.** Latency = one-way (`recvTs − srcTs`); `loss%` = seq gaps (received vs the seq span).
`loss%` is reliable for steady streams; under link **saturation** it inflates (buffered/late frames
widen the span), so at saturation read **hz + latency**, not loss%. For network impairment, point the
bench at a remote gateway with `?gw` and use the real path — real RTT, real reorder/drop.

---

## 1. Language/transport is *not* the bottleneck

The gateway is a byte-relay (the browser decodes via the self-describing 8-byte hash), so it's I/O-bound,
not CPU-bound — Python keeps pace with Deno, and Zenoh keeps pace with LCM. In-browser, 4× PoseStamped
@ 100 Hz runs **~417 Hz / p50 ~1–2 ms** across the bus transports — throughput parity, sub-ms p50. The
choice is not about language or bus; it's about the **delivery mechanism × payload × network** (below).

## 2. The *light* stream (pose) — nothing differentiates

4× PoseStamped @ 100 Hz (~28 kB/s): on a clean link WS ≈ SSE ≈ poll, all sub-ms–few-ms p50. The
mechanism only starts to matter as the network degrades — and even then, at 28 kB/s, all of them are
"fine enough." **Don't choose a mechanism on light-stream numbers.** The decisive differences need real
payload (§3), and a real degraded path (`?gw` to a distant/lossy gateway).

## 3. Realistic robot streams — where it actually bites ⭐

Real streams aren't 28 kB/s. `scenarios/bench.py` emits a configurable large Image (byte-accurate proxy
for a lidar/depth/camera frame):

| stream | bytes/frame | rate | throughput |
|---|--:|--:|--:|
| **lidar** | ~200 KB | 10 Hz | **~1.85 MB/s** |
| **dense lidar / depth** | ~1 MB | 20 Hz | **~18 MB/s** |
| **camera (JPEG)** | ~550 KB | 20 Hz | **~9.8 MB/s** |
| **mixed** | pose 100 Hz + grid + lidar | — | concurrent fleet |

(These are the `STREAM_PROFILES` the Bench tab exposes — `packages/topics/src/bench.ts`.)

**What the heavy streams reveal (that pose hid):**
1. **The mechanism finally matters** — at MB/s, SSE's base64 inflation makes it **1.5–2× the latency**
   of WS; on pose this was invisible.
2. **MB/s streams saturate cellular.** ~2 MB/s lidar collapses toward ~1 Hz on a 4G-class link; 10–18
   MB/s (camera/dense) is **impossible** on cellular — SSE and poll die entirely, WS merely limps.
3. **Mixed load → light streams suffer** when a heavy one saturates the pipe (→ QoS, §6).
4. **Conclusion:** heavy streams need a fat pipe (LAN/5G), **compression/decimation**, or the **media
   plane** (encode-at-source) — not a raw topic relay over cellular.

## 4. Throughput ceiling

The stack **sustains ~18 MB/s on localhost** (dense, all three TCP mechanisms keep full rate). Beyond
that the gateway/WS becomes the limit — for higher (raw camera ≈ 55 MB/s) you encode at the source and
carry compressed video on the bus (the media plane), not raw frames.

## 5. Decode location — client-binary vs server-JSON

The Bench tab's decode A/B (4× PoseStamped + grid): server-side decode→JSON costs **~2.64× the bytes**
of the binary self-describing relay, with worse p99 and gateway CPU — the rosbridge tax. **At MB-scale
it's catastrophic**: a lidar/point-cloud frame is a packed binary array; as JSON it balloons several-fold
*and* the gateway pays the deserialize. Keep decode on the client (the gateway stays a byte-relay).

## 6. QoS — the client knobs

| knob | result |
|---|---|
| **rate-limit** `setRateLimit(2)` on the ~2 MB/s lidar | ~4.8× less bandwidth, client-driven; the gateway downsamples that topic |
| **on-demand** (subscribe 1 of 4 topics) | **~75% reduction** (a topic is on the wire only while it has a live subscriber) |
| **prioritization** (mixed, saturated): cap the lidar so pose survives | pose throughput recovers ~4× when the lidar is rate-limited |

The headline: **rate-limit the heavy stream and the light ones recover** — "pick a budget, keep
pose/teleop responsive." Full design + A/B in [qos-demo.md](./qos-demo.md).

## 7. WebRTC DataChannel (e2e)

~**350 Hz** of `/bench/*` over a real unordered/`maxRetransmits:0` DataChannel — binary frames (no
base64), no TCP head-of-line blocking. Its win over WS shows up *under loss* (§8).

## 8. WebTransport (HTTP/3 / QUIC) — datagrams + streams, and the loss payoff ⭐

The service's WebTransport server (`gateway/transports/webtransport.py`, QUIC on `:8443`) **size-routes**
each frame — small (pose/imu, ≤1100 B) over **datagrams** (unreliable/unordered, never retransmitted),
large (lidar) over a per-frame **unidirectional stream** (reliable). Verified in real Chrome
(`serverCertificateHashes` over the short-lived self-signed cert): ~917 datagrams + 81 streams in 3 s,
datagram p95 ≈ 2 ms / stream p95 ≈ 9 ms on a clean link.

**The payoff — under loss.** A lost datagram simply never arrives — no retransmit, no head-of-line
blocking — so delivered-datagram latency stays flat while throughput falls ~linearly with the drop rate.
A reliable, ordered TCP stream (WebSocket) instead turns the same loss into retransmit → stall (p95 climbs
into seconds on an impaired link). Flat-few-ms vs seconds — that gap is the whole reason WebTransport (and
WebRTC) exist for lossy/remote links. Reproduce it on a real degraded path with `?gw` (see
[real-wan.md](./real-wan.md)).

---

## Takeaways — what to use when

| situation | use |
|---|---|
| small/medium topics (pose, odom, costmap), any network | **WebSocket binary** — simplest, duplex, fine everywhere |
| heavy streams (lidar/camera) on **LAN/5G** | **WebSocket binary** (SSE/poll add 1.5–2× latency at MB/s) |
| heavy streams on **cellular** | **compress/decimate** or the **media plane**; raw MB/s won't fit |
| protect light streams under load | **rate-limit the heavy ones** (QoS prioritization) |
| where to decode | **client-binary** — server-JSON is ~2.64× at 28 kB/s, catastrophic at MB/s |
| lossy/remote links | **WebTransport** datagrams (small/fast) + streams (big) — no HoL |
