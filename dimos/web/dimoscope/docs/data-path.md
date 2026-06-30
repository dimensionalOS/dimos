# dimos → client data path: mapping & benchmarking every delivery mechanism

**The one question this answers:** how do you move a dimos topic from the robot to a
client, and where do you decode it? Not "which app to build" — the *protocol*. Everything
here is in service of measuring that one thing under realistic robot load and bad networks.

> Refocus note: the first pass spread across applications (media plane, Rerun-3D, CV, multi-robot,
> a React cockpit). This pass deletes all of that from the spotlight and goes deep on the data path:
> the SDK, a load generator, rigorous metrics, and a network-impairment benchmark — runnable from the
> CLI in one command, with tests in the package. The React app is parked.

---

## Two axes

**Axis 1 — transfer mechanism** (how bytes reach the client). All carry the *same*
self-describing payload (8-byte type hash + LCM fields), so the codec is identical; only
the transport differs.

| mechanism | wire | dir | reliability/order | HoL under loss | browser | status here |
|---|---|---|---|---|---|---|
| **WebSocket** (binary) | TCP | duplex | reliable, ordered | **yes (TCP HoL)** | universal | ✅ `gatewayWs` (mature) |
| **zenoh-ts** (remote-api) | TCP/WS | duplex | reliable | yes (rides WS) | needs bridge | ✅ prototype |
| **SSE** | TCP/HTTP | server→client | reliable, ordered | yes | universal (binary→base64) | ✅ **new** `adapters/sse.ts` |
| **HTTP long-poll** | TCP/HTTP | req/resp | reliable | yes | universal | ✅ **new** `adapters/httpPoll.ts` |
| **WebRTC DataChannel** | SCTP/DTLS/**UDP** | duplex | **configurable** (unordered/partial) | **no** | universal | ✅ **new** (server + adapter, e2e-verified) |
| **WebTransport** | HTTP/3 **QUIC** | streams+datagrams | **configurable** | **no** | Baseline 3/2026 | ✅ **new** (server + adapter, Chrome-verified) |

**Axis 2 — decode location** (where bytes become a typed object).

| strategy | gateway role | client needs | bandwidth | gateway CPU | industry ref |
|---|---|---|---|---|---|
| **client binary decode** (8-byte hash) | thin byte-relay | `@dimos/msgs` | smallest | ~0 | **dimos today** |
| **server decode → JSON** | full deserialize | nothing | largest | high → drops under load | **rosbridge** (~36% drop) |
| **server decode → Protobuf/CBOR** | deserialize + re-encode | schema | small | medium | **Foxglove** (C++ bridge) |

dimos's self-describing **binary byte-relay + client decode** is the differentiator: the
gateway parses nothing, so it's I/O-bound not CPU-bound. rosbridge (server→JSON) is
documented to drop **~36%** of messages on a high-rate camera stream; Foxglove rewrote its
bridge in C++ and moved to Protobuf/FlatBuffers to cope. We **measured** Axis 2 on our own bus
(`--decode server-json`): server-JSON costs **2.64× the bytes** of the binary relay (see Results).

---

## What's built (this pass)

- **Load generator** — `bench/bench_source.py`, a configurable dimos `Module` (`BenchSource`)
  emitting small (PoseStamped), medium (OccupancyGrid) and large (Image, up to ~10 MB) streams
  at chosen rates, each stamped with a publish time (one-way latency) + a per-topic sequence
  (exact drop detection). Runs standalone (what the harness drives) or as a `.blueprint()`.
- **SDK** — same `@dimos/topics` `Transport` contract, now with `ws` / `sse` / `httpPoll`
  adapters (all decode through one shared `gatewayFrame.ts`), isomorphic browser/Deno.
- **Gateway** — `servers/gateway.ts` serves WS **+ `/sse` + `/poll`** off one LCM bus tap, so
  mechanisms are compared on identical frames.
- **Metrics** — `packages/topics/src/bench.ts` now reports p50/p95/**p99**/max + **stddev** +
  **wire loss%** (from seq gaps) alongside hz/kB/s.
- **Decode-location axis** — `servers/gateway.ts` `decode=server-json` mode + `decode_bench.ts`:
  measures client-binary vs server-JSON on the same bus.
- **WebRTC DataChannel** — `servers/webrtc_data.py` (a re-transmitter in front of the gateway) +
  `adapters/webRtcData.ts`, verified e2e headlessly via a Python aiortc probe.
- **WebTransport** (HTTP/3 / QUIC) — `servers/webtransport.py` (an `aioquic` re-transmitter) +
  `adapters/webTransport.ts`, **size-routing** small frames over datagrams and large over per-frame
  unidirectional streams; verified headlessly (aioquic probe) **and in real Chrome**
  (`serverCertificateHashes`). Its loss test rides `bench/netsim-udp.ts` (real datagram drops, no sudo).
- **Bad-network** — `bench/netsim.ts`, a zero-install TCP impairment proxy (latency/jitter/
  bandwidth, cellular profiles) in front of the gateway; `bench/netsim-udp.ts` is its UDP sibling for
  the QUIC/WebTransport path (drops **real** datagrams).

> **Two load generators, one wire contract.** The `bench-load` blueprint
> (`dimos/robot/benchmark/bench_load.py`) is the coordinator-wired, RPC-controllable *app* driver
> (BenchPanel Start/Stop); `bench/bench_source.py` is its standalone *headless* twin that the matrix
> runs (no coordinator boot per run). Both stamp per-topic `frame_id=str(seq)` for loss detection.

---

## Run it (CLI)

From `dimos/web/dimoscope/` — each is a single self-contained command (starts its own
servers + load + teardown):

| command | what it does |
|---|---|
| `deno task test` | unit suite (metrics math, seq-loss, decode-axis, rate-limit) |
| `deno task bench:matrix` | WS/SSE/poll × lan/wifi/4g/3g/2g → `bench/RESULTS-mechanisms.md` |
| `deno task bench:decode` | client-binary vs server-JSON decode (the 2.64× tax) |
| `deno task bench:webrtc` | WebRTC DataChannel e2e throughput (aiortc browser stand-in) |
| `deno task bench:webtransport` | WebTransport (HTTP/3 / QUIC) e2e — datagrams + streams (aioquic probe) |
| `deno task bench:loss` | WebTransport datagrams under **real** packet loss (`netsim-udp`) |
| `deno task bench:all` | all of the above in sequence |

Tune with env, e.g. `PROFILES="lan 3g 2g" DUR=3000 deno task bench:matrix`.

---

## Methodology

- **Load**: `bench_source.py` over LCM; 4× PoseStamped @ 100 Hz (+ optional grid / image).
- **Latency = one-way** (publish→recv). Publisher and client share a clock (single host), so
  `recvTs − srcTs` is true one-way latency, not a halved round-trip.
- **Loss = seq gaps**: every message carries a per-topic counter; loss% = `1 − received/span`.
  Not nominal-rate-based, so it's exact even when the publisher can't hit nominal rate.
- **Impairment**: one TCP proxy (`netsim.ts`) shapes the byte stream, so every TCP mechanism
  degrades under the *same* conditions. Profiles: `lan` 0 ms · `wifi` 8 ms/±3 · `4g`
  50 ms/±15/12 Mbps · `3g` 150 ms/±40/2 Mbps · `2g` 400 ms/±120/280 kbps · `lossy` 80 ms/±80/5 Mbps.
- **Honest caveat**: on a reliable TCP stream, packet *loss* manifests as retransmit →
  latency/stall, not drops — so `loss%` stays 0 for WS/SSE/poll and impairment shows as
  latency. That is precisely why the **UDP/QUIC** mechanisms (WebRTC, WebTransport) are worth
  comparing: they don't head-of-line-block. Measured — `bench/netsim-udp.ts` drops **real** datagrams,
  and WebTransport's delivered-datagram p95 stays flat at 16 ms to 20% loss (see Results).

---

## Results (mechanism × network)

Generated by `deno task bench:matrix` — see `bench/RESULTS-mechanisms.md`. One-way ms.

| profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|--:|--:|--:|--:|--:|--:|
| lan | ws   | 293 | 24.4 | 1.7 | 4.8 | 6.8 | 0 |
| lan | sse  | 287 | 23.8 | 1.7 | 4.7 | 8.0 | 0 |
| lan | poll | 296 | 24.6 | 2.0 | 4.6 | 6.6 | 0 |
| 3g  | ws   | 235 | 19.5 | 282 | 392 | 414 | 0 |
| 3g  | sse  | 236 | 19.6 | 283 | 384 | 409 | 0 |
| 3g  | poll | 201 | 16.7 | 489 | 875 | 951 | 0 |
| 2g  | ws   |  82 | 6.8  | 1461 | 1876 | 1912 | 0 |
| 2g  | sse  |  56 | 4.6  | 1537 | 1789 | 1801 | 0 |
| 2g  | poll |   0 | 0    | — | — | — | — |

**Reading it:**
1. **On a clean link nothing differentiates** — WS ≈ SSE ≈ poll (~290 Hz, sub-2 ms p50). Don't
   pick a mechanism on LAN numbers; pick on the degraded ones.
2. **HTTP long-poll pays an extra RTT per cycle** — already 1.7× WS's p50 on 3G, and it
   **collapses to zero throughput on 2G** (the per-cycle round-trip is fatal on a slow link).
   It stays as the baseline/control, not a recommendation.
3. **SSE's base64 tax is real and shows up where bandwidth is scarce** — identical to WS on LAN,
   but on 2G it moves ~33% fewer bytes (4.6 vs 6.8 kB/s) and ~30% fewer messages, because every
   binary frame rides as base64 over a 280 kbps pipe. SSE is fine for light telemetry, wrong for
   the heavy streams.
4. **WebSocket binary is the right TCP default** — lowest overhead, duplex (teleop on the same
   socket), best degraded throughput of the three. Its weakness (TCP head-of-line blocking under
   loss) is invisible here because netsim shapes latency/bw, not raw loss — that's the Phase-2
   story.

### Decode location (same gateway, two decode modes) — `deno task bench:decode`

4× PoseStamped + grid, end-to-end:

| decode | kB/s | p50 | p99 | note |
|---|--:|--:|--:|---|
| **client binary** (relay) | 93 | 0.5 | 3.5 | dimos today — gateway parses nothing |
| **server-json** (rosbridge) | 247 | 1.3 | 4.6 | **2.64× the bytes**, + deserialize CPU on the gateway |

The self-describing **binary byte-relay wins**: the array-heavy grid bloats badly as JSON and the
gateway pays the deserialize cost — exactly the wall rosbridge hits (~36% drops). Client-decode keeps
the gateway I/O-bound.

### WebRTC DataChannel (e2e) — `deno task bench:webrtc`

A headless aiortc probe pulls **~350 Hz** of `/bench/*` over a real unordered/`maxRetransmits:0`
DataChannel relayed from the live gateway — binary frames (no base64), no TCP head-of-line blocking.
Its advantage over WS *under loss* is measured for the sibling QUIC datagram path (WebTransport) below.

### WebTransport (e2e + loss) — `deno task bench:webtransport` / `bench:loss`

A headless aioquic probe pulls ~**330 Hz** from `servers/webtransport.py`, which **size-routes** small
frames over **datagrams** and large frames over per-frame **unidirectional streams** (clean link:
datagram p95 ≈ 2 ms, stream p95 ≈ 9 ms). Verified in **real Chrome** via `serverCertificateHashes`
(917 datagrams + 81 streams in 3 s). The loss payoff (`bench:loss` through the `netsim-udp` real-drop
relay, latency held at 10 ms ± 5 to isolate loss):

| drop | datagrams/s | dgP50 | dgP95 |
|---|--:|--:|--:|
| 0% (direct) | 299 | 0 | 0 |
| 10% | 271 | 11 | 16 |
| 20% | 237 | 11 | **16** |

Delivered-datagram **p95 stays flat (16 ms) to 20% loss while throughput degrades linearly** — no
head-of-line blocking. The same loss on a reliable WS/TCP stream becomes retransmit→stall (the matrix's
3g/2g p95 is 1400–1900 ms). This reproduces the independent `realtime-web` result on our own stack.

---

## Bad-network tooling — options (researched)

DevTools throttling *can* shape WebSockets (Chrome 99+) and WebRTC (124+), but **manual-only** — the
CDP automation API (`Network.emulateNetworkConditions`, used by Playwright/Puppeteer) still skips
WS/WebRTC, so it's no good for reproducible/CI runs. `tc/netem` is Linux-only (host is macOS).

| tool | layer | platforms | UDP/QUIC | scriptable | role here |
|---|---|---|---|---|---|
| **`bench/netsim.ts`** (this repo) | TCP proxy | all (Deno) | ❌ | ✅ env/profiles | **chosen** (TCP) — zero-install, controllable, one binary fewer |
| **`bench/netsim-udp.ts`** (this repo) | UDP relay | all (Deno) | ✅ | ✅ env/profiles | **chosen** (QUIC/WebTransport) — drops **real** datagrams, no sudo |
| **toxiproxy** (Shopify) | TCP proxy | all | ❌ | ✅ HTTP API | industry standard for TCP fault injection; drop-in if preferred |
| **dummynet** (`dnctl`+`pfctl`) | link | macOS/BSD | ✅ | ✅ per-port | system-wide UDP shaping; the fallback for a symmetric raw-drop test on TCP too |
| **tc/netem** | link (kernel) | Linux | ✅ | ✅ | most reproducible; CI in a Linux container |
| comcast / Network Link Conditioner | link | x-platform / macOS | ✅ | CLI / GUI | quick device-level profiles |

We built `netsim.ts` because it's zero-install, fully scriptable from the matrix, and impairs every TCP
mechanism uniformly. A TCP proxy can't carry QUIC, so `netsim-udp.ts` is its UDP sibling — it relays
QUIC datagrams and drops a configurable fraction (real loss), giving the WebTransport loss bench without
sudo. **dummynet** (macOS-native) / **netem** stay the fallback for *system-wide* shaping (and a
symmetric raw-drop test on the TCP mechanisms too).

### Simulating bad networks in the browser (not just the headless bench)

`netsim.ts` shapes at the TCP layer, so the **same proxy impairs a real browser** — point the app
through it with the `?gw=host:port` override (added to `main.tsx` + `bench.tsx`):

```bash
deno task gateway                                                    # or `deno task servers`
NETSIM_PROFILE=3g NETSIM_LISTEN=8099 NETSIM_TARGET=localhost:8089 \
  deno run -A bench/netsim.ts                                        # impair the LCM gateway
deno task app                                                        # Vite → :5173
# open http://localhost:5173/?gw=localhost:8099  (or /bench.html?gw=localhost:8099) and pick the
# matching transport — the whole WS/SSE/poll data path now degrades like the bench:matrix 3g column.
```

| want | use |
|---|---|
| reproducible WS/SSE/poll impairment (app or `bench.html`) | **`?gw=` → netsim** (above) — scriptable, CI-able |
| quick manual "feel on 3G" | **Chrome DevTools → Slow 3G** (throttles WS since Chrome 99) |
| impair **WebRTC** camera/datachannel (UDP) | **Network Link Conditioner / dummynet** — system-wide; `?gw=`/DevTools can't shape UDP |

The media plane (`:8092`, WebRTC) and `?gw=` are independent: the override only reroutes the data
gateway, so the camera stays direct (use NLC for it).

---

## Recommendation & next steps

- **Default the SDK to WebSocket binary + client decode** (what `gatewayWs` already does):
  lowest overhead, duplex, best degraded behavior, and the self-describing relay keeps the
  gateway CPU-free — the property rosbridge/Foxglove had to fight for.
- **Keep SSE/poll as deliberate fallbacks** — SSE for one-way light telemetry on networks that
  block WS; poll as the universal baseline. Don't put the heavy streams on either.
- **Decode location: keep client-binary** (the relay). Measured **2.64× bandwidth** + gateway CPU
  for server-JSON — the rosbridge wall, avoided.
- **WebRTC DataChannel + WebTransport are built + e2e-verified.** WebTransport (HTTP/3 via `aioquic`,
  datagrams + streams, browser handshake over `serverCertificateHashes`) is measured under **real**
  packet loss via `bench/netsim-udp.ts`: delivered-datagram p95 stays flat at 16 ms to 20% loss while
  WS/TCP p95 hits seconds — matching the independent `realtime-web` result.
- **Phase 2 — deferred, documented not built:** **zenoh-ts hardening** (reconnect + live discovery), and
  a **symmetric raw-drop comparison** putting all six mechanisms on one chart (kernel shaping via
  dummynet/netem so the TCP mechanisms face real drops too).

---

## Sources
- rosbridge drops ~36% on high-rate streams; Foxglove WS protocol (Protobuf/FlatBuffers, C++): <https://foxglove.dev/robotics/rosbridge>, <https://github.com/foxglove/ros-foxglove-bridge>
- WS/SSE/poll/WebRTC/WebTransport comparison & TCP head-of-line: <https://rxdb.info/articles/websockets-sse-polling-webrtc-webtransport.html>, <https://websocket.org/comparisons/>
- WebSocket vs WebRTC vs WebTransport under packet loss (tc/netem): <https://github.com/Sh3b0/realtime-web>
- WebTransport Baseline (all browsers, 3/2026) + `aioquic` server: <https://caniuse.com/webtransport>, <https://developer.mozilla.org/en-US/docs/Web/API/WebTransport_API>
- Network impairment tools (toxiproxy / netem / comcast / Network Link Conditioner): <https://github.com/Shopify/toxiproxy>, <https://devops-daily.com/posts/network-tools-that-simulate-slow-network-connection>
