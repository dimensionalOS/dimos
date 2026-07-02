# dimoscope — DimOS topics in the browser (Dimos JS)

`@dimos/web` (Dimos JS) is a transport-agnostic browser SDK to subscribe to DimOS topics, visualize
them, and teleoperate — plus a single backend service and a transport benchmark. The browser decodes
messages itself with [@dimos/msgs](https://jsr.io/@dimos/msgs) (the 8-byte type hash is
self-describing), so the service is a thin byte-relay and the same SDK works over any transport.

```
robot / sim ─► DimOS bus (LCM | Zenoh)
   │
   └─► the gateway — ONE Python process (`deno task serve`), http://0.0.0.0:8080
         taps BOTH LCM + Zenoh → one normalized stream, fanned out over every transport:
           GET /            the built web app  (the SDK consumer itself)
           WS  /ws          data plane: topics + teleop/goal/rpc  (the trust boundary)
           GET /sse · /poll Server-Sent Events · HTTP long-poll
           WS  /rtc         WebRTC DataChannel          UDP :8443  WebTransport (QUIC)
           WS  /media       camera: webrtc / webcodecs / jpeg
           GET /cert        WebTransport self-signed cert hash
   ▼
@dimos/web  (decode via @dimos/msgs · on-demand · QoS · client.call RPC · useVideo media)
   ▼
@dimos/react ─► app (WorldView · Camera · Pose · Stats · Commands · Topics) + teleop
```

## Quickstart

From the **repo root**, with only the `web` extra — every line is copy-paste:

```bash
uv sync --extra web                    # backend deps (FastAPI + zenoh + aioquic), once
cd dimos/web/dimoscope
deno install && deno task build        # frontend deps + the app bundle the gateway serves, once

deno task serve                        # tab 1 — the gateway → http://localhost:8080/
deno task load                         # tab 2 — /load/* data source (multi-rate lanes + crankable flood)
```

Open **http://localhost:8080/** — topics appear in the sidebar; the **Topics** tab shows per-topic
stream cards (live hz / kB/s / latency) with per-topic QoS controls. `deno task app` runs a Vite dev
server on :5173 (hot reload) instead of the built bundle; the gateway also runs cwd-free as
`uv run python -m dimos.web.dimoscope.gateway`.

## The SDK in 15 lines — list topics, subscribe, control the push rate

```ts
import { createDimosClient } from "@dimos/web";

const client = createDimosClient();          // default transport: WebSocket
await client.connect("ws://localhost:8080"); // "/ws" appended automatically

console.table(client.listTopics());          // [{topic, type}, …] — live discovery, zero config
const sub = client.subscribe("/load/cloud", (m) => {
  console.log(m.data, m.meta.latencyMs);     // m.data = the decoded dimos-lcm message (@dimos/msgs)
});

// Server-side push control — per client, per topic. The gateway downsamples/sheds on ITS side
// (bytes leave the wire); other subscribers of the same topic are unaffected:
client.setQos("/load/cloud", { maxHz: 2, priority: "low", reliability: "best-effort" });

const one = await client.peek("/load/grid", { timeoutMs: 2000 }); // pull-based one-shot
sub.unsubscribe();                           // on-demand: bytes stop on the wire
```

Also on the client: `client.teleop(lin, ang)` (the service clamps velocity + runs a TTL deadman),
`client.subscribeAll(cb)` (firehose), `client.topic(name)` (rich handle — `getLatest()`,
`setQos({ maxHz: 15 })`, `stats()` → `{ hz, bytesPerSec, lastLatencyMs, count }`), and
`await client.modules.GO2Load.start_bench(hz, bytes, kind)` — RPC via a typed Proxy.

**Server-side you write nothing.** The gateway self-discovers topics from both LCM and Zenoh, assigns
QoS lanes by a type/name heuristic (override per deployment via `qos.rules.json` — see
`qos.rules.example.json`), and runs a per-client priority outbox. RPC is a server-side whitelist —
the `RPC_COMMANDS` list in `gateway/egress.py` (GO2Connection `standup`/`liedown` + GO2Load
`start_all`/`stop_all`/`start_bench`/`stop_bench`); edit it to expose your own `@rpc`.

Typed topics + commands are generated from the blueprint: `deno task gen-types` writes
`app/src/dimos.topics.gen.ts` (scenarios + `go2_load.py`), and
`createDimosClient<DimosTopics, DimosCommands>()` autocompletes topic names, message fields, and RPC
signatures — see [`packages/web/README.md`](packages/web/README.md).

## Why a gateway (and not zenoh-ts)

zenoh-ts bridges a raw bus session into the tab — every client gets every subscribed byte at bus rate,
with no per-client shaping and no trust boundary. A browser on hotel wifi needs **decimation/QoS at a
server, per client**: the gateway downsamples per subscriber+topic (`maxHz`), runs a per-client
**priority outbox** (4 bands, weighted round-robin; best-effort topics conflate to the newest frame,
reliable ones keep a bounded depth), and can pace egress (`EGRESS_KBPS`) so the backlog — and
therefore priority — lives in the outbox instead of the socket buffer. Measured under a saturated
link: pose p50 **1294 ms → 4 ms** with the heavy stream still flowing
([benchmarks §2](docs/benchmarks.md)). The gateway stays a thin byte-relay — decoding happens in the
browser — and it is also where teleop gets its velocity clamp + TTL deadman + stop-on-disconnect.

## The 1 Gb/s test — subscribe to a firehose, the browser survives

Offer ~1 Gb/s on one topic (125 MB/s of incompressible 1 MB frames):

```bash
DIMOS_TRANSPORT=zenoh uv run dimos run load -o go2load.heavy_hz=125 -o go2load.heavy_bytes=1000000
```

Subscribe `/load/img` in the app: it lanes as **bulk** (best-effort, conflated), so the per-client
outbox holds only the newest frame and the tab stays interactive; the pose lanes keep streaming at
~1 ms beside it. Loopback ceiling: one Python process relays **~255 MB/s (~2 Gb/s)** over WS; at
180 MB/s offered the browser takes 154 MB/s at p50 25 ms ([benchmarks §1](docs/benchmarks.md)). The
same knob lives in the UI — **Topics tab → Benchmark drawer → load generator** (`raw-1080p` =
180 MB/s, `firehose` = 300 MB/s). For the server-on-one-machine, browser-over-the-internet half, see
the [VPS runbook](docs/benchmarks.md) (§3).

## The demo — `go2-load` (the dog)

One command runs the **teleoperable go2 dimsim** *and* a multi-rate stream source, so you can drive the
robot and exercise on-demand + QoS at once. The dog is **heavier than `--extra web`** (dimsim +
Playwright Chromium):

```bash
uv sync --extra unitree --extra sim --extra mapping   # once — the sim stack
deno task serve   # the gateway (:8080 + WT QUIC :8443)
deno task dog     # go2 dimsim: teleop + camera + /load/* streams  (= dimos --simulation dimsim run go2-load)
```

`go2-load` (`dimos/robot/benchmark/go2_load.py`) is `autoconnect(unitree_go2, GO2Load)`: the go2 plus
`/load/{fast,mid,slow,grid,cloud}` published at **distinct rates** (100 / 20 / 2 / 5 / 10 Hz) with
`start_all`/`stop_all`/`enable`/`disable`/`set_rate` `@rpc`, **plus** a crankable `/load/img` flood
(`start_bench`/`stop_bench`) for the benchmark. In the browser:

- **Topics tab →** click the `/load/*` chips (or **all discovered**) → five cards at their distinct Hz;
  **remove/add a card = on-demand off/on** (bytes stop/start on the wire while the sim keeps running).
- **WASD** drives the robot; release → deadman stop; close the tab → stop-on-disconnect.
- **Commands** → *Start/Stop streams* + *Start/Stop bench* (`GO2Load`) + *Stand up* (`GO2Connection`).
- **Fallback:** open in Safari (no WT) or with UDP `:8443` blocked → Auto lands on WebSocket; teleop
  still works. Other data sources: `deno task sim` / `sim:dimsim` / `sim:replay` (plain go2), or
  `deno task scope:{nav,arm,cam,bench}` ([scenarios](scenarios/README.md)).

## Transports — pick a delivery mechanism from the topbar dropdown

Same app, same `@dimos/web` SDK, **five swappable delivery mechanisms — all on the one service,
same-origin** (the dropdown rebuilds the client; `?gw=host:port` overrides the origin, e.g. a remote
VPS for real-WAN testing). They carry identical self-describing frames, so the codec is unchanged;
only the wire differs:

| Mechanism        | Wire              | Path        | Notes                                                                                                        |
| ---------------- | ----------------- | ----------- | ------------------------------------------------------------------------------------------------------------ |
| **WebSocket**    | TCP               | `/ws`       | duplex, reliable; carries teleop/goal/rpc; the universal fallback for Auto                                   |
| **SSE**          | TCP/HTTP          | `/sse`      | server→client only (binary→base64)                                                                           |
| **HTTP poll**    | TCP/HTTP          | `/poll`     | universal req/resp baseline                                                                                  |
| **WebRTC data**  | SCTP/DTLS/**UDP** | `/rtc`      | configurable unordered/unreliable → no TCP head-of-line blocking                                             |
| **WebTransport** | HTTP/3 **QUIC**   | UDP `:8443` | streams + datagrams, no HoL; also carries teleop/rpc; the Auto default; cert hash from `/cert` (Chrome/Edge) |

**Default = Auto (WT→WS).** The app's default transport prefers **one WebTransport connection**
carrying *both* data (QUIC datagrams/streams, no HoL) *and* control (subscribe/QoS + teleop/goal/rpc
over its bidirectional stream), and **falls back to WebSocket** when WebTransport is unavailable
(Safari), can't connect (UDP-blocked), or **dies mid-session** — tracked subscriptions replay onto the
fallback wire. The WebSocket transport itself reconnects with exponential backoff and a ping heartbeat
(half-open links are detected; subscriptions + QoS replay on reopen). The topbar dropdown still forces
any single mechanism for benchmarking.

**Safety:** teleop/goal/rpc ride the `/ws` **and WebTransport** control paths, both funneled through
one shared `SafetyEgress` (velocity clamp + TTL deadman + stop-on-disconnect); the read-only
mechanisms (SSE/poll/WebRTC-data) can't bypass it.
The gateway taps **both** LCM and Zenoh, so whichever bus the robot uses reaches the browser with no
config.

## Media plane (camera)

The camera is the one heavy stream (~11 MB/s JPEG, ~55 MB/s raw), so it rides its **own plane** beside
the topic data — the `/media` **WebSocket** on the same service (`gateway/media.py`). `useVideo(topic)`
negotiates the best available delivery (capability + graceful fallback), picked by the topbar **cam:**
toggle:

| Mode          | How                                                          | Path                        |
| ------------- | ------------------------------------------------------------ | --------------------------- |
| **webrtc**    | aiortc encodes once; browser HW-decodes a `<video>`          | `/media`                    |
| **webcodecs** | libx264 NAL chunks over WS → `VideoDecoder` → `<canvas>`     | `/media`                    |
| **jpeg**      | the Image topic itself, decoded in-browser (universal floor) | the data plane (`/ws` etc.) |

Encode-once-per-topic → fan out to N viewers; multiple cameras = multiple topics. The jpeg floor works
over any data mechanism.

## Benchmark

The benchmark runs **in the real browser** across all 5 delivery mechanisms (WS · SSE · poll ·
WebRTC-data · WebTransport), so WebRTC/WebTransport are measured on the actual browser stacks. See
**[docs/benchmarks.md](docs/benchmarks.md)** for the methodology, QoS, real-WAN runbook, and full tables.

```bash
deno task serve   # the one service on :8080
deno task load    # the /load/* lanes + the GO2Load start_bench/stop_bench flood knob
# open http://localhost:8080/ → Topics tab → Benchmark drawer
```

Drive the load generator up the ladder (light 2 MB/s → firehose ~300 MB/s), sweep each transport, and
copy the results as Markdown. The top tiers can stress or crash the tab — that's the case being
measured, so sweep light→heavy. Route through a remote VPS with `?gw=host:port` for real-WAN numbers;
tune duration with `?dur=ms`.

Headlines: the service is a byte-relay, so **language isn't the bottleneck** (one Python process relays
~255 MB/s over WS; ~1 ms p50 on loopback); at robot-realistic bulk (≤20 MB/s) **WebTransport and
WebSocket deliver identical throughput at 0% loss**, and WT's datagram lanes stay at 1 ms even when its
bulk stream is saturated (no head-of-line blocking between lanes); **WebSocket is the robust WAN
baseline** — it carries 20 MB/s through 5% injected loss with **0% data loss**; WT's no-HoL win is
measured *under loss* and wants a domain + CA TLS (see [docs/benchmarks.md](docs/benchmarks.md));
server-JSON decode costs **2.64× the bytes** of the binary relay; **~75% on-demand** bandwidth cut.

## Layout

| Path                                                  | What                                                                                                                                                                                                                           |
| ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `packages/web/`                                       | `@dimos/web` (Dimos JS) — client + topic + `src/transports/` (`gatewayWs`=`ws()`, `webTransport`, `composite`=`webtransport()` Auto, `experimental/` bench transports) + `src/media/` (jpeg/webRtc/webCodecs) + `src/bench.ts` |
| `packages/react/`                                     | `@dimos/react` — hooks (`useTopics`, `useVideo`, `useTeleop`, `useRpc`/`useCommands`, `SubscribeBar`, …)                                                                                                                       |
| `app/`                                                | Vite example app (`panels/`: WorldView, CameraView, PoseReadout, TeleopPad, StatsBar, CommandsPanel, BenchDrawer, `streams/` the Topics tab)                                                                                   |
| `gateway/app.py`                                      | the single backend entrypoint (`python -m gateway`) — one process, all transports + the app                                                                                                                                   |
| `gateway/bus.py`                                      | the LCM+Zenoh bus tap → one normalized stream every transport reads from                                                                                                                                                       |
| `gateway/{data,media}.py`                             | `/ws` data plane (topics + teleop/goal/rpc) · `/media` camera plane                                                                                                                                                            |
| `gateway/{qos,egress}.py`                             | per-client priority outbox (the QoS enforcement point) · `SafetyEgress` (clamp + deadman + rpc whitelist)                                                                                                                      |
| `gateway/transports/`                                 | the `/sse` `/poll` `/rtc` + WebTransport planes                                                                                                                                                                                |
| `scenarios/`                                          | dimos publisher blueprints — live data sources (`nav`/`arm`/`cam`) + `bench.py` (standalone `/load/*` source)                                                                                                                  |
| `packages/web/src/bench.ts` · `app/…/BenchDrawer.tsx` | the in-browser benchmark core (`STREAM_PROFILES`, `measureScenario`) + the Topics-tab Benchmark drawer UI                                                                                                                      |

## Status / next

Done:

- SDK + React app + teleop (verified in Chrome, incl. driving the robot) + transport benchmark.
- Single-service gateway: one Python process taps LCM + Zenoh and serves the app + all five delivery
  mechanisms (WS · SSE · poll · WebRTC-data · WebTransport) + the camera.
- Media plane: camera over WebRTC / WebCodecs / JPEG, capability-negotiated (`useVideo` + cam toggle).
- RPC bridge: `client.call("GO2Connection","standup")` → whitelisted dimos `@rpc` (Commands panel).
- WebTransport teleop + Auto (WT→WS) default: one QUIC connection carries data and control through the
  shared `SafetyEgress` (clamp + deadman + stop-on-disconnect), with a WebSocket fallback at connect
  and mid-session. Plus the `go2-load` demo blueprint (teleop go2 dimsim + multi-rate `/load/*` lanes +
  crankable flood + Topics-tab chips).
- No head-of-line blocking under loss: WebRTC-data / WebTransport (UDP) stay smooth where WS (TCP)
  stalls; plus on-demand subscribe + server-enforced QoS (maxHz downsample / priority / reliability / depth).
