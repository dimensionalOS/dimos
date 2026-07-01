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
@dimos/react ─► app (WorldView · Camera · Rerun 3D · Pose · Stats · Commands · Streams) + teleop
```



## Quickstart

The frontend is **one Deno workspace** (`deno install` covers the app + the `@dimos/{web,react}`
packages); the backend is **one Python service** (`uv sync --extra web`, from the repo root). Install
both once, then run each process in its own tab:

```bash
# one-time install
uv sync --extra web                            # backend service (FastAPI + aioquic/QUIC) — from repo root
cd dimos/web/dimoscope && deno install         # frontend workspace deps

# run (each in its own tab)
deno task serve     # the whole backend in one process → http://localhost:8080  (= uv run python -m gateway)
deno task sim       # a data source: mujoco go2;  or  sim:dimsim / sim:replay  (or a real robot)
deno task app       # the Vite app → http://localhost:5173  (or just open http://localhost:8080/)
```

The gateway serves the app at `/` too, so a built deploy needs only the one process; in dev, `deno task app` gives hot reload and connects to `:8080`.

The app auto-discovers topics, draws the map + robot + trail in **WorldView**, shows a live camera +
pose readout + a **Stats** panel (hz / kB/s / latency), and drives the robot with **WASD / arrows**
(the service clamps velocity + deadman-stops). Headless checks: `deno task smoke`,
`deno task teleop:test`.



## Teleop demo — `go2-scope`

One command runs the **teleoperable go2 dimsim** *and* a multi-rate stream source, so you can drive the
robot and exercise on-demand + QoS at once (use it in place of `deno task sim`):

```bash
deno task serve                                                       # the gateway (:8080 + WT QUIC :8443)
DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run go2-scope   # go2 dimsim + /scope/* streams
deno task app                                                         # http://localhost:5173 (or open :8080)
```

`go2-scope` (`dimos/robot/benchmark/scope_bench.py`) is `autoconnect(unitree_go2, ScopeBench)`: the go2 plus
`/scope/{fast,mid,slow,grid,cloud}` published at **distinct rates** (100 / 20 / 2 / 5 / 10 Hz) with
`start_all`/`stop_all`/`enable`/`disable`/`set_rate` `@rpc`. In the browser (connected on **Auto → WebTransport**):

- **Streams tab →** `scope` **preset** → five cards at their distinct Hz, each with its QoS lane + live metrics;
**remove/add a card = on-demand off/on** (bytes stop/start on the wire while the sim keeps running).
- **WASD** drives the robot over WebTransport; release → deadman stop; close the tab → stop-on-disconnect.
- **Commands** → *Start/Stop streams* (`ScopeBench`) + *Stand up* (`GO2Connection`), RPC over WT.
- **Fallback:** open in Safari (no WT) or with UDP `:8443` blocked → Auto lands on WebSocket; teleop still works.



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


**Default = Auto (WT→WS).** The app's first/default transport prefers **one WebTransport connection**
carrying *both* data (QUIC datagrams/streams, no HoL) *and* control (subscribe/QoS + teleop/goal/rpc over
its bidirectional stream), and **transparently falls back to WebSocket** when WebTransport is unavailable
(Safari) or can't connect (UDP-blocked). The topbar dropdown still lets you force any single mechanism for
benchmarking.

**Safety:** teleop/goal/rpc ride the `/ws` **and WebTransport** control paths, both funneled through
one shared `SafetyEgress` (velocity clamp + TTL deadman + stop-on-disconnect); the read-only
mechanisms (SSE/poll/WebRTC-data) can't bypass it.
The gateway taps **both** LCM and Zenoh, so whichever bus the robot uses reaches the browser with no
config. For data over both at once, run DimSim: `DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run unitree-go2`.

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
over any data mechanism; webrtc/webcodecs need a camera source on the bus (the gateway taps both LCM and
Zenoh).

## Commands (RPC)

The browser invokes **whitelisted dimos** `@rpc` **commands** through the service (`/ws`) —
`client.call("GO2Connection", "standup")` → `{op:"rpc"}` → `rpc_backend().call_sync(...)` → result. The
service holds a **server-side whitelist** (default `standup`/`liedown`; override
`DIMOS_GATEWAY_RPC="Module/method,…"`) and advertises it in `hello`; the **Commands** panel renders a
button per advertised command. Velocity stays on the clamped/deadman teleop path — RPC is for discrete
commands only.

## The SDK (`@dimos/web`)

```ts
import { createDimosClient, ws } from "@dimos/web";
const client = createDimosClient();                   // transport defaults to ws(); connect is a method
await client.connect("ws://localhost:8080/ws");
// the app default is Auto (WT→WS): createDimosClient({ transport: webtransport() }) — ONE WebTransport
// connection (data + teleop/rpc, no HoL) with a transparent WebSocket fallback (Safari / UDP-blocked)

client.subscribe("/odom", (m) => { … m.data … m.ts … m.meta.latencyMs … }); // one { data, ts, meta } envelope
client.listTopics();                                  // [{topic, type}, …] (live discovery)
const odom = client.topic("/odom");                   // rich per-topic handle
odom.setRateLimit(15);                                // backpressure (also asks the service to downsample)
odom.stats();                                         // { hz, bytesPerSec, dropped, lastLatencyMs }
client.teleop(0.5, 0.0);                              // safe: the service clamps + TTL/deadman watchdog
await client.modules.GO2Connection.standup();         // RPC via a typed Proxy over @rpc (from registered modules)
```

Typed topics + modules: `createDimosClient<DimosTopics, DimosCommands>()` (maps from the codegen). The
research/benchmark transports (`sse`/`poll`/`webrtc`/raw-WT) live in `@dimos/web/experimental`.
React: `DimosProvider`, `useTopics`, `useTopicLatest`, `useTopicRef` (no re-render → rAF canvas),
`useTopicStats`, `useTeleop`, `useVideo` (camera → `<video>`/`<canvas>`), `useRpc`/`useCommands`.
**On-demand:** a topic is subscribed on the wire only while it has a live subscriber.

## Benchmark

The benchmark runs **in the real browser** across all 5 delivery mechanisms (WS · SSE · poll ·
WebRTC-data · WebTransport), so WebRTC/WebTransport are measured on the actual browser stacks. See
**[docs/benchmark-report.md](docs/benchmark-report.md)** for the methodology and full tables.

```bash
deno task serve         # the one service on :8080
deno task scope:bench   # data source → publishes /bench/* (scenarios/bench.py)
deno task app           # then open http://localhost:5173/bench.html
```

Then open `/bench.html` for the full 5-mechanism sweep, or the in-app **Bench** tab to vary QoS
knobs against the live transport (live sparklines + copy-as-Markdown). Route through a remote VPS with
`?gw=host:port` for real-WAN numbers; tune duration with `?dur=ms`.

Headlines: the service is a byte-relay, so **language/transport isn't the bottleneck** (throughput
parity, sub-ms p50 on LAN); the differences appear at **MB/s under loss**, where **WS (TCP HoL) stalls**
while **WebRTC/WebTransport (UDP, no HoL) stay smooth**; server-JSON decode costs **2.64× the bytes** of
the binary relay; **~75% on-demand** bandwidth cut.

## Layout


| Path                                        | What                                                                                                                                                                                                                                                                                           |
| ------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `packages/web/`                             | `@dimos/web` (Dimos JS) — transport iface + `transports/` (`gatewayWs`=`ws()`, `webTransport`, `composite`=`webtransport()`, `experimental/` bench transports) + **media plane** (`media/`: `jpegTopicMedia`/`webRtcMedia`/`webCodecsMedia`) + client (incl. `call` RPC), topic, decode, stats |
| `packages/react/`                           | `@dimos/react` — hooks (`useTopics`, `useVideo`, `useTeleop`, `useRpc`/`useCommands`, …)                                                                                                                                                                                                       |
| `app/`                                      | Vite example app (`panels/`: WorldView, Camera, PoseReadout, TeleopPad, StatsBar, CommandsPanel, SubscribeBar, `streams/` Streams tab)                                                                                                                                                         |
| `gateway/app.py`                            | the single backend entrypoint (`python -m gateway`) — one process, all transports + the app                                                                                                                                                                                                    |
| `gateway/bus.py`                            | the LCM+Zenoh bus tap → one normalized stream every transport reads from                                                                                                                                                                                                                       |
| `gateway/{data,media}.py`                   | `/ws` data plane (topics + teleop/goal/rpc) · `/media` camera plane                                                                                                                                                                                                                            |
| `gateway/transports/`                       | the `/sse` `/poll` `/rtc` + WebTransport bench transports                                                                                                                                                                                                                                      |
| `scenarios/`                                | dimos publisher blueprints — live data sources (`nav`/`arm`/`cam`) + `bench.py` (the browser-bench `/bench/*` source)                                                                                                                                                                          |
| `app/src/bench.tsx` · `panels/BenchTab.tsx` | the in-browser benchmark (`/bench.html`) + in-app Bench tab                                                                                                                                                                                                                                    |


The original teaching prototype (`app/src/bus.ts`, `app/src/widgets/`, `WALKTHROUGH.md`) is the
parts-bin this was extracted from.

## Status / next

Done:
- SDK + React app + teleop (verified in Chrome, incl. driving the robot) + transport benchmark.
- Single-service gateway: one Python process taps LCM + Zenoh and serves the app + all five delivery
  mechanisms (WS · SSE · poll · WebRTC-data · WebTransport) + the camera. The old per-transport
  gateways are retired.
- Media plane: camera over WebRTC / WebCodecs / JPEG, capability-negotiated (`useVideo` + cam toggle).
- RPC bridge: `client.call("GO2Connection","standup")` → whitelisted dimos `@rpc` (Commands panel).
- WebTransport teleop + Auto (WT→WS) default: one QUIC connection carries data and control through the
  shared `SafetyEgress` (clamp + deadman + stop-on-disconnect), with a WebSocket fallback. Plus the
  `go2-scope` demo blueprint (teleop go2 dimsim + multi-rate `/scope/*` + Streams-tab preset).
- No head-of-line blocking under loss: WebRTC-data / WebTransport (UDP) stay smooth where WS (TCP)
  stalls; plus on-demand subscribe + a QoS layer (rate-limit / conflation / priority).

Known limitation:
- Rerun 3D panel (`panels/RerunPanel.tsx`): the stock web viewer embeds and streams but paints a black
  canvas — the undecimated `RerunBridge` firehose blows the browser WASM viewer's heap ceiling. The
  native `dimos-viewer` (:9876) handles the same feed; a browser-viable 3D needs server-side decimation.

