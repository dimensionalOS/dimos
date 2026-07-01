# dimoscope — DimOS topics in the browser (Dimos JS)

The **`@dimos/web`** package (product name **Dimos JS**) is a **transport-agnostic browser SDK** to
subscribe to DimOS topics, visualize them, and
teleoperate — plus a single backend service and a transport benchmark. The browser is a first-class
bus client: it decodes messages itself with [`@dimos/msgs`](https://jsr.io/@dimos/msgs) (the 8-byte
type hash is self-describing), so the service is a thin byte-relay and the **same SDK works over
any transport**.

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
@dimos/react ─► app (WorldView · Camera · Rerun 3D · Pose · Stats · Commands) + teleop
```

## Quickstart (real DimOS, no hardware)

The frontend is **one Deno workspace** (`deno install` covers the app + the `@dimos/{topics,react}`
packages); the backend is **one Python service** (`uv sync --extra web`). Two tabs:

```bash
cd dimos/web/dimoscope && deno install        # frontend deps (one-time)

deno task serve     # the whole backend in one process → http://localhost:8080  (= uv run python -m gateway)
deno task sim       # a data source: mujoco go2;  or  sim:dimsim / sim:replay  (or a real robot)
deno task app       # the Vite app → http://localhost:5173  (or just open http://localhost:8080/)
```
The gateway serves the app at `/` too, so a built deploy needs only the one process; in dev, `deno task
app` gives hot reload and connects to `:8080`.

The app auto-discovers topics, draws the map + robot + trail in **WorldView**, shows a live camera
+ pose readout + a **Stats** panel (hz / kB/s / latency), and drives the robot with **WASD / arrows**
(the service clamps velocity + deadman-stops). Headless checks: `deno task smoke`,
`deno task teleop:test`.

## Transports — pick a delivery mechanism from the topbar dropdown

Same app, same `@dimos/web` SDK, **five swappable delivery mechanisms — all on the one service,
same-origin** (the dropdown rebuilds the client; `?gw=host:port` overrides the origin, e.g. a remote
VPS for real-WAN testing). They carry identical self-describing frames, so the codec is unchanged;
only the wire differs:

| Mechanism | Wire | Path | Notes |
|---|---|---|---|
| **WebSocket** | TCP | `/ws` | duplex, reliable — the default; carries teleop/goal/rpc too |
| **SSE** | TCP/HTTP | `/sse` | server→client only (binary→base64) |
| **HTTP poll** | TCP/HTTP | `/poll` | universal req/resp baseline |
| **WebRTC data** | SCTP/DTLS/**UDP** | `/rtc` | configurable unordered/unreliable → no TCP head-of-line blocking |
| **WebTransport** | HTTP/3 **QUIC** | UDP `:8443` | streams + datagrams, no HoL; cert hash from `/cert` (Chrome/Edge) |

**Safety:** teleop/goal/rpc ride the `/ws` **and WebTransport** control paths, both funneled through
one shared `SafetyEgress` (velocity clamp + TTL deadman + stop-on-disconnect); the read-only
mechanisms (SSE/poll/WebRTC-data) can't bypass it.
The gateway taps **both** LCM and Zenoh, so whichever bus the robot uses reaches the browser with no
config. For data over both at once, run DimSim: `DIMOS_TRANSPORT=zenoh uv run dimos --simulation
dimsim run unitree-go2`.

## Media plane (camera)

The camera is the one heavy stream (~11 MB/s JPEG, ~55 MB/s raw), so it rides its **own plane** beside
the topic data — the **`/media` WebSocket** on the same service (`gateway/media.py`). `useVideo(topic)`
negotiates the best available delivery (capability + graceful fallback), picked by the topbar **cam:**
toggle:

| Mode | How | Path |
|---|---|---|
| **webrtc** | aiortc encodes once; browser HW-decodes a `<video>` | `/media` |
| **webcodecs** | libx264 NAL chunks over WS → `VideoDecoder` → `<canvas>` | `/media` |
| **jpeg** | the Image topic itself, decoded in-browser (universal floor) | the data plane (`/ws` etc.) |

Encode-once-per-topic → fan out to N viewers; multiple cameras = multiple topics. The jpeg floor works
over any data mechanism; webrtc/webcodecs need a camera source on the bus (the gateway taps both LCM and
Zenoh).

## Commands (RPC)

The browser invokes **whitelisted dimos `@rpc` commands** through the service (`/ws`) —
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
// bad internet? one line: createDimosClient({ transport: webtransport() }) — WS control + WT data, no HoL

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

Then open **`/bench.html`** for the full 5-mechanism sweep, or the in-app **Bench** tab to vary QoS
knobs against the live transport (live sparklines + copy-as-Markdown). Route through a remote VPS with
`?gw=host:port` for real-WAN numbers; tune duration with `?dur=ms`.

Headlines: the service is a byte-relay, so **language/transport isn't the bottleneck** (throughput
parity, sub-ms p50 on LAN); the differences appear at **MB/s under loss**, where **WS (TCP HoL) stalls**
while **WebRTC/WebTransport (UDP, no HoL) stay smooth**; server-JSON decode costs **2.64× the bytes** of
the binary relay; **~75% on-demand** bandwidth cut.

## Layout
| Path | What |
|---|---|
| `packages/web/` | `@dimos/web` (Dimos JS) — transport iface + `transports/` (`gatewayWs`=`ws()`, `webTransport`, `composite`=`webtransport()`, `experimental/` bench transports) + **media plane** (`media/`: `jpegTopicMedia`/`webRtcMedia`/`webCodecsMedia`) + client (incl. `call` RPC), topic, decode, stats |
| `packages/react/` | `@dimos/react` — hooks (`useTopics`, `useVideo`, `useTeleop`, `useRpc`/`useCommands`, …) |
| `app/` | Vite example app (`panels/`: WorldView, Camera, PoseReadout, TeleopPad, StatsBar, CommandsPanel, SubscribeBar) |
| `gateway/app.py` | the single backend entrypoint (`python -m gateway`) — one process, all transports + the app |
| `gateway/bus.py` | the LCM+Zenoh bus tap → one normalized stream every transport reads from |
| `gateway/{data,media}.py` | `/ws` data plane (topics + teleop/goal/rpc) · `/media` camera plane |
| `gateway/transports/` | the `/sse` `/poll` `/rtc` + WebTransport bench transports |
| `scenarios/` | dimos publisher blueprints — live data sources (`nav`/`arm`/`cam`) + `bench.py` (the browser-bench `/bench/*` source) |
| `app/src/bench.tsx` · `panels/BenchTab.tsx` | the in-browser benchmark (`/bench.html`) + in-app Bench tab |

The original teaching prototype (`app/src/bus.ts`, `app/src/widgets/`, `WALKTHROUGH.md`) is the
parts-bin this was extracted from.

## Status / next
- ✅ SDK + React app + teleop (verified in Chrome incl. driving the robot) + transport benchmark.
- ✅ **Single service** — the gateway: one Python process taps **LCM + Zenoh** and serves the app +
  all five delivery mechanisms (WS · SSE · poll · WebRTC-data · WebTransport) + the camera; the old
  per-transport gateways (Deno↔LCM / Python↔Zenoh / media node / zenoh-ts bridge) are **retired**.
- ✅ **Media plane** — camera over WebRTC / WebCodecs / JPEG, capability-negotiated (`useVideo` +
  cam-mode toggle) on the **`/media`** path of the single service.
- ✅ **RPC bridge** — `client.call("GO2Connection","standup")` → whitelisted dimos `@rpc` (Commands panel).
- ⚠️ **Rerun 3D panel** (`panels/RerunPanel.tsx`): the stock `@rerun-io/web-viewer-react@0.32.0-alpha.1`
  **embeds, loads (after a Vite `optimizeDeps.exclude` for the wasm MIME), connects to dimos
  `serve_grpc` :9877, and streams** (console-confirmed) — but **paints a black canvas**: the `RerunBridge` firehoses every
  stream undecimated, so the browser WASM viewer blows its **128 MiB channel budget in ~1 s** and its
  **~2.3 GiB heap ceiling in ~1 min**, then thrashes GC and never paints (root cause: the undecimated
  firehose ceiling). **The native
  `dimos-viewer` (:9876) handles the same feed fine — it's the 3D path; a browser-viable 3D would
  need server-side decimation.** Start the feed with:
  `DIMOS_TRANSPORT=lcm python -c "from dimos.visualization.rerun.bridge import run_bridge; run_bridge(rerun_open='web', rerun_web=True)"`.
- ✅ **Forked `dimos-viewer` WASM: BUILT.** `cargo run -p re_dev_tools -- build-web-viewer --debug`
  with Homebrew LLVM (`CC_wasm32_unknown_unknown=$(brew --prefix llvm)/bin/clang` — Apple clang lacks
  the wasm32 target) produces `web_viewer/re_viewer_bg.wasm` (128 MB *debug*; `--release` shrinks ~3×).
  ⏭ Next: serve that `web_viewer/` dir + iframe it (gRPC proxy URL) for in-3D teleop/click-to-nav via
  `RerunWebSocketServer` :3030 — the fork's edge over the stock viewer. **But the fork's WASM build
  inherits the same browser heap ceiling** (the firehose above), so it won't survive the feed either;
  the **native** `dimos-viewer` is what holds it, and a browser-viable 3D still needs server-side
  decimation.
- ✅ **No head-of-line blocking under loss** — the **WebRTC-data** and **WebTransport** mechanisms
  (UDP) stay smooth where WS (TCP HoL) stalls; plus **on-demand** subscribe + a **QoS** layer
  (rate-limit / conflation / priority) so important data survives a saturated link. (The earlier
  zenoh-ts-direct path that proved true per-client on-demand was retired with its bridge.)
- ✅ **Camera** (`useImageTopic`) + **transport selector** (topbar dropdown over all five delivery mechanisms).
- ✅ **Tabbed 3D** (Rerun) ⇄ **zoomable/pannable WorldView**, with streams scoped to the active tab.
