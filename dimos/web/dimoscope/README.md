# dimoscope — DimOS topics in the browser

A **transport-agnostic browser SDK** to subscribe to DimOS topics, visualize them, and
teleoperate — plus the gateways and a transport benchmark. The browser is a first-class bus
client: it decodes messages itself with [`@dimos/msgs`](https://jsr.io/@dimos/msgs) (the 8-byte
type hash is self-describing), so the gateway is a thin byte-relay and the **same SDK works over
any transport**.

```
robot / sim ─► DimOS bus (LCM | Zenoh)
   │
   ├─► DATA gateway  (thin byte-relay + the teleop/goal/RPC trust boundary)
   │     Python↔Zenoh  servers/gateway_zenoh.py  :8088        zenoh-ts (direct):
   │     Bun↔LCM       servers/gateway.ts        :8089        browser speaks Zenoh via
   │       └─ WebSocket: raw LCM frames + JSON control        zenoh-bridge-remote-api :10000
   │          (subscribe · teleop · goal · rpc)               (no gateway in the read path)
   │
   └─► MEDIA node  (camera only — its OWN Zenoh peer)
         servers/media_server.py  :8092
         WebRTC / WebCodecs, encode-once-per-topic → fan out
   ▼
@dimos/topics  (decode via @dimos/msgs · on-demand · stats · client.call RPC · useVideo media)
   ▼
@dimos/react ─► app (WorldView · Camera · Rerun 3D · Pose · Stats · Commands) + teleop
```

## Quickstart (real DimOS, no hardware)

From the dimos repo root (uses the repo `.venv`):

```bash
# 1) a robot + sensors (publish /odom + /map over LCM)
.venv/bin/python examples/simplerobot/simplerobot.py --headless
.venv/bin/python examples/fakesensors.py

# 2) the servers — data gateways + the camera media node, each on its own port
cd dimos/web/dimoscope && bun install && bash servers/start-all.sh
#   Python↔Zenoh :8088 · Bun↔LCM :8089 · media node :8092 · zenoh-ts bridge :10000
#   (for the WebRTC/WebCodecs camera, run a ZENOH camera source, e.g.
#    DIMOS_TRANSPORT=zenoh uv run dimos --simulation mujoco run unitree-go2)

# 3) the app
cd dimos/web/dimoscope/app && bun install && bun run dev                # http://localhost:5173
```

The app auto-discovers topics, draws the map + robot + trail in **WorldView**, shows a live camera
+ pose readout + a **Stats** panel (hz / kB/s / latency), and drives the robot with **WASD / arrows**
(gateway clamps velocity + deadman-stops). Headless checks: `bun run servers/probe.ts`,
`bun run bench/sdk_smoke.ts`, `bun run bench/teleop_test.ts`.

## Transports — pick one from the topbar dropdown

Same app, same `@dimos/topics` SDK, three swappable transports (the dropdown rebuilds the client):

| Option | How the browser reaches the bus | Port |
|---|---|---|
| **Python↔Zenoh** | gateway re-wraps Zenoh samples → WS | `:8088` |
| **Bun↔LCM** | gateway relays LCM multicast → WS | `:8089` |
| **zenoh-ts (direct)** | browser speaks Zenoh itself via [`@eclipse-zenoh/zenoh-ts`](https://www.npmjs.com/package/@eclipse-zenoh/zenoh-ts) → a `zenoh-bridge-remote-api` WebSocket — **no gateway in the read path** | `:10000` |

`bash servers/start-all.sh` launches all three. For data over Zenoh + LCM at once, run DimSim:
`DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run unitree-go2`.

**zenoh-ts bridge** (one-time): `cargo install zenoh-bridge-remote-api` (pin to your Zenoh version —
the repo uses 1.9). start-all.sh runs it on `:10000` (peer mode, joins the dimos Zenoh net). The
direct path is **true per-client end-to-end on-demand**: only the keys you `declareSubscriber` ever
transit to the browser (vs the LCM gateway, which receives every topic off multicast). **Safety:**
zenoh-ts handles only the *read* path — teleop/goal still go through a gateway so its velocity clamp +
deadman + stop-on-disconnect apply (a browser doing raw Zenoh `put` would bypass all of that).

## Media plane (camera)

The camera is the one heavy stream (~11 MB/s JPEG, ~55 MB/s raw), so it rides its **own plane** beside
the topic data — a **standalone media node** (`servers/media_server.py`, **:8092**), its own Zenoh peer.
`useVideo(topic)` negotiates the best available delivery (capability + graceful fallback), picked by the
topbar **cam:** toggle:

| Mode | How | Served by |
|---|---|---|
| **webrtc** | aiortc encodes once; browser HW-decodes a `<video>` | media node `:8092` |
| **webcodecs** | libx264 NAL chunks over WS → `VideoDecoder` → `<canvas>` | media node `:8092` |
| **jpeg** | the Image topic itself, decoded in-browser (universal floor) | data gateway |

Encode-once-per-topic → fan out to N viewers; multiple cameras = multiple topics, one node serves each.
**Caveat:** the media node is a Zenoh peer, so webrtc/webcodecs need a **Zenoh** camera source (the jpeg
floor works on any transport via the data plane).

## Commands (RPC)

The browser invokes **whitelisted dimos `@rpc` commands** through the gateway —
`client.call("GO2Connection", "standup")` → `{op:"rpc"}` → `rpc_backend().call_sync(...)` → result. The
gateway holds a **server-side whitelist** (default `standup`/`liedown`; override
`DIMOS_GATEWAY_RPC="Module/method,…"`) and advertises it in `hello`; the **Commands** panel renders a
button per advertised command (empty on Bun↔LCM, which has no RPC bridge). Velocity stays on the
clamped/deadman teleop path — RPC is for discrete commands only.

## The SDK (`@dimos/topics`)

```ts
import { connect } from "@dimos/topics";
const client = await connect({ url: "ws://localhost:8088" });

client.listTopics();                                  // [{topic, type}, …] (live discovery)
const odom = client.topic<PoseStamped>("/odom");
odom.subscribeLatest((pose, meta) => { … meta.latencyMs … });
odom.setRateLimit(15);                                // backpressure (also asks gateway to downsample)
odom.stats();                                         // { hz, bytesPerSec, dropped, lastLatencyMs }
client.teleop(0.5, 0.0);                              // safe: gateway clamps + TTL/deadman watchdog
await client.call("GO2Connection", "standup");        // RPC: invoke a whitelisted dimos @rpc command
client.commands;                                      // [{target, method, label}] the gateway advertises
```
React: `DimosProvider`, `useTopics`, `useTopicLatest`, `useTopicRef` (no re-render → rAF canvas),
`useTopicStats`, `useTeleop`, `useVideo` (camera → `<video>`/`<canvas>`), `useRpc`/`useCommands`.
**On-demand:** a topic is subscribed on the wire only while it has a live subscriber.

## Benchmark

```bash
bench/run.sh           # Bun↔LCM gateway (:8090)
bench/run.sh zenoh     # Python↔Zenoh gateway (:8091)
bench/run.sh ts        # zenoh-ts direct via the :10000 bridge — headless under Deno
bench/run.sh all       # all three → combined bench/RESULTS.md (end-to-end latency)
pytest bench/test_bench.py -s
```

**zenoh-ts benches headless under Deno, not Bun:** its wasm-bindgen *bundler*-target WASM fails in
Bun (`wasm.__wbindgen_start is not a function`) but Deno instantiates it (zenoh-ts's own examples are
Deno-based). So `run.sh all` covers all three via the same `@dimos/topics/bench` core — gateways under
Bun, zenoh-ts under Deno (`bench/bench_deno.ts`).

There's also a **browser** bench — the app's real runtime — for an end-to-end cross-check:
```bash
bash bench/serve-bench.sh              # 3 servers + bench_publisher on both buses
# then open http://localhost:5173/bench.html → Run  (copy-Markdown / download-JSON)
```

Results: **[bench/RESULTS.md](bench/RESULTS.md)** (headless, all 3) + **[bench/RESULTS-browser.md](bench/RESULTS-browser.md)**
(in-browser, all 3) — same measurement core. Headline: **throughput parity** (~330 hz headless / ~417
in-browser) across all three; **sub-ms end-to-end latency** (p50 ~0.3–0.4, p95 ~0.7–1 ms); **~75% on-demand**
bandwidth cut — for zenoh-ts that's *true* per-client on-demand (only declared keys transit). The gateways'
own WS-hop latency (`run.sh lcm`/`zenoh`) shows **Zenoh's tail ~3–6× lower** than LCM multicast.

## Layout
| Path | What |
|---|---|
| `packages/topics/` | `@dimos/topics` — transport iface + adapters (`gatewayWs`, `zenohTs`) + **media plane** (`media.ts`, `jpegTopicMedia`/`webRtcMedia`/`webCodecsMedia`) + client (incl. `call` RPC), topic, decode, stats |
| `packages/react/` | `@dimos/react` — hooks (`useTopics`, `useVideo`, `useTeleop`, `useRpc`/`useCommands`, …) |
| `app/` | Vite example app (`panels/`: WorldView, Camera, PoseReadout, TeleopPad, StatsBar, CommandsPanel, SubscribeBar) |
| `servers/gateway_zenoh.py` | Python↔Zenoh **data gateway** — relay + teleop/goal/RPC |
| `servers/gateway.ts` | Bun↔LCM data gateway (LC03 reassembly, on-demand, teleop safety) |
| `servers/media_server.py` | **media node** `:8092` — camera WebRTC/WebCodecs (own Zenoh peer) |
| `bench/` | publisher + headless bench + `run.sh` + `test_bench.py` + RESULTS.md |

The original prototype (`bridge.ts`, `app/src/bus.ts`, `app/src/widgets/`, `WALKTHROUGH.md`) is the
parts-bin this was extracted from.

## Status / next
- ✅ SDK + Bun↔LCM gateway + React app + teleop (verified in Chrome incl. driving the robot) +
  benchmark + Python↔Zenoh gateway + comparison.
- ✅ **Media plane** — camera over WebRTC / WebCodecs / JPEG, capability-negotiated (`useVideo` +
  cam-mode toggle) on a **standalone media node** (`:8092`, own Zenoh peer — split out of the gateway).
- ✅ **RPC bridge** — `client.call("GO2Connection","standup")` → whitelisted dimos `@rpc` (Commands panel).
- ⚠️ **Rerun 3D panel** (`panels/RerunPanel.tsx`): the stock `@rerun-io/web-viewer-react@0.32.0-alpha.1`
  **embeds, loads (after a Vite `optimizeDeps.exclude` for the wasm MIME), connects to dimos
  `serve_grpc` :9877, and streams** (console-confirmed) — but the viewport isn't painting geometry yet
  (needs a blueprint/view config or a WebGL-in-embed nudge). Start the feed with:
  `DIMOS_TRANSPORT=lcm python -c "from dimos.visualization.rerun.bridge import run_bridge; run_bridge(rerun_open='web', rerun_web=True)"`.
- ✅ **Forked `dimos-viewer` WASM: BUILT.** `cargo run -p re_dev_tools -- build-web-viewer --debug`
  with Homebrew LLVM (`CC_wasm32_unknown_unknown=$(brew --prefix llvm)/bin/clang` — Apple clang lacks
  the wasm32 target) produces `web_viewer/re_viewer_bg.wasm` (128 MB *debug*; `--release` shrinks ~3×).
  ⏭ Next: serve that `web_viewer/` dir + iframe it (gRPC proxy URL) for in-3D teleop/click-to-nav via
  `RerunWebSocketServer` :3030 — the fork's edge over the stock viewer.
- ✅ **True end-to-end on-demand** — the **zenoh-ts (direct)** transport: the browser
  `declareSubscriber`s Zenoh keys itself via the remote-api bridge, so only subscribed keys transit
  (verified in-app; teleop/goal kept on the gateway for the safety boundary).
- ✅ **Camera** (`useImageTopic`) + **transport selector** (topbar dropdown over all three transports).
- ✅ **Tabbed 3D** (Rerun) ⇄ **zoomable/pannable WorldView**, with streams scoped to the active tab.
