# dimoscope — DimOS topics in the browser

A **transport-agnostic browser SDK** to subscribe to DimOS topics, visualize them, and
teleoperate — plus the gateways and a transport benchmark. The browser is a first-class bus
client: it decodes messages itself with [`@dimos/msgs`](https://jsr.io/@dimos/msgs) (the 8-byte
type hash is self-describing), so the gateway is a thin byte-relay and the **same SDK works over
any transport**.

```
robot / sim ─► DimOS bus (LCM | Zenoh)
                  │
        ┌─────────┴──────── gateway (thin relay) ─────────┐      zenoh-ts (direct):
        │  Python↔Zenoh (servers/gateway_zenoh.py) :8088   │      browser speaks Zenoh via
        │  Bun↔LCM      (servers/gateway.ts)       :8089   │      zenoh-bridge-remote-api :10000
        └─────────┬────────────────────────────────────────┘      (no gateway in the read path)
              WebSocket (raw frames + JSON control)        ╲              │
                  ▼                                         ╲             ▼
        @dimos/topics  (decode via @dimos/msgs · backpressure · stats · on-demand)  ◄─ pick in the UI
                  ▼
        @dimos/react ─► app (WorldView 2D · Camera · Rerun 3D · Pose · Stats) + teleop ▲
```

## Quickstart (real DimOS, no hardware)

From the dimos repo root (uses the repo `.venv`):

```bash
# 1) a robot + sensors (publish /odom + /map over LCM)
.venv/bin/python examples/simplerobot/simplerobot.py --headless
.venv/bin/python examples/fakesensors.py

# 2) the gateways — all three transports at once, each on its own port
cd dimos/web/dimoscope && bun install && bash servers/start-all.sh
#   Python↔Zenoh :8088 · Bun↔LCM :8089 · zenoh-ts bridge :10000

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

## The SDK (`@dimos/topics`)

```ts
import { connect } from "@dimos/topics";
const client = await connect({ url: "ws://localhost:8090" });

client.listTopics();                                  // [{topic, type}, …] (live discovery)
const odom = client.topic<PoseStamped>("/odom");
odom.subscribeLatest((pose, meta) => { … meta.latencyMs … });
odom.setRateLimit(15);                                // backpressure (also asks gateway to downsample)
odom.stats();                                         // { hz, bytesPerSec, dropped, lastLatencyMs }
client.teleop(0.5, 0.0);                              // safe: gateway clamps + TTL/deadman watchdog
```
React: `DimosProvider`, `useTopics`, `useTopicLatest`, `useTopicRef` (no re-render → rAF canvas),
`useTopicStats`, `useTeleop`. **On-demand:** a topic is subscribed on the wire only while it has a
live subscriber.

## Benchmark

```bash
bench/run.sh           # Bun↔LCM gateway
bench/run.sh zenoh     # Python↔Zenoh gateway
bench/run.sh all       # both → combined bench/RESULTS.md
pytest bench/test_bench.py -s
```
**All 3 transports, in the real browser runtime** (the only way to bench zenoh-ts — its
wasm-bindgen *bundler*-target WASM won't instantiate in Bun/Node):

```bash
bash bench/serve-bench.sh              # 3 servers + bench_publisher on both buses
# then open http://localhost:5173/bench.html → Run  (copy-Markdown / download-JSON)
```

Results: **[bench/RESULTS.md](bench/RESULTS.md)** (headless gateways) + **[bench/RESULTS-browser.md](bench/RESULTS-browser.md)**
(in-browser, all 3) — both off the same `@dimos/topics/bench` measurement core.
Headline: headless, **sub-ms p50** WS-hop latency on both gateways with **Python↔Zenoh ~6× lower p95**
(reliable transport vs LCM multicast jitter) → "Python is slow" is a non-issue for a byte-relay gateway.
In-browser, all three hit **throughput parity (~417 hz)** with comparable end-to-end latency, and
**on-demand cuts ~75%** bandwidth — for zenoh-ts that's *true* per-client on-demand (only declared keys transit).

## Layout
| Path | What |
|---|---|
| `packages/topics/` | `@dimos/topics` — transport iface + `gatewayWs` adapter, client, topic, decode, stats |
| `packages/react/` | `@dimos/react` — hooks |
| `app/` | Vite example app (`panels/`: WorldView, PoseReadout, TeleopPad, StatsBar) |
| `servers/gateway.ts` | Bun↔LCM gateway (LC03 reassembly, on-demand, teleop safety) |
| `servers/gateway_zenoh.py` | Python↔Zenoh gateway (same WS protocol) |
| `bench/` | publisher + headless bench + `run.sh` + `test_bench.py` + RESULTS.md |

The original prototype (`bridge.ts`, `app/src/bus.ts`, `app/src/widgets/`, `WALKTHROUGH.md`) is the
parts-bin this was extracted from.

## Status / next
- ✅ SDK + Bun↔LCM gateway + React app + teleop (verified in Chrome incl. driving the robot) +
  benchmark + Python↔Zenoh gateway + comparison.
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
