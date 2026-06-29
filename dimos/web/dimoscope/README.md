# dimoscope — DimOS topics in the browser

A **transport-agnostic browser SDK** to subscribe to DimOS topics, visualize them, and
teleoperate — plus the gateways and a transport benchmark. The browser is a first-class bus
client: it decodes messages itself with [`@dimos/msgs`](https://jsr.io/@dimos/msgs) (the 8-byte
type hash is self-describing), so the gateway is a thin byte-relay and the **same SDK works over
any transport**.

```
robot / sim ─► DimOS bus (LCM | Zenoh)
                  │
        ┌─────────┴───────── gateway (thin relay) ─────────┐
        │  Bun↔LCM (servers/gateway.ts, default)            │   ← byte-relay + on-demand filter
        │  Python↔Zenoh (servers/gateway_zenoh.py, alt)     │     + teleop safety
        └─────────┬─────────────────────────────────────────┘
              WebSocket (raw LCM frames + JSON control)
                  ▼
        @dimos/topics  (decode via @dimos/msgs · backpressure · stats · on-demand)
                  ▼
        @dimos/react  ─►  example app (WorldView · Pose · Teleop · Stats)  +  teleop ▲
```

## Quickstart (real DimOS, no hardware)

From the dimos repo root (uses the repo `.venv`):

```bash
# 1) a robot + sensors (publish /odom + /map over LCM)
.venv/bin/python examples/simplerobot/simplerobot.py --headless
.venv/bin/python examples/fakesensors.py

# 2) the gateway (Bun↔LCM)
cd dimos/web/dimoscope && bun install && bun run servers/gateway.ts     # ws://localhost:8090

# 3) the app
cd dimos/web/dimoscope/app && bun install && bun run dev                # http://localhost:5173
```

The app auto-discovers topics, draws the map + robot + trail in **WorldView**, shows a live pose
readout + a **Stats** panel (hz / kB/s / latency), and drives the robot with **WASD / arrows**
(gateway clamps velocity + deadman-stops). Headless checks: `bun run servers/probe.ts`,
`bun run bench/sdk_smoke.ts`, `bun run bench/teleop_test.ts`.

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
pytest bench/test_bench.py -s
```
See **[bench/RESULTS.md](bench/RESULTS.md)**. Headline: **sub-ms p50 latency** on both; **Python↔Zenoh
has ~3× lower p95** (reliable transport vs LCM multicast jitter) → "Python is slow" is a non-issue
for a byte-relay gateway. On-demand subscription cuts WS-hop bandwidth ~75%.

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
- ✅ SDK + Bun↔LCM gateway + React app + teleop (verified in Chrome) + benchmark + Python↔Zenoh gateway.
- ⏭ **3D viewer:** embed the **stock** Rerun web viewer (`@rerun-io/web-viewer-react@0.32.0-alpha.1`)
  fed by dimos `serve_grpc` :9877. The **forked `dimos-viewer`** (in-3D teleop/click-to-nav) needs a
  cold Rust→WASM compile of the whole Rerun viewer — Apple `clang` lacks the `wasm32` target, so it
  needs Homebrew LLVM (`CC` override); deferred (stock is the reliable path).
- ⏭ True end-to-end on-demand on the Zenoh gateway (per-client `declareSubscriber`/`undeclare`).
- ⏭ Camera (`useImageTopic`) + a transport selector in the app.
