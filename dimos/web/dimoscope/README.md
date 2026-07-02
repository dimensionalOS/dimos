# dimoscope — DimOS topics in the browser (Dimos JS)

`@dimos/web` (Dimos JS) is a transport-agnostic browser SDK to subscribe to DimOS topics, visualize
them, and teleoperate — plus a single backend service and a transport benchmark. The browser decodes
messages itself with [@dimos/msgs](https://jsr.io/@dimos/msgs) (the 8-byte type hash is
self-describing), so the service is a thin byte-relay and the same SDK works over any transport.

```
robot / sim ─► DimOS bus (LCM | Zenoh)
   │
   └─► the backend (`deno task serve`) — the Python gateway (http://0.0.0.0:8080) + the native Rust
         WebTransport sidecar (UDP :8443, fed over a unix socket). The gateway taps BOTH
         LCM + Zenoh → one normalized stream, fanned out over every transport:
           GET /            the built web app  (the SDK consumer itself)
           WS  /ws          data plane: topics + teleop/goal/rpc  (the trust boundary)
           GET /sse · /poll Server-Sent Events · HTTP long-poll
           WS  /rtc         WebRTC DataChannel          UDP :8443  WebTransport (QUIC, the sidecar)
           WS  /media       camera: webrtc / webcodecs / jpeg
           GET /cert        the sidecar's self-signed cert hash
   ▼
@dimos/web  (decode via @dimos/msgs · on-demand · QoS · client.call RPC · useVideo media)
   ▼
@dimos/react ─► app (WorldView · Camera · Pose · Stats · Commands · Topics) + teleop
```

## Quickstart

Prereqs: [uv](https://docs.astral.sh/uv/) + [Deno](https://deno.com/) 2.x (no Node — Deno runs Vite) +
[Rust](https://rustup.rs/) (cargo builds the WebTransport sidecar); Chrome/Edge for WebTransport
(everything else falls back to WebSocket).

From the **repo root**, with only the `web` extra — every line is copy-paste:

```bash
uv sync --extra web                    # backend deps (FastAPI + zenoh), once
cd dimos/web/dimoscope
deno install && deno task build        # frontend deps + the app bundle the gateway serves, once

deno task serve                        # tab 1 — gateway + WT sidecar → http://localhost:8080/
                                       #         (first run compiles the sidecar, ~2 min)
deno task load                         # tab 2 — /load/* data source (multi-rate lanes + crankable flood)
```

Open **http://localhost:8080/** — topics appear in the sidebar; the **Topics** tab shows per-topic
stream cards (live hz / kB/s / latency) with per-topic QoS controls. `deno task app` runs a Vite dev
server on :5173 (hot reload) instead of the built bundle; the gateway also runs cwd-free as
`uv run python -m dimos.web.dimoscope.gateway`.

Want a robot instead of synthetic lanes? Install the sim stack once and swap tab 2 for the dog:

```bash
uv sync --extra web --extra unitree --extra sim --extra mapping   # once — dimsim + go2 (repo root)
deno task dog     # teleoperable go2 (dimsim) + camera + the same /load/* lanes
```

WASD drives it; close the tab → deadman stop. Plain go2 variants: `deno task sim` (mujoco) ·
`sim:dimsim` · `sim:replay`; real-world data profiles: `deno task scope:{nav,arm,cam,bench}`
([scenarios](scenarios/README.md)).

## The SDK in 15 lines — list topics, subscribe, control the push rate

```ts
import { createDimosClient } from "@dimos/web";

const client = createDimosClient();          // default transport: WebSocket
await client.connect("ws://localhost:8080"); // "/ws" appended automatically

console.table(client.listTopics());          // [{topic, type}, …] — live discovery, zero config
const sub = client.subscribe("/load/cloud", (m) => {
  console.log(m.data, m.meta.latencyMs);     // m.data = the decoded dimos-lcm message (@dimos/msgs)
});

// Server-side push control, per client per topic. The gateway downsamples/sheds before sending
// (bytes leave the wire); other subscribers of the same topic are unaffected:
client.setQos("/load/cloud", { maxHz: 2, priority: "low", reliability: "best-effort" });

const one = await client.peek("/load/grid", { timeoutMs: 2000 }); // pull-based one-shot
sub.unsubscribe();                           // on-demand: bytes stop on the wire
```

Also on the client: `client.teleop(lin, ang)` (the service clamps velocity + runs a TTL deadman),
`client.subscribeAll(cb)` (firehose), `client.topic(name)` (rich handle — `getLatest()`,
`setQos({ maxHz: 15 })`, `stats()` → `{ hz, bytesPerSec, lastLatencyMs, count }`), and
`await client.modules.GO2Load.start_bench(hz, bytes, kind)` — RPC via a typed Proxy.

Server-side you write nothing. The gateway self-discovers topics from both LCM and Zenoh, assigns
QoS lanes by a type/name heuristic (override per deployment via `qos.rules.json` — see
`qos.rules.example.json`), and runs a per-client priority outbox. RPC is a server-side whitelist —
the `RPC_COMMANDS` list in `gateway/egress.py` (GO2Connection `standup`/`liedown` + GO2Load
`start_all`/`stop_all`/`start_bench`/`stop_bench`); edit it to expose your own `@rpc`.

Typed topics + commands are generated from the blueprint: `deno task gen-types` writes
`app/src/dimos.topics.gen.ts` (scenarios + `go2_load.py`), and
`createDimosClient<DimosTopics, DimosCommands>()` autocompletes topic names, message fields, and RPC
signatures — see [`packages/web/README.md`](packages/web/README.md).

## Transports — pick a delivery mechanism from the topbar dropdown

Same app, same `@dimos/web` SDK, five swappable delivery mechanisms — all on the one service,
same-origin. The dropdown rebuilds the client; `?gw=host:port` overrides the origin, e.g. a remote VPS
for real-WAN testing. They carry identical self-describing frames, so the codec is unchanged; only the
wire differs:

| Mechanism        | Wire              | Path        | Notes                                                                                                        |
| ---------------- | ----------------- | ----------- | ------------------------------------------------------------------------------------------------------------ |
| **WebSocket**    | TCP               | `/ws`       | duplex, reliable; carries teleop/goal/rpc; the universal fallback for Auto                                   |
| **SSE**          | TCP/HTTP          | `/sse`      | server→client only (binary→base64)                                                                           |
| **HTTP poll**    | TCP/HTTP          | `/poll`     | universal req/resp baseline                                                                                  |
| **WebRTC data**  | SCTP/DTLS/**UDP** | `/rtc`      | configurable unordered/unreliable → no TCP head-of-line blocking                                             |
| **WebTransport** | HTTP/3 **QUIC**   | UDP `:8443` | streams + datagrams, no HoL; also carries teleop/rpc; the Auto default; cert hash from `/cert` (Chrome/Edge); served by the native Rust sidecar (`gateway/wt-sidecar` — bulk numbers in [benchmarks §3](docs/benchmarks.md)) |

## Benchmark

The benchmark runs **in the real browser** across all 5 delivery mechanisms (WS · SSE · poll ·
WebRTC-data · WebTransport), so WebRTC/WebTransport are measured on the actual browser stacks. See
**[docs/benchmarks.md](docs/benchmarks.md)** for the methodology, QoS, real-WAN runbook, and full tables.

```bash
deno task serve   # the backend on :8080 (+ WT QUIC :8443)
deno task load    # the /load/* lanes + the GO2Load start_bench/stop_bench flood knob
# open http://localhost:8080/ → Topics tab → Benchmark drawer
```

## Development

```bash
deno task check                                     # typecheck: SDK + react + app
deno task test                                      # SDK unit tests
uv run pytest dimos/web/dimoscope/gateway/tests -q  # gateway unit tests — run from the REPO ROOT
deno task fmt && deno task lint
```

`deno task serve` builds the **native WT sidecar** (Rust) with cargo, then launches it beside the
gateway — the sidecar owns UDP `:8443`, fed over a unix socket (`gateway/pipe.py`). To iterate on the
sidecar alone, run `uv run python -m gateway` + `deno task wt-sidecar` in separate shells: kill/rebuild
the sidecar and the pipe reconnects while the gateway keeps serving (numbers in
[benchmarks §3](docs/benchmarks.md)).

Without deno — prod/VPS shape, exactly what `deno task serve` automates:

```bash
cargo build --release --manifest-path gateway/wt-sidecar/Cargo.toml   # once (~2 min)
DIMOS_TRANSPORT=zenoh uv run python -m gateway &                      # :8080 — RPC backend must match the blueprints'
gateway/wt-sidecar/target/release/wt-sidecar &                        # QUIC/UDP :8443 — WebTransport
```

The two processes start in any order and find each other over the pipe; `/cert` serves 503 until the
sidecar is up (Auto lands on WebSocket meanwhile). Full VPS runbook — firewall, the dog, netem:
[benchmarks §3](docs/benchmarks.md).
