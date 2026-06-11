# lcmflow constellation dashboard

A high-definition, real-time web visualization of DimOS transport: every
module is a glowing node in a force-directed spiderweb, every topic a hub
between them, and every packet a comet flying along the strands. Big
bursts (raw images, point clouds) detonate as supernova shockwaves.

```
┌──────────┐  UDP multicast   ┌────────────┐  WebSocket (metadata   ┌─────────┐
│ DimOS    │ ───────────────► │ server.ts  │  only, 50 ms batches)  │ browser │
│ modules  │                  │ (deno)     │ ─────────────────────► │ canvas  │
└──────────┘                  └────────────┘                        └─────────┘
                                    ▲
                                    │ "Transport" events (module ↔ topic ↔
                                    │ transport ↔ direction) parsed from the
                                    │ newest run's main.jsonl
                                    └── logs/<run-id>/main.jsonl
```

Packet payloads never reach the browser — the bridge forwards only
`{channel, count, bytes}` batches, so a 40 MiB/s image stream costs the
page a few hundred bytes per second. The module graph is recovered from
the running instance's structured log, which records every stream
binding (module, topic, transport) at startup; it covers all transports
(LCM, SHM, ROS, DDS), while live packet animation covers LCM.

Stream **direction** (which side publishes vs subscribes, for the arrow
flow) is not in that log, so rather than change the coordinator the
server shells out once to `python -m dimos.utils.cli.lcmflow.topology`,
which introspects the blueprint's `In[]`/`Out[]` declarations and prints
`{"<Module>|<stream>": "in"|"out"}`. If that's unavailable the dashboard
simply falls back to undirected edges — no core dependency.

## Run

```bash
# 1. Start any DimOS stack (the dashboard needs a robot running for
#    the module graph; start it first)
dimos --replay run unitree-go2 --daemon

# 2. Start the dashboard (requires deno)
dimos lcmflow dashboard            # http://localhost:8090
# or directly:
deno run --allow-net --allow-read --allow-env --unstable-net server.ts

# Stop it
dimos lcmflow dashboard stop

# Options
dimos lcmflow dashboard --port 9000 --log /path/to/main.jsonl
```

If the port is taken, the server walks up to the next free one
(8090 → 8091 → …) and prints the URL it actually bound.

## Controls

| Input | Action |
|-------|--------|
| drag  | pin/move a node |
| `space` | pause animation (stats keep flowing) |
| `t` | toggle the per-topic traffic panel |
| `f` | toggle FPS / particle counter |

## Visual language

- **Module nodes**: cyan hexagonal rings, pulse with traffic through them.
- **Topic hubs**: small orbs colored by message type (Image=yellow,
  PointCloud2=purple, OccupancyGrid=blue, TFMessage=green, …) — the same
  palette as the `lcmflow` TUI.
- **Comets**: one per packet (capped per batch), sized by `log2(bytes)`,
  travelling publisher → hub → subscribers when direction is known.
- **Shockwaves**: any >1 MiB burst rings the hub with an expanding wave.
- **Strands**: edge brightness follows the hub's recent traffic.
- **Dashed hubs**: published but no module subscribes in this blueprint —
  a dangling stream costing bandwidth with no consumer.

## How packets are sniffed

`server.ts` subscribes to every channel through the `@dimos/lcm` client,
exactly like `examples/language-interop/ts`:

```ts
const lcm = new LCM();
await lcm.start();
lcm.subscribeRaw("*", (msg) => note(msg.channel, msg.data.byteLength));
await lcm.run();
```

`subscribeRaw` fires once per fully-reassembled message; we read only
`data.byteLength`, so payloads are never decoded. Note the wildcard is
glob-style (`"*"`), **not** the regex `".*"` — the client compiles
`".*"` to `/^\..*$/`, which matches nothing because channels start with
`/`.
