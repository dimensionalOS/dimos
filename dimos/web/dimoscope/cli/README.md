# `dtop` — headless @dimos/topics CLI

The no-browser surface for the DimOS topics SDK: connect over any delivery transport, inspect
topics, measure throughput/latency/**loss**, and generate typed topic maps. The browser app is a
*second* consumer of the same SDK — this is where you iterate and prove the data path first.

## Run
```bash
deno task dtop <cmd> [--transport gatewayWs|zenohTs|sse|httpPoll] [--url URL] [--dur MS]
# or: deno run -A cli/dtop.ts <cmd> …
```

| Command | What |
|---|---|
| `list` | discovered topics + types |
| `echo <topic>` | decode + print messages (`--count N` to stop) |
| `stats [topic]` | live hz / kB/s / latency / dropped / **loss%** (from seq gaps) |
| `probe` | connect + verify data flows; exit 0/1 (reliability/health check) |
| `bench` | run `BENCH_SCENARIOS` via the shared measurement core → markdown table (p50/p95/p99/max/std/loss%) |
| `gen-types [--out F]` | emit a typed `DimosTopics` map from live discovery |

Examples:
```bash
deno task dtop list                                   # against the default Bun↔LCM gateway
deno task dtop bench --transport sse --dur 4000       # benchmark the SSE delivery path
deno task dtop stats /bench/p0                         # live loss% on a seq-stamped topic
deno task dtop echo /custom --decode server-json       # render a user-defined type, zero rebuild
deno task dtop gen-types --out app/src/dimos.topics.gen.ts
```

## Notes
- **`gen-types`** validates every discovered type against `@dimos/msgs.getTypeNames()`: known types
  become typed (`"/odom": geometry_msgs.PoseStamped`), user-defined ones fall through to `unknown`
  (they still stream + decode at runtime). Codegen and dynamic decode compose.
- **`--decode server-json`** (gatewayWs only) asks the gateway to decode and ship JSON, so a custom
  message renders with no `@dimos/msgs` entry — the zero-rebuild path (see
  [`docs/custom-messages-in-the-browser.md`](../docs/custom-messages-in-the-browser.md)).
- **`loss%`** is computed from per-topic sequence gaps (`MessageMeta.seq`, stamped by the load
  source); `n/a` for named-frame topics that carry no numeric seq. Distinct from `dropped` (client
  rate-limit coalescing).
- **Transports**: `gatewayWs` / `zenohTs` / `sse` / `httpPoll` all run headless here. `webRtcData`
  is **browser-only** (needs `RTCPeerConnection`, absent in Deno), so it's not a CLI transport.

## How it composes with the parallel work
- **Imports the shared measurement core** (`packages/topics/src/bench.ts`) for `bench` — no
  reimplementation; picks up p99/std/loss automatically.
- **`stats` loss%** tracks `MessageMeta.seq` in the CLI (TopicStats doesn't expose seq) — same
  pooling as `bench.ts`.
- Imports the **specific data-plane modules** (not the `index.ts` barrel) so the DOM-only media
  plane stays out of the Deno graph.
- Registered as `deno task dtop`; covered by `deno task check` + `deno task test`.

See also: [`docs/data-path.md`](../docs/data-path.md), [`bench/RESULTS-mechanisms.md`](../bench/RESULTS-mechanisms.md).
