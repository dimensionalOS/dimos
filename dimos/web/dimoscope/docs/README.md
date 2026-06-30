# dimoscope docs

**dimoscope** is a research effort to build a solid JavaScript SDK/framework for integrating with the DimOS stack from the browser: a transport-agnostic topics SDK (`@dimos/topics`), the gateways behind it, a media plane for the camera, teleoperation, and benchmarks. It lives at `dimos/web/dimoscope/`.

The thesis: **"DimOS topics in the browser", not "Zenoh in the browser."** A small `Transport` interface + the self-describing 8-byte `@dimos/msgs` codec means the *same* app and SDK code runs over LCM or Zenoh, and the gateway stays a thin byte-relay.

## Docs

- **[findings.md](./findings.md)** — the research summary: transports & the browser story, the `@dimos/topics` SDK + three gateways, the media plane, multi-robot/multi-camera, the #2502 web-API direction, QoS, forks, side-fixes, and the open questions for the team.
- **[benchmarks.md](./benchmarks.md)** — throughput / latency / bandwidth across all three transports (headless + in-browser), with takeaways. Curated from the raw runs in [`../bench/RESULTS.md`](../bench/RESULTS.md) and [`../bench/RESULTS-browser.md`](../bench/RESULTS-browser.md).
- **[custom-messages-in-the-browser.md](./custom-messages-in-the-browser.md)** — a zero-rebuild design proposal: how a user's *custom* message type can render in the browser without regenerating/republishing a codec package.
- **[data-path.md](./data-path.md)** — the data-path benchmark: every delivery mechanism (WebSocket / SSE / HTTP-poll / WebRTC) × network condition × decode location, with one-command CLI runners (`deno task bench:matrix` / `bench:decode` / `bench:webrtc`) and the findings.

## The three transports (all behind one SDK)

| Transport | Server | Port | Notes |
|---|---|--:|---|
| Deno↔LCM | `servers/gateway.ts` | 8089 | reads LCM UDP multicast, relays raw packets |
| Python↔Zenoh | `servers/gateway_zenoh.py` | 8088 | subscribes Zenoh `**`, re-wraps as LC02 |
| zenoh-ts (direct) | `packages/topics/src/adapters/zenohTs.ts` | 10000 | browser ↔ `zenoh-bridge-remote-api`, no gateway in the read path |

Plus a standalone **media node** (`servers/media_server.py`, :8092) for the camera — its own bus peer, encode-once-per-topic, fan-out.

## Run it

```bash
# 1. a data source on zenoh (dimsim go2, or a real go2)
DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run unitree-go2

# 2. all gateways + media node + zenoh-ts bridge at once
bash dimos/web/dimoscope/servers/start-all.sh
#    → Python↔Zenoh :8088 · media :8092 · Deno↔LCM :8089 · zenoh-ts :10000

# 3. the app  →  http://localhost:5173
deno task --cwd dimos/web/dimoscope app
```

(zenoh-ts needs `cargo install zenoh-bridge-remote-api`; pick the transport live in the topbar dropdown.)

See also the repo-level [`../README.md`](../README.md) and [`../WALKTHROUGH.md`](../WALKTHROUGH.md).
