# dimoscope docs

**dimoscope** is a research effort to build a solid JavaScript SDK/framework for integrating with the DimOS stack from the browser: a transport-agnostic topics SDK (`@dimos/topics`), the gateways behind it, a media plane for the camera, teleoperation, and benchmarks. It lives at `dimos/web/dimoscope/`.

The thesis: **"DimOS topics in the browser", not "Zenoh in the browser."** A small `Transport` interface + the self-describing 8-byte `@dimos/msgs` codec means the *same* app and SDK code runs over LCM or Zenoh, and the gateway stays a thin byte-relay.

## Docs

- **[findings.md](./findings.md)** — the research summary: transports & the browser story, the `@dimos/topics` SDK + three gateways, the media plane, multi-robot/multi-camera, the #2502 web-API direction, QoS, forks, side-fixes, and the open questions for the team.
- **[benchmarks.md](./benchmarks.md)** — throughput / latency / bandwidth across all three transports (headless + in-browser), with takeaways. Curated from the raw runs in [`../bench/RESULTS.md`](../bench/RESULTS.md) and [`../bench/RESULTS-browser.md`](../bench/RESULTS-browser.md).
- **[custom-messages-in-the-browser.md](./custom-messages-in-the-browser.md)** — a zero-rebuild design proposal: how a user's *custom* message type can render in the browser without regenerating/republishing a codec package.
- **[data-path.md](./data-path.md)** — the data-path benchmark: every delivery mechanism (WebSocket / SSE / HTTP-poll / WebRTC) × network condition × decode location, with one-command CLI runners (`deno task bench:matrix` / `bench:decode` / `bench:webrtc`) and the findings.

## The five delivery mechanisms (all on one service, behind one SDK)

One Python process (`serve.py`, http://localhost:8080) taps **both LCM and Zenoh** and fans the same
self-describing frames out over every mechanism — pick one live in the topbar dropdown:

| Mechanism | Wire | Path | Notes |
|---|---|---|---|
| WebSocket | TCP | `/ws` | duplex, reliable — default; also carries teleop/goal/rpc |
| SSE | TCP/HTTP | `/sse` | server→client only (binary→base64) |
| HTTP poll | TCP/HTTP | `/poll` | universal req/resp baseline |
| WebRTC data | UDP | `/rtc` | unordered/unreliable → no TCP head-of-line blocking |
| WebTransport | QUIC | UDP `:8443` | streams + datagrams, no HoL; cert hash from `/cert` (Chrome) |

The camera rides the same service on `/media` (webrtc / webcodecs / jpeg, encode-once → fan-out).

## Run it

```bash
# 1. a data source (dimsim go2, a real go2, or bench_publisher.py)
DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run unitree-go2

# 2. the whole backend — one process, all transports + the app
deno task --cwd dimos/web/dimoscope serve            # = uv run python serve.py → http://localhost:8080

# 3. the app (dev, hot reload)  →  http://localhost:5173   (or just open http://localhost:8080/)
deno task --cwd dimos/web/dimoscope app
```

See also the repo-level [`../README.md`](../README.md) and [`../WALKTHROUGH.md`](../WALKTHROUGH.md).
