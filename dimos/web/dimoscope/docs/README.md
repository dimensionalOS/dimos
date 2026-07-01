# dimoscope docs

**dimoscope** is a research effort to build a solid JavaScript SDK/framework for integrating with the DimOS stack from the browser: a transport-agnostic topics SDK (`@dimos/topics`), the gateways behind it, a media plane for the camera, teleoperation, and benchmarks. It lives at `dimos/web/dimoscope/`.

The thesis: **"DimOS topics in the browser", not "Zenoh in the browser."** A small `Transport` interface + the self-describing 8-byte `@dimos/msgs` codec means the *same* app and SDK code runs over LCM or Zenoh, and the gateway stays a thin byte-relay.

## Docs

- **[benchmark-report.md](./benchmark-report.md)** — the transport benchmark: the five delivery mechanisms measured in the real browser (`/bench.html` + the in-app Bench tab), the cross-browser support matrix, real-WAN (Mac → VPS) results, and which transport to use for what.
- **[benchmarks.md](./benchmarks.md)** — the detailed numbers behind the report: throughput / latency / bandwidth per mechanism × stream, decode location, QoS, WebTransport under loss — with takeaways.
- **[qos-demo.md](./qos-demo.md)** — the QoS story: client-declares → gateway-enforces → transport-reinforces, the priority-outbox design, and the A/B that keeps pose/teleop crisp under a saturated link.
- **[real-wan.md](./real-wan.md)** — runbook for measuring the real internet path (MacBook browser ↔ VPS gateway) via `?gw=host:port`, all five transports over a raw IP.

## The five delivery mechanisms (all on one service, behind one SDK)

One Python process (the gateway, http://localhost:8080) taps **both LCM and Zenoh** and fans the same
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
# 1. a data source (dimsim go2, a real go2, or the synthetic bench source `deno task scope:bench`)
DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run unitree-go2

# 2. the whole backend — one process, all transports + the app
deno task --cwd dimos/web/dimoscope serve            # = uv run python -m gateway → http://localhost:8080

# 3. the app (dev, hot reload)  →  http://localhost:5173   (or just open http://localhost:8080/)
deno task --cwd dimos/web/dimoscope app
```

See also the repo-level [`../README.md`](../README.md) and [`../WALKTHROUGH.md`](../WALKTHROUGH.md).
