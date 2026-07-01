# dimoscope transport benchmark — methodology, coverage, and results

How we measure every way of getting a DimOS stream from the robot to a **browser**, and which
transport to use for what. One backend (the gateway) is the single binary under test; only the
*delivery mechanism* and the *network* change. Everything here is measured in the **real browser
runtime** — the actual deliverable — via the in-browser bench.

## What's under test

The five same-origin delivery mechanisms on the gateway, plus two orthogonal axes:

| Mechanism | Wire | Path | Key property |
|---|---|---|---|
| WebSocket | TCP | `/ws` | reliable, ordered, duplex (default; carries teleop/goal/rpc) |
| SSE | TCP/HTTP | `/sse` | reliable, ordered, server→client |
| HTTP poll | TCP/HTTP | `/poll` | reliable; universal req/resp baseline |
| WebRTC data | UDP (SCTP/DTLS) | `/rtc` | configurable unordered/unreliable → **no TCP head-of-line blocking** |
| WebTransport | QUIC (HTTP/3) | UDP `:8443` | streams + datagrams → **no HoL**; cert via `/cert` hash |

- **Decode-location axis:** client-binary (the self-describing 8-byte-hash relay) vs server→JSON
  (the rosbridge model).
- **QoS axis:** rate-limit · conflation (freshest-wins) · priority lanes. See
  [qos-demo.md](./qos-demo.md).

## Methodology — measure in the real browser

Two harnesses, both reusing the SDK's measurement core (`@dimos/web/bench`), so they measure
identically:

- **`/bench.html`** — the full-page sweep. Opens a client per mechanism and runs the same scenarios
  across all five, so **WebRTC and WebTransport are exercised on the real browser stacks** (ICE/DTLS,
  QUIC, the `serverCertificateHashes` cert flow) — not a stand-in. Latency is end-to-end
  (publish → browser). Unsupported mechanisms (e.g. WebTransport on Safari/Firefox) record a NaN row,
  which doubles as the cross-browser support matrix.
- **In-app Bench tab** — measures the **live active transport** (whatever the topbar dropdown picked)
  while you vary client-side QoS knobs (maxHz, server-vs-client rate-limit, conflation) and an optional
  server-JSON decode A/B, with live sparklines and copy-as-Markdown export.

**Load:** the synthetic `scenarios/bench.py` (`deno task scope:bench`) — a deterministic dimos module
that publishes `/bench/*` (configurable pose / grid / large-Image streams), each stamped with `ts` for
one-way latency and a per-topic `seq` for exact loss. A real sim (`deno task sim` mujoco/dimsim) works
too as a realism cross-check.

**Network:** for impairment, point the bench at a remote gateway with `?gw=host:port` (or `GATEWAY_URL`)
and let the **real internet path** supply latency/loss — see [real-wan.md](./real-wan.md). This is
ground truth: real RTT, real reorder/drop, no simulator to trust.

## What only the browser can measure

- The **real** WebRTC + WebTransport browser stacks — ICE/DTLS, QUIC, and the `serverCertificateHashes`
  cert path.
- **WebCodecs** decode + canvas render (the camera path).
- **Cross-browser** support + behaviour: **Safari/WebKit has no WebTransport**; Firefox lags; WebRTC has
  per-engine quirks. (`/bench.html` records unsupported mechanisms as a NaN row = the support matrix.)
- Real `getStats` / `chrome://webrtc-internals`, DevTools throttling, glass-to-glass UX.

---

## Results

### Cross-browser support matrix

`/bench.html` records unsupported mechanisms as NaN rows, so the support matrix falls out of a run:

| mechanism | Chromium/Edge | Firefox | Safari/WebKit |
|---|:-:|:-:|:-:|
| WebSocket | ✅ | ✅ | ✅ |
| SSE | ✅ | ✅ | ✅ |
| HTTP poll | ✅ | ✅ | ✅ |
| WebRTC data | ✅ | ✅ | ✅ (quirks) |
| **WebTransport** | ✅ | ❌ (in progress) | ❌ |

→ **the SDK falls back to WS** for WebTransport on Firefox/Safari — already handled (the dropdown won't
offer it / it errors into a NaN row). WS/SSE/poll/WebRTC are universal.

### Real WAN — real Chrome, all 5 mechanisms, Mac → VPS by IP

Over the public internet (`http://localhost:5173/?gw=<vps>:8080`), 4× PoseStamped + grid:

| transport | 4×Pose hz | grid hz | loss% |
|---|--:|--:|--:|
| WebSocket | 385 | 23.5 | ~0 |
| SSE | 335 | 18.5 | 0 |
| HTTP poll | 412 | 22.5 | 0 |
| WebRTC data | 352 | 19.25 | 0 |
| WebTransport | 356 | 19.25 | 0 |

→ **all 5 deliver ~335–412 Hz browser→VPS over the public internet**, loss ~0. Getting there took: server
**CORS** on `/cert` (cross-origin cert fetch for WT); a **`/rtc` 403 fix** (the WebRTC WS handler param
was untyped → FastAPI rejected the handshake); a **WebRTC client-adapter fix** (resolve on `dc.onopen`,
not on offer-sent); and opening `8080/tcp` + `8443/udp` + the aiortc ICE UDP range on the VPS (the
public-IP host candidate needs no TURN). _(Latency is omitted — Mac/VPS clocks unsynced.)_

### WebTransport verified in real Chrome

`/bench.html` on WebTransport connects via `serverCertificateHashes` over the short-lived self-signed
cert and pulls both **datagrams** (small pose/imu, unreliable/unordered) and per-frame **unidirectional
streams** (large lidar, reliable) — ~917 datagrams + 81 streams in a 3 s window on a clean link.

### The findings the Bench tab demonstrates

- **Head-of-line blocking is the whole story under loss.** A dropped datagram on WebRTC/WebTransport
  simply never arrives — no retransmit, no HoL — so delivered-frame latency stays flat while throughput
  falls ~linearly with the drop rate. A reliable ordered TCP stream (WebSocket) instead converts loss
  into retransmit → latency/stall. This is the teleop-under-bad-network case for the UDP transports; it
  shows up on a real lossy `?gw` path.
- **Decode stays on the client.** Flipping the server-JSON A/B in the Bench tab: for the small
  PoseStamped + grid load, server→JSON moves **~2.64× the bytes** of the binary self-describing relay
  (and it balloons further on packed arrays like lidar/pointcloud, plus the gateway pays the
  deserialize). Keep the gateway a byte-relay.
- **On-demand cuts bandwidth.** Subscribing 1 of 4 topics vs all 4 is a **~75% reduction** — a topic is
  on the wire only while it has a live subscriber.
- **QoS keeps pose alive.** Rate-limiting the heavy lidar (`setRateLimit(2)`) frees the link so pose/
  teleop stay responsive on a saturated path; priority lanes drain high-priority topics first and
  conflate the bulk lane. Full story + A/B in [qos-demo.md](./qos-demo.md).
- **MB/s streams saturate cellular.** A raw ~18 MB/s dense-lidar relay is impossible on 4G/3G on any
  mechanism — heavy streams need a fat pipe, compression/decimation, or the media plane (encode at
  source), not a raw topic relay.

---

## Recommendations

- **Default to WebSocket** for the data + control plane: lowest overhead, duplex, the trust boundary.
- **Heavy streams (MB/s) under loss/cellular → WebRTC-data or WebTransport** (UDP, no HoL): WS stalls
  with TCP head-of-line blocking where these stay smooth. WebTransport additionally has clean QUIC
  streams+datagrams and connection migration.
- **SSE / HTTP-poll = deliberate fallbacks** (one-way light telemetry / universal baseline) — not for
  heavy streams.
- **Decode stays on the client** (the byte-relay): server→JSON costs ~2.64× bytes + gateway CPU.
- **QoS** (rate-limit heavy streams, priority lanes) keeps pose/teleop responsive on a saturated link.
- **Browser caveat:** WebTransport is Chrome/Edge only → the SDK must fall back (WS) on Safari/Firefox.

## Reproduce

```bash
deno task serve         # the one service on :8080
deno task scope:bench   # data source → /bench/* (scenarios/bench.py)
deno task app           # then open the bench:
#   http://localhost:5173/bench.html            → full 5-mechanism sweep
#   the in-app Bench tab                         → live transport + QoS knobs
#   ?gw=<vps>:8080  → route through a real remote path (see real-wan.md)
#   ?dur=ms         → per-scenario duration      ·   ?wt=8443 → WebTransport port
```

See [benchmarks.md](./benchmarks.md) for the detailed per-stream numbers and [qos-demo.md](./qos-demo.md)
for the QoS A/B.
