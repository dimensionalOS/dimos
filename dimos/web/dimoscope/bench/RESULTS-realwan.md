# dimoscope — real-WAN: ALL 5 transports, browser → VPS over the public internet

_2026-07-01 · real Chrome → VPS `37.60.232.68`, domain-free (raw IP; self-signed WT cert only)._

## ⭐ Result: all 5 transports work browser → VPS over the real internet

Measured in **real Chrome** (page on `http://localhost` for a secure context; transports at the VPS by
raw IP). 4000 ms/scenario.

| transport | 4×Pose throughput | grid (large) | loss% | notes |
|---|--:|--:|--:|---|
| **WebSocket** | 385 Hz | 23.5 Hz | ~0 | reliable, duplex, default |
| **SSE** | 335 Hz | 18.5 Hz | 0 | one-way, reliable |
| **HTTP poll** | 412 Hz | 22.5 Hz | 0 | universal fallback |
| **WebRTC data** | 352 Hz | 19.25 Hz | 0 | UDP, unordered/lossy → no HoL |
| **WebTransport** | 356 Hz | 19.25 Hz | 0 | QUIC datagrams → no HoL |

_(One-way latency isn't meaningful here — Mac and VPS clocks aren't synced. Throughput + loss are valid.
For real RTT-vs-loss behavior see the netsim/netem tables in `RESULTS-vps.md`.)_

## What it took (the fixes that made all 5 work over the WAN)

1. **`serve.py` CORS** (`CORSMiddleware`, `allow_origins=*`): the SDK runs on `http://localhost` and the
   gateway is on the VPS IP → cross-origin. Without CORS the browser couldn't `fetch('/cert')` → no
   WebTransport. Now `/cert` + `/health` are cross-origin-readable.
2. **WebRTC `/rtc` 403 fix** (`servers/bench.py`): `WebRtcDataPlane.handle`'s `ws` param was **untyped**,
   so FastAPI treated it as a query-param dependency and rejected every `/rtc` handshake with 403 (WebRTC
   never connected — even the CLI aiortc probe). Annotating `ws: WebSocket` fixed it.
3. **WebRTC client adapter** (`packages/topics/src/adapters/webRtcData.ts`): `connect()` resolved right
   after sending the SDP offer, *before* the DataChannel opened → the caller measured a dead channel. Now
   it resolves on `dc.onopen` (with a 15 s WAN timeout).
4. **ufw**: opened `8080/tcp` (WS/SSE/poll/signaling/cert) + `8443/udp` (WebTransport QUIC) +
   `32768:60999/udp` (aiortc's ephemeral ICE range — the VPS `eth0` is the public IP, so its host
   candidate is directly reachable, no TURN needed). No external cloud firewall → on-host ufw sufficed.
5. **WebSocket "0" locally was a stale local serve**, not a bug — WS works fine against the fresh VPS.

## 🔧 Self-test recipe (drive all 5 yourself)

The VPS gateway + a synthetic source are **live** (tmux `dimos-realwan-65329`) and a local vite is
serving the app. In your browser:

- **Benchmark page (all 5, numbers):** `http://localhost:5175/bench.html?gw=37.60.232.68:8080` → click
  **▶ Run benchmark** → every transport row is non-zero.
- **The app (pick a transport live):** `http://localhost:5175/?gw=37.60.232.68:8080` → use the topbar
  **transport dropdown** (WebSocket · SSE · HTTP poll · WebRTC data · WebTransport) → topics stream on
  each.

**Keep the page on `http://localhost`** (secure context + no mixed-content) — do *not* open the app at
`http://37.60.232.68:8080` directly (raw-IP http isn't a secure context → the browser disables
WebTransport). No domain / no app-TLS needed.

**If something's down when you return, bring it back up:**
```bash
# local (Mac): serve the app
cd dimos/web/dimoscope && deno task app         # vite on :5173/:5175
# VPS: gateway + source (if the tmux session died)
ssh -i ~/.ssh/vps-coolify kristjan@37.60.232.68
cd ~/dimos-bench/dimos/web/dimoscope
tmux new-session -d -s dimos-realwan-65329 "PORT=8080 WT_PORT=8443 uv run python serve.py"
DIMOS_TRANSPORT=lcm BENCH_HZ=100 PYTHONPATH=bench nohup uv run python bench/bench_source.py &
```

Teardown (when fully done): see `docs/real-wan.md` → "Currently deployed + TEARDOWN".

## VPS / kernel-netem / CLI numbers
See `RESULTS-vps.md` (matrix × profiles, kernel `tc/netem` reorder/loss, WebTransport datagram-loss = no
HoL, QoS) and `RESULTS-mechanisms*.md`.
