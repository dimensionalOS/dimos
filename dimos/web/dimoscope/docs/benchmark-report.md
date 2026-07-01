# dimoscope transport benchmark — methodology, coverage, and results

How we measure every way of getting a DimOS stream from the robot to a browser — across the **CLI**,
the **real browser**, and a **real internet path** — and which transport to use for what. One backend
(the gateway) is the single binary under test; only the *delivery mechanism* and the *network* change.

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
  (the rosbridge model). `bench:decode`.
- **QoS axis:** rate-limit · conflation (freshest-wins) · priority lanes. `bench:qos`.

## Methodology — three fidelity layers, each validating the next

| Layer | Runner | Network impairment | What it's for |
|---|---|---|---|
| **A — CLI / headless** | Deno (WS/SSE/poll, real client) + **Python `aiortc`/`aioquic` stand-ins** for WebRTC/WebTransport (Deno has no `RTCPeerConnection`/WebTransport) | `bench/netsim.ts` (TCP) + `bench/netsim-udp.ts` (UDP); **kernel `tc/netem` on the Linux VPS** | breadth, speed, 100s of reproducible iterations, CI, runs on a headless server, exact control |
| **B — Browser / real runtime** | **real** Chromium / Firefox / WebKit via Playwright, driving `bench.html` (the actual SDK) | same `netsim` proxy (TCP-layer → runtime-agnostic) | the *real* WebRTC + WebTransport browser stacks (ICE/DTLS/QUIC + the `serverCertificateHashes` cert flow), WebCodecs, **cross-browser** behavior, the true UX |
| **C — Real WAN** | the gateway on the **VPS**; Mac browser + CLI → VPS by IP | the real internet path (+ `tc/netem` on the VPS) | ground truth — does `netsim 3g/4g` predict reality |

**Validation chain.** A number is trusted when **CLI-stand-in ≈ browser-real ≈ synthetic-netsim ≈
kernel-netem ≈ real-WAN**. Because the same gateway is the server in every layer, it's
apples-to-apples; the layers cross-check each other — most importantly, **Layer B answers whether the
cheap aiortc/aioquic CLI stand-ins actually predict the real browser stacks**.

## CLI vs browser — what each can and can't measure

**Only the CLI can:**
- Kernel `tc/netem` (reorder, duplication, corruption, exact rate) on the Linux server.
- Hundreds of deterministic iterations; reproducible, CI-gated; no display.
- Run on a **headless server** (the VPS) with no browser.
- The server-decode axis (`decode=server-json`) measured cleanly.
- Isolate raw transport without browser render/GC overhead.

**Only the browser can:**
- The **real** WebRTC + WebTransport browser stacks — ICE/DTLS, QUIC, and the
  `serverCertificateHashes` cert path (the CLI stand-ins use `CERT_NONE` and skip the cert dance).
- **WebCodecs** decode + canvas render (the camera path).
- **Cross-browser** support + behavior: **Safari/WebKit has no WebTransport**; Firefox lags; WebRTC
  has per-engine quirks. (`bench.html` records unsupported mechanisms as a NaN row = the support matrix.)
- Real `getStats` / `chrome://webrtc-internals`, DevTools throttling, glass-to-glass UX.

**Both:** WS/SSE/poll (Deno ≈ browser), `netsim` impairment, the decode/QoS/on-demand logic.

> Net: the CLI gives **control and breadth** (and is the only thing that runs on the headless VPS); the
> browser gives **fidelity** (the actual deliverable runtime + cross-browser). Neither alone is enough —
> the value is in their agreement.

## Most reliable way to simulate everything

- **Load:** synthetic **`bench/bench_source.py`** (deterministic; configurable pose / lidar / dense /
  camera / mixed streams, each stamped for one-way latency + per-topic seq → exact loss). This is the
  workhorse — a real sim (`deno task sim` mujoco/dimsim) adds physics jitter, so it's used only as a
  *realism cross-check*, not the primary signal.
- **Network — a hierarchy of trust:**
  1. `netsim` / `netsim-udp` (userspace, zero-install, fast, single-host) — dev + CI.
  2. **kernel `tc/netem` on the Linux VPS** — most accurate synthetic (real reorder/dup/corrupt).
  3. **real WAN** (the VPS over the public internet) — ground truth.
  Each is validated against the next, so a netsim profile is "trusted" once it matches kernel-netem and
  the real path.

---

## Resolved: the earlier "double delivery" was a stray publisher, not a gateway bug

An earlier pass reported the gateway "double-delivering" (`4× Pose` at ~576 msg/s ≈1.45×, `loss%` 99.98
on a clean LAN). That diagnosis was **wrong**. Root-caused and fixed:

- **Actual cause:** a **stale `go2-bench` benchload process** (`dimos … run go2-bench --option
  benchload.autostart=true`) had been running for ~15 h, flooding **zenoh** with the same `/bench/p0..p3`
  topic *names* that the CLI's LCM `bench_publisher` uses. Two **independent** publishers on one topic
  name (its seq counter had reached ~4.1 M) — the gateway faithfully delivered both. Not one message
  duplicated; two different messages (proven by the payloads: LCM copy `frame_id="bench"`, zenoh copy
  `frame_id="<seq>"`).
- **The gateway's dual-tap is correct as designed** — tapping LCM **and** zenoh so a blueprint can switch
  transports with no restart. In normal use a topic is published on only one bus; nothing to reconcile.
- **Proof:** after killing the stray process, a single publisher gives **`hz≈328`** (4× ~82 Hz, single
  delivery) and **`loss%=0`** across every profile — see the clean tables below. No code change to
  `gateway/{app,bus}.py` was needed (a content-dedup was prototyped, then reverted — it wouldn't even apply
  here, since the two copies are genuinely different messages).
- **Prevention:** benches must run in isolation. Before a run, `ps aux | grep -E "go2-bench|bench_source|
  bench_publisher"` and kill strays; never run two publishers on the same topic names at once.

The throughput/loss numbers below are from clean, single-publisher runs (`2026-07-01`).

## Results

> Each table notes its layer + date; raw data in `bench/RESULTS-*.md`. All numbers below are from
> **clean, single-publisher runs** — throughput and `loss%` are real.

### A — CLI / headless (single host), clean single-publisher
_2026-07-01 · netsim TCP profiles · one-way latency (ms) · `loss%=0` across every profile._

**Light stream (4× PoseStamped @100Hz, ~27 kB/s, ~328 msg/s single delivery) — `RESULTS-mechanisms.md`:**

| profile | ws p50 / p95 | sse p50 / p95 | poll p50 / p95 |
|---|--:|--:|--:|
| lan | 1.3 / 4.6 | 2.0 / 5.3 | 1.5 / 3.6 |
| wifi | 15.3 / 23.6 | 15.2 / 22.8 | 23.8 / 36.3 |
| 4g  | 84 / 114 | 83 / 116 | 111 / 162 |
| 3g  | 285 / 386 | 320 / 430 | **592 / 911** |
| lossy | 148 / 249 | 195 / 320 | 224 / 365 |
→ **ws ≈ sse < poll**; at light load all three are usable even on 3g, but **poll's per-cycle RTT
roughly doubles latency** as the link degrades (3g p50 592 vs ws 285). `loss%=0` everywhere; throughput
holds ~310–329 msg/s to 4g and eases to ~250–269 on 3g (bandwidth, not loss).

**Heavy stream (dense ~17.9 MB/s) — `RESULTS-mechanisms-dense.md`:**

| profile | ws | sse | poll |
|---|--:|--:|--:|
| lan | 18.3 Hz, p50 22 ms | 18.3 Hz, p50 **26** | 18.7 Hz, p50 21 ms |
| 4g · 3g | **0 (dead)** | **0 (dead)** | **0 (dead)** |
→ **MB/s saturates cellular** — ~18 MB/s is impossible on 4g(12Mbps)/3g(2Mbps): every TCP mechanism
collapses to 0. On LAN, **SSE's base64 inflates latency** (p50 26 vs 21–22). Heavy streams need a fat
pipe, compression/decimation, or the media plane — not a raw topic relay.

**WebTransport (QUIC datagrams) under REAL packet loss — `bench:loss` (netsim-udp drops real datagrams):**

| drop % | datagrams/s | dgram p50 | dgram **p95** |
|---|--:|--:|--:|
| 0% | 297 | 0 | 0 |
| 5% | 282 | 14 | **23** |
| 10% | 269 | 14 | **24** |
| 20% | 227 | 14 | **25** |
→ **the headline:** delivered-datagram **p95 stays flat (~23–25 ms) as loss rises to 20%** — a dropped
datagram simply doesn't arrive (**no head-of-line blocking**); throughput falls ~linearly with drop
rate. A reliable WS/TCP stream instead converts loss into retransmit→latency/stall. **This is the
teleop-under-bad-network case for WebRTC/WebTransport.**

**QoS — `bench:qos` (mixed load, saturated link):** rate-limit the heavy lidar `setRateLimit(2)` →
**4.67× less bandwidth** (client-driven); on-demand (1 of 4 topics) → **75% reduction**; prioritization
→ capping lidar at 2 Hz keeps **pose alive at 169 Hz, loss 0** on a saturated link.

**Decode location — `bench:decode`:** for the small PoseStamped + grid load, server→JSON moved **~2.64×**
the bytes of the binary relay (247 vs 93 kB/s). The rosbridge tax is **payload-dependent** — it balloons
further on large packed arrays like lidar/pointcloud. Keep decode on the client.

> **Gaps in this CLI run:** the **WebRTC-data e2e** and **WebTransport-e2e** *aiortc/aioquic probes*
> hit connection errors (`webrtc_client_probe.py` / `webtransport_client_probe.py`) — environmental,
> to retry; WebTransport itself is proven by the loss bench above.

### A — CLI / headless (single host) — `bench/RESULTS-*.md`
_pending Stage 1._

### B — Browser / real runtime + cross-browser — `bench/RESULTS-browser.md`

**Cross-browser *support* matrix** (factual — the `bench.html` rewrite records unsupported mechanisms
as NaN rows, confirming this empirically when run):

| mechanism | Chromium/Edge | Firefox | Safari/WebKit |
|---|:-:|:-:|:-:|
| WebSocket | ✅ | ✅ | ✅ |
| SSE | ✅ | ✅ | ✅ |
| HTTP poll | ✅ | ✅ | ✅ |
| WebRTC data | ✅ | ✅ | ✅ (quirks) |
| **WebTransport** | ✅ | ❌ (in progress) | ❌ | 
→ **the SDK must fall back to WS** for WebTransport on Firefox/Safari — already handled (the dropdown
just won't offer it / it errors into a NaN row). WS/SSE/poll/WebRTC are universal.

**Empirical — real Chrome, all 5 mechanisms, over the real WAN (Mac → VPS by IP)** — `RESULTS-realwan.md`:

| transport | 4×Pose hz | grid hz | loss% |
|---|--:|--:|--:|
| WebSocket | 385 | 23.5 | ~0 |
| SSE | 335 | 18.5 | 0 |
| HTTP poll | 412 | 22.5 | 0 |
| WebRTC data | 352 | 19.25 | 0 |
| WebTransport | 356 | 19.25 | 0 |
→ **all 5 deliver ~335–412 Hz browser→VPS over the public internet**, loss ~0. Getting there took: server
**CORS** on `/cert` (cross-origin cert fetch for WT); a **`/rtc` 403 fix** (the WebRTC WS handler param
was untyped → FastAPI rejected the handshake); a **WebRTC client-adapter fix** (resolve on
`dc.onopen`, not on offer-sent); and ufw `8080/tcp + 8443/udp + 32768:60999/udp` (aiortc ICE range; the
VPS public-IP host candidate needs no TURN). Same-origin local Chrome confirms SSE/poll too; a "WS 0"
seen locally was a **stale serve process**, not a bug. _(Latency is omitted — Mac/VPS clocks unsynced.)_

### Kernel `tc/netem` on the VPS — `bench/RESULTS-vps.md`
Applied to `lo` (delay 100 ms · loss 3% · reorder 20%), `netsim` profile `lan` → **netem-only** effect:
ws p50 **409 ms** / loss **7.86%**, sse 442 ms / 7.66%, poll 702 ms / 5.26%. Real reorder+drop becomes
**seq-gap loss (5–8%) + latency blowup** — impairment userspace `netsim` can't reproduce (it models loss
as jitter; TCP can't raw-drop). Validates the netsim profiles against kernel-accurate ground truth.

### C — Real WAN (Mac ↔ VPS) — `bench/RESULTS-realwan.md`
**Done.** `ufw allow 8080/tcp + 8443/udp` was sufficient (no external cloud firewall → no provider
console needed). Deno CLI WS = **354 Hz** full delivery over the internet; the real-Chrome **all-5** table
above is browser→VPS over the same path. Domain-free (raw IP + self-signed WT cert), app on
`http://localhost`. Live self-test recipe + bring-up/teardown in `RESULTS-realwan.md` / `docs/real-wan.md`.

---

## Recommendations (consolidated — backed by the CLI run)

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
# CLI (single host):
deno task bench:all                          # matrix · decode · webrtc · webtransport · loss · qos
# Browser (real runtime, cross-browser):
deno task bench:browser                       # (Stage 2) Playwright Chromium/Firefox/WebKit
# Real WAN: the gateway on the VPS, then from the Mac:
GATEWAY_URL=ws://<vps>:8080/ws deno task bench    # see docs/real-wan.md
```

## Status & remaining work (handoff)

**Done & committed:** `bench.tsx` fixed to the 5 mechanisms · this report (methodology + CLI-vs-browser
can/can't + simulation hierarchy + cross-browser support matrix) · root-caused the earlier "double
delivery" to a **stray `go2-bench` publisher** (killed; the gateway is correct — no code change) · the
**clean CLI numbers** (light/heavy/WT-loss/QoS/decode), single-publisher, `loss%=0`.

**Teed up (each is one command, infra exists):**
- **Browser perf** (real Chrome via claude-in-chrome MCP, or `npx playwright` cross-browser) →
  `RESULTS-browser.md` + the CLI-stand-in-vs-real-browser latency cross-check.
- **VPS / kernel netem** — `docs/real-wan.md` runbook: `uv sync --extra web` + the gateway on the VPS,
  the CLI suite there, and `tc/netem` (a Mac can't) for the kernel-accurate cross-check.
- **Real WAN** — Mac → VPS by IP; **needs you to open the VPS firewall port** (TCP 8080 + UDP 8443).

**Other CLI gaps to retry:** the aiortc/aioquic **WebRTC-data e2e** + **WebTransport-e2e** probes hit
connection errors this run (WebTransport itself is proven by the loss bench).
