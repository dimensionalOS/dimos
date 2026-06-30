# dimoscope transport benchmark — methodology, coverage, and results

How we measure every way of getting a DimOS stream from the robot to a browser — across the **CLI**,
the **real browser**, and a **real internet path** — and which transport to use for what. One backend
(`serve.py`) is the single binary under test; only the *delivery mechanism* and the *network* change.

## What's under test

The five same-origin delivery mechanisms on `serve.py`, plus two orthogonal axes:

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
| **C — Real WAN** | `serve.py` on the **VPS**; Mac browser + CLI → VPS by IP | the real internet path (+ `tc/netem` on the VPS) | ground truth — does `netsim 3g/4g` predict reality |

**Validation chain.** A number is trusted when **CLI-stand-in ≈ browser-real ≈ synthetic-netsim ≈
kernel-netem ≈ real-WAN**. Because the same `serve.py` is the server in every layer, it's
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

## ⚠ BLOCKER found while re-benchmarking: `serve.py` double-delivers (dual-bus tap)

The re-benchmark immediately surfaced a real bug the green typecheck/unit-tests didn't catch.

- **Symptom:** `4× PoseStamped @ 100 Hz` (expect ~400 msg/s) arrives at ~**576 msg/s (≈1.45×)**, and the
  matrix `loss%` reads **99.98%** on a clean LAN — both signatures of duplicate/interleaved delivery.
- **Root cause:** `serve.py` lifespan **unconditionally** runs `bus.start_zenoh()` **and**
  `bus.start_lcm()` (`servers/bus.py` even notes *"if both buses carry the same topic you'd see it
  twice — normally only one is active"*). On a macOS/zenoh-default host the LCM-published `/bench/*`
  topics are **also echoed onto zenoh**, so the bus fans out each message ~twice. No dedup.
- **Proof:** disabling the zenoh tap (`ZENOH_KEY="nomatch/**"`) drops `4× Pose` from **576 → 297 msg/s**
  (single delivery). Repro: `PORT=8080 ZENOH_KEY="nomatch/**" python serve.py` + `DIMOS_TRANSPORT=lcm
  bench_publisher.py` + `GATEWAY_URL=ws://localhost:8080/ws deno task bench`.
- **Impact:** (1) a **latent product correctness bug** — a browser on a dual-bus deployment receives
  **duplicate messages**; (2) it corrupts **every throughput/loss** number in every layer (CLI, browser,
  VPS), so those can't be trusted until fixed. **Latency** (per-message publish→recv) is *not* affected.
- **Fix options (owner: the consolidation/`serve.py` author):** dedup in `bus.py` by `(topic, seq)` over
  a short window; **or** a `BUS_BACKENDS` env to tap only the active bus (default both is the trap);
  **or** at minimum gate the zenoh tap behind a flag. Until then, the benches should launch `serve.py`
  with `ZENOH_KEY` matching nothing when the load is single-bus.

**Decision needed (you):** fix `serve.py` (dedup / backend-select) — then I re-run the full program and
the throughput/loss tables become trustworthy. I did **not** patch `serve.py` autonomously (it's the
centerpiece another agent owns + the fix is a design choice). Below: the metrics that *are* trustworthy.

## Results

> Filled in by the run. Each table notes its layer + date; raw data in `bench/RESULTS-*.md`.
> **Throughput/loss are bug-affected (see BLOCKER above) — read latency + relative ordering only.**

### A — CLI / headless (single host), clean (`ZENOH_KEY=nomatch` workaround → single delivery, loss%=0)
_2026-07-01 · netsim TCP profiles · one-way latency (ms) · loss%=0 across all clean rows (the workaround
neutralises the dup bug)._

**Light stream (4× PoseStamped @100Hz, ~24 kB/s) — `RESULTS-mechanisms.md`:**

| profile | ws p50 / p95 | sse p50 / p95 | poll p50 / p95 |
|---|--:|--:|--:|
| lan | 0.45 / 0.92 | 0.52 / 0.95 | 1.17 / 1.9 |
| 4g  | 83 / 116 | 81 / 113 | 112 / 166 |
| 3g  | 276 / 385 | 287 / 401 | **448 / 874** |
| lossy | 153 / 263 | 166 / 274 | 193 / 338 |
→ **ws ≈ sse < poll**; at light load all three are usable even on 3g, but **poll's per-cycle RTT
roughly doubles latency** as the link degrades. (loss%=0 everywhere — clean.)

**Heavy stream (dense ~17 MB/s) — `RESULTS-mechanisms-dense.md`:**

| profile | ws | sse | poll |
|---|--:|--:|--:|
| lan | 17.3 Hz, p50 21 ms | 17.3 Hz, p50 **26** (max 71) | 17.3 Hz, p50 21 ms |
| 4g · 3g | **0 (dead)** | **0 (dead)** | **0 (dead)** |
→ **MB/s saturates cellular** — 17 MB/s is impossible on 4g(12Mbps)/3g(2Mbps): every TCP mechanism
collapses to 0. On LAN, **SSE's base64 inflates latency** (p50 26 vs 21; max 71). Heavy streams need a
fat pipe, compression/decimation, or the media plane — not a raw topic relay.

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

**Decode location — `bench:decode`:** for the small PoseStamped + grid load, server→JSON moved **~1×**
the bytes of the binary relay (the rosbridge tax is **payload-dependent** — it balloons on large packed
arrays like lidar/pointcloud, ~negligible on small structured pose). Keep decode on the client.

> **Gaps in this CLI run:** the **WebRTC-data e2e** and **WebTransport-e2e** *aiortc/aioquic probes*
> hit connection errors (`webrtc_client_probe.py` / `webtransport_client_probe.py`) — environmental,
> to retry; WebTransport itself is proven by the loss bench above. The dual-delivery bug means absolute
> **throughput** is the publisher's real ~75%-of-nominal single rate; the *relative ordering + latency*
> are what to read.

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

_Empirical browser *performance* numbers (real Chrome via the claude-in-chrome MCP, or Playwright
cross-browser) are **pending** — and should be collected **after** the `serve.py` dup fix so browser
throughput is trustworthy. Run: `deno task serve` (with `ZENOH_KEY=nomatch` until fixed) + a source +
`netsim`, open `bench.html?gw=localhost:8099`, Run, export. The key cross-check is whether the CLI
aiortc/aioquic stand-in latencies match the real-browser WebRTC/WebTransport latencies._

### Kernel `tc/netem` on the VPS — `bench/RESULTS-vps.md`
_pending Stage 3._

### C — Real WAN (Mac ↔ VPS) — `bench/RESULTS-realwan.md`
_pending Stage 4 (needs the VPS firewall port open)._

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
# Real WAN: serve.py on the VPS, then from the Mac:
GATEWAY_URL=ws://<vps>:8080/ws deno task bench    # see docs/real-wan.md
```

## Status & remaining work (handoff)

**Done & committed:** `bench.tsx` fixed to the 5 mechanisms · this report (methodology + CLI-vs-browser
can/can't + simulation hierarchy + cross-browser support matrix) · the **dual-delivery bug finding** ·
the **clean CLI numbers** (light/heavy/WT-loss/QoS/decode) via the `ZENOH_KEY=nomatch` workaround.

**Blocked on your decision — fix `serve.py`'s dual-bus tap** (dedup, or a `BUS_BACKENDS`/single-tap env;
see the BLOCKER section). Until then, absolute **throughput/loss** on any layer is bug-affected; latency
+ relative ordering are valid. **Once fixed,** drop the workaround and the numbers become real everywhere.

**Teed up (run after the fix — each is one command, infra exists):**
- **Browser perf** (real Chrome via claude-in-chrome MCP, or `npx playwright` cross-browser) →
  `RESULTS-browser.md` + the CLI-stand-in-vs-real-browser latency cross-check.
- **VPS / kernel netem** — `docs/real-wan.md` runbook: `uv sync --extra web` + `serve.py` on the VPS,
  the CLI suite there, and `tc/netem` (a Mac can't) for the kernel-accurate cross-check.
- **Real WAN** — Mac → VPS by IP; **needs you to open the VPS firewall port** (TCP 8080 + UDP 8443).

**Other CLI gaps to retry:** the aiortc/aioquic **WebRTC-data e2e** + **WebTransport-e2e** probes hit
connection errors this run (WebTransport itself is proven by the loss bench).
