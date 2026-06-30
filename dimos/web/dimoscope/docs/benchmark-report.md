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

### A — CLI / headless (single host), LAN sanity (zenoh tap disabled for single delivery)
| mechanism | hz (lcm-only) | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|
| WebSocket | 297 (4× pose) | 0 ms | 0.27 | 0.6 |
_Latency is sub-ms on LAN as expected; the full mechanism × profile latency matrix is pending the
`serve.py` fix so throughput/loss are also valid. WebRTC/WebTransport via aiortc/aioquic stand-ins._

### A — CLI / headless (single host) — `bench/RESULTS-*.md`
_pending Stage 1._

### B — Browser / real runtime + cross-browser — `bench/RESULTS-browser.md`
_pending Stage 2 (Playwright Chromium/Firefox/WebKit). Includes the browser×mechanism support matrix
and the CLI-stand-in-vs-browser-real agreement check._

### Kernel `tc/netem` on the VPS — `bench/RESULTS-vps.md`
_pending Stage 3._

### C — Real WAN (Mac ↔ VPS) — `bench/RESULTS-realwan.md`
_pending Stage 4 (needs the VPS firewall port open)._

---

## Recommendations (consolidated)

> Finalized after the runs; the prior single-host findings already point here:
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
