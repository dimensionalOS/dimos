# dimoscope transport benchmark — REAL browser (Chrome, same-origin)

> **UPDATE (resolved):** the WS/WebRTC/WebTransport "0" rows below were **not** real failures. WS=0 was a
> **stale local serve process**; WebRTC needed a `/rtc` handshake fix (untyped `ws` param → 403) + a
> client-adapter fix (resolve on `dc.onopen`); WebTransport needed server **CORS** on `/cert`. After
> those, **all 5 transports work in real Chrome — including browser→VPS over the real WAN** (see
> `RESULTS-realwan.md`). The same-origin table below is kept as the original diagnostic record.



_2026-07-01 · real Chrome via claude-in-chrome MCP · page served same-origin from `serve.py` at
`http://localhost:8080/bench.html` · single `bench_source` (LCM) · 4000 ms/scenario · end-to-end
latency (publish→browser)._

This is the **real browser runtime** (not the CLI aiortc/aioquic stand-ins) — the actual `@dimos/topics`
SDK running in Chrome's JS engine, hitting the live `serve.py`.

## Results (Chrome)

| mechanism | throughput hz | p50 (ms) | p95 (ms) | large/grid hz | status |
|---|--:|--:|--:|--:|---|
| **SSE** | 240.8 | 2.47 | 4.76 | 18.25 | ✅ works |
| **HTTP poll** | 246.0 | 3.90 | 8.60 | 18.50 | ✅ works |
| WebSocket | 0 | — | — | 0 | ⚠ harness 0 (see note) |
| WebRTC data | 0 | — | — | 0 | ⚠ harness 0 (see note) |
| WebTransport | 0 | — | — | 0 | ⚠ harness 0 (see note) |

SSE ≈ HTTP-poll in real Chrome (~240–246 Hz for 4× pose, ~18 Hz for the large grid), consistent with the
CLI/VPS numbers for those two mechanisms.

## ⚠ The WS/WebRTC/WebTransport zeros are a **harness bug, not a server/transport limit**

Direct probing proves the server + browser path work; the `bench.tsx` measurement adapters don't record
them:

- **Raw `WebSocket('ws://localhost:8080/ws')` from the same page, with the correct subscribe frame
  (`{type:"subscribe",topics:["/bench/p0"]}`), received real data** (799-byte binary message). The
  browser *can* consume the `/ws` data plane; the SDK's default WebSocket transport adapter inside the
  `bench.tsx` measure loop is what fails to count it (SSE/poll use different adapters and work).
- The **Deno CLI client** hits the same `/ws` fine (VPS matrix: WS ≈ 354 Hz) — `/ws` is healthy.
- WebTransport: same-origin `/cert` returns 200 and `localhost` is a secure context, so the API is
  available — the 0 is again the harness adapter, not a capability gap.

**Cross-origin caveat found:** serving the page from vite (`:5175`) → transports at `:8080` makes
`fetch()` to `/cert` + `/health` fail — only `/sse` + `/poll` set `access-control-allow-origin: *`
(`servers/bench.py:93,128`). So **run the browser bench SAME-ORIGIN** (from `serve.py`'s `:8080`), or add
CORS to `/cert` for a cross-origin / real-WAN browser run.

## Follow-ups (flagged; do not affect the CLI/VPS numbers)
1. Fix the `bench.tsx` WebSocket/WebRTC/WebTransport **measurement adapters** (raw WS works → the bug is
   in the measure loop: subscribe timing / `reconnect:false` race), then re-run for real all-5 numbers.
2. Add `access-control-allow-origin` to `/cert` (+ `/health`) so the SDK can be served from a different
   origin than the gateway — required for the real-WAN "app on localhost → gateway on VPS IP" topology.
3. Authoritative transport numbers = the CLI/VPS ones (`RESULTS-vps.md`, `RESULTS-mechanisms*.md`,
   `bench:loss`). This page confirms **SSE/poll** in real Chrome and proves **/ws delivers to the browser**.
