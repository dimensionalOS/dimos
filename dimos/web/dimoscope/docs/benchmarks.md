# dimoscope transport benchmarks

How the three browser transports compare on throughput, latency, and bandwidth. The headline isn't "which transport wins" — it's that **the gateway is a byte-relay, so language and transport are not the bottleneck**: Python↔Zenoh keeps pace with Bun↔LCM, and the direct `zenoh-ts` path keeps pace with both.

See also: [README](./README.md) · [findings](./findings.md) · raw data: [`../bench/RESULTS.md`](../bench/RESULTS.md) (headless) and [`../bench/RESULTS-browser.md`](../bench/RESULTS-browser.md) (in-browser).

## Method
- Synthetic source (`bench_publisher.py`, no sim) so the numbers reflect the transport path, not a robot.
- Same scenarios + measurement module (`@dimos/topics/bench`, `bench/bench.ts`) across every transport.
- **Headless** measures the gateway→browser WS hop (via the gateway send-stamp). **In-browser** measures full **publish→browser** end-to-end for every transport, so `zenoh-ts` (browser-only) is compared apples-to-apples.
- 4000 ms per scenario.

## Headless (gateway→client WS hop) — `bench/RESULTS.md`
4× PoseStamped throughput scenario:

| transport | hz | kB/s | p50 ms | p95 ms | max ms |
|---|--:|--:|--:|--:|--:|
| Bun↔LCM | 330.5 | 27.76 | 0.34 | 0.78 | 2.03 |
| Python↔Zenoh | 331 | 27.80 | 0.43 | 0.95 | 4.01 |
| zenoh-ts (direct, Deno) | 331 | 27.80 | 0.40 | 0.80 | 3.40 |

Sub-ms p50 on all three; Python↔Zenoh holds parity with Bun↔LCM.

## In-browser (publish→browser, end-to-end) — `bench/RESULTS-browser.md`
4× PoseStamped throughput scenario, Chrome:

| transport | hz | kB/s | p50 ms | p95 ms | max ms |
|---|--:|--:|--:|--:|--:|
| Python↔Zenoh | 419 | 35.19 | 1.96 | 7.25 | 29.05 |
| Bun↔LCM | 415.25 | 34.87 | 1.21 | 4.79 | 15.85 |
| zenoh-ts (direct) | 417 | 35.02 | 1.83 | 6.15 | 24.18 |

Throughput parity (~417 Hz); end-to-end latency comparable across all three (p50 ~1.2–2.5 ms, p95 ~5–7 ms). The higher numbers vs headless are the real browser runtime under display load, measured the same way for all three.

## On-demand bandwidth
Subscribing **1 of 4** topics delivers **~6.95 kB/s** vs **~27.8 kB/s** for all four — a **~75% reduction** on the WS hop, on every transport.
- The two **gateways** still read the whole bus and filter per-client on the WS hop (`gateway.ts` downsample; `gateway_zenoh.py` subscribes `**`).
- **zenoh-ts is true end-to-end on-demand**: the browser `declareSubscriber`s each key, so unsubscribed keys never transit the network *or* the WS — the gateways can't match that.

## Takeaways
1. **Byte-relay ⇒ language isn't the bottleneck.** Python↔Zenoh ≈ Bun↔LCM. The gateway parses nothing (the browser decodes via the self-describing 8-byte hash), so it's I/O-bound, not CPU-bound.
2. **Throughput parity** across all three transports.
3. **~75% on-demand saving**, and only `zenoh-ts` makes it *true* end-to-end on-demand.
4. Latency is **comparable**, not a differentiator at this load — tail (p95/max) is where Zenoh's reliability vs LCM-multicast jitter shows up, but all are well within interactive range.

> Caveat worth noting: `zenoh-ts` headless runs under **Deno, not Bun** — its wasm-bindgen *bundler*-target WASM fails in Bun (`__wbindgen_start`), but Deno instantiates it. Both gateways run under Bun.
