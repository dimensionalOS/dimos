# dimoscope transport benchmark — in-browser (all 3 transports)

_2026-06-29 21:29 · Chrome · 4000 ms/scenario · **end-to-end latency** (publish→browser), comparable across transports · synthetic `bench_publisher.py` source (no sim)._

Run via `app/bench.html` (`bash bench/serve-bench.sh` first → 3 servers + `bench_publisher` on both buses). Reuses the same `@dimos/topics/bench` scenarios + measurement as the headless Bun bench (`bench/bench.ts`), but in the **real browser runtime** — so **zenoh-ts (browser-only) is benched apples-to-apples** with the gateways.

## Python↔Zenoh — `ws://localhost:8088`
| scenario | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 419 | 35.19 | 1.96 | 7.25 | 29.05 |
| 1x PoseStamped (on-demand) | 104.25 | 8.76 | 1.83 | 7.13 | 23.69 |
| 1x OccupancyGrid (large) | 23.25 | 84.24 | 1.85 | 5.3 | 6.86 |

On-demand saving: **75%**.

## Bun↔LCM — `ws://localhost:8089`
| scenario | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 415.25 | 34.87 | 1.21 | 4.79 | 15.85 |
| 1x PoseStamped (on-demand) | 103.5 | 8.69 | 1.3 | 6.36 | 20.04 |
| 1x OccupancyGrid (large) | 23.25 | 84.24 | 1.9 | 5.64 | 7.01 |

On-demand saving: **75%**.

## zenoh-ts (direct) — `ws://localhost:10000`
| scenario | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 417 | 35.02 | 1.83 | 6.15 | 24.18 |
| 1x PoseStamped (on-demand) | 104 | 8.73 | 1.48 | 6.97 | 19.43 |
| 1x OccupancyGrid (large) | 23.5 | 85.14 | 2.46 | 6.29 | 10.72 |

On-demand saving: **75%** — here it's **true per-client on-demand** (the browser `declareSubscriber`s the key; unsubscribed keys never transit), vs the gateways filtering on the WS hop.

## Takeaways
- **Throughput parity** — all three deliver ~417 hz / ~35 kB/s on the 4× PoseStamped load; **zenoh-ts keeps up with the gateways** (no penalty for the direct path).
- **End-to-end latency** (publish→browser) is comparable across all three: p50 ~1.2–2.5 ms, p95 ~5–7 ms. (This differs from `RESULTS.md`'s headless numbers, which measure only the gateway→browser WS hop via the gateway send-stamp; here every transport is measured the same publish→browser way for fairness.)
- **75% on-demand** on each.

> **Why zenoh-ts is benched here, not in the CLI:** it ships a wasm-bindgen *bundler*-target WASM module that fails to instantiate in Bun (`wasm.__wbindgen_start is not a function`), so there's no headless Node/Bun client — the browser is its runtime. The Bun CLI bench (`bench/run.sh all`) covers the two gateways; this page covers all three.
