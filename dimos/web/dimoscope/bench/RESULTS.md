# dimoscope transport benchmark — all transports

_Generated 2026-06-29 23:49 · headless · **end-to-end latency** (publish→client) · synthetic `bench_publisher.py` source (no sim)._

# Transport benchmark — Bun↔LCM gateway

_4000ms per scenario · ws://localhost:8090 · 2026-06-29 23:49_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1322 | 330.5 | 27.76 | 0.34 | 0.78 | 2.03 |
| 1x PoseStamped (on-demand) | 1 | 331 | 82.75 | 6.95 | 0.31 | 0.98 | 8.96 |
| 1x OccupancyGrid (large) | 1 | 74 | 18.5 | 67.03 | 0.26 | 0.53 | 0.61 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.95 kB/s** vs **27.76 kB/s** for all 4 — a **75% reduction** on the WS hop.

---

# Transport benchmark — Python↔Zenoh gateway

_4000ms per scenario · ws://localhost:8091 · 2026-06-29 23:49_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1324 | 331 | 27.8 | 0.43 | 0.95 | 4.01 |
| 1x PoseStamped (on-demand) | 1 | 331 | 82.75 | 6.95 | 0.43 | 0.78 | 1.02 |
| 1x OccupancyGrid (large) | 1 | 75 | 18.75 | 67.93 | 0.33 | 0.73 | 1.07 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.95 kB/s** vs **27.8 kB/s** for all 4 — a **75% reduction** on the WS hop.

---

# Transport benchmark — zenoh-ts (direct, Deno)

_4000ms per scenario · ws://localhost:10000 · 2026-06-29 23:50_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1324 | 331 | 27.8 | 0.4 | 0.8 | 3.4 |
| 1x PoseStamped (on-demand) | 1 | 332 | 83 | 6.97 | 0.33 | 0.65 | 0.72 |
| 1x OccupancyGrid (large) | 1 | 74 | 18.5 | 67.03 | 0.3 | 0.62 | 9.85 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.97 kB/s** vs **27.8 kB/s** for all 4 — a **75% reduction** on the WS hop.

> All three measured end-to-end (publish→client) for a fair compare. The two gateways run under **Bun**; **zenoh-ts runs under Deno** — it can't run in Bun (its wasm-bindgen bundler-target WASM fails: `__wbindgen_start`), but Deno instantiates it. zenoh-ts has no gateway in the read path → true per-client on-demand.
