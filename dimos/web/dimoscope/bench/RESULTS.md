# dimoscope transport benchmark — all transports

_Generated 2026-06-29 23:30 · headless `@dimos/topics` SDK vs each gateway · synthetic `bench_publisher.py` source (no sim)._

# Transport benchmark — Bun↔LCM gateway

_4000ms per scenario · ws://localhost:8090 · 2026-06-29 23:30_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1316 | 329 | 27.63 | 0 | 1 | 6 |
| 1x PoseStamped (on-demand) | 1 | 332 | 83 | 6.97 | 0 | 1 | 1 |
| 1x OccupancyGrid (large) | 1 | 74 | 18.5 | 67.03 | 0 | 1 | 1 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.97 kB/s** vs **27.63 kB/s** for all 4 — a **75% reduction** on the WS hop.

---

# Transport benchmark — Python↔Zenoh gateway

_4000ms per scenario · ws://localhost:8091 · 2026-06-29 23:30_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1308 | 327 | 27.46 | 0 | 0.15 | 0.75 |
| 1x PoseStamped (on-demand) | 1 | 327 | 81.75 | 6.87 | 0 | 0.28 | 4.27 |
| 1x OccupancyGrid (large) | 1 | 75 | 18.75 | 67.93 | 0.05 | 0.47 | 0.62 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.87 kB/s** vs **27.46 kB/s** for all 4 — a **75% reduction** on the WS hop.

> **zenoh-ts (direct)** is browser-only (its client has no Bun/Node target), so it can't run in this headless harness — bench it via the in-app **Stats** latency on the `zenoh-ts (direct)` dropdown option.
