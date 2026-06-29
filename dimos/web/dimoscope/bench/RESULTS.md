# dimoscope transport benchmark — all transports

_Generated 2026-06-29 20:32 · headless `@dimos/topics` SDK vs each gateway · synthetic `bench_publisher.py` source (no sim)._

# Transport benchmark — Bun↔LCM gateway

_4000ms per scenario · ws://localhost:8090 · 2026-06-29 20:32_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1148 | 287 | 24.1 | 0 | 1 | 4 |
| 1x PoseStamped (on-demand) | 1 | 292 | 73 | 6.13 | 0 | 1 | 3 |
| 1x OccupancyGrid (large) | 1 | 69 | 17.25 | 62.5 | 0 | 1 | 1 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.13 kB/s** vs **24.1 kB/s** for all 4 — a **75% reduction** on the WS hop (per-client gateway filtering).

> Over LCM the gateway still *receives* every topic (UDP multicast), so this saves the browser hop only. True end-to-end on-demand (robot→gateway) needs the Zenoh gateway (`declareSubscriber`/`undeclare`).

---

# Transport benchmark — Python↔Zenoh gateway

_4000ms per scenario · ws://localhost:8091 · 2026-06-29 20:32_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1136 | 284 | 23.85 | 0 | 0.17 | 1.15 |
| 1x PoseStamped (on-demand) | 1 | 286 | 71.5 | 6 | 0 | 0.33 | 2.72 |
| 1x OccupancyGrid (large) | 1 | 70 | 17.5 | 63.4 | 0.04 | 0.43 | 1.27 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6 kB/s** vs **23.85 kB/s** for all 4 — a **75% reduction** on the WS hop (per-client gateway filtering).

> Over LCM the gateway still *receives* every topic (UDP multicast), so this saves the browser hop only. True end-to-end on-demand (robot→gateway) needs the Zenoh gateway (`declareSubscriber`/`undeclare`).

> **zenoh-ts (direct)** is browser-only (its client has no Bun/Node target), so it can't run in this headless harness — bench it via the in-app **Stats** latency on the `zenoh-ts (direct)` dropdown option.
