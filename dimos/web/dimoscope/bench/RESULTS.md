# Transport benchmark — Bun↔LCM gateway (pytest)

_3000ms per scenario · ws://localhost:8092 · (unstamped)_

| scenario | topics | msgs | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 2000 | 666.67 | 55.99 | 0.6 | 1.45 | 19.6 |
| 1x PoseStamped (on-demand) | 1 | 498 | 166 | 13.94 | 0.39 | 1.11 | 7.13 |
| 1x OccupancyGrid (large) | 1 | 110 | 36.67 | 132.85 | 0.58 | 1.48 | 6.55 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **13.94 kB/s** vs **55.99 kB/s** for all 4 — a **75% reduction** on the WS hop (per-client gateway filtering).

> Over LCM the gateway still *receives* every topic (UDP multicast), so this saves the browser hop only. True end-to-end on-demand (robot→gateway) needs the Zenoh gateway (`declareSubscriber`/`undeclare`).
