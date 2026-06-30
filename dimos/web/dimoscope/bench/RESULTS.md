# dimoscope transport benchmark — all transports

_Generated 2026-06-30 16:11 · headless · **end-to-end latency** (publish→client) · synthetic `bench_publisher.py` source (no sim)._

# Transport benchmark — Deno↔LCM gateway

_4000ms per scenario · ws://localhost:8090 · 2026-06-30 16:11_

| scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | max | std | loss% |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1332 | 333 | 27.97 | 0.44 | 2.04 | 4.55 | 5.31 | 0.76 | 0 |
| 1x PoseStamped (on-demand) | 1 | 332 | 83 | 6.97 | 0.75 | 2.78 | 5.37 | 9.1 | 1.1 | 0 |
| 1x OccupancyGrid (large) | 1 | 74 | 18.5 | 67.03 | 0.64 | 2.05 | 4.76 | 4.76 | 0.78 | 0 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.97 kB/s** vs **27.97 kB/s** for all 4 — a **75% reduction** on the WS hop.

---

# Transport benchmark — Python↔Zenoh gateway

_4000ms per scenario · ws://localhost:8091 · 2026-06-30 16:11_

| scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | max | std | loss% |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1316 | 329 | 27.63 | 0.84 | 3.41 | 5.33 | 5.46 | 1.11 | 0 |
| 1x PoseStamped (on-demand) | 1 | 332 | 83 | 6.97 | 0.97 | 3.5 | 6.35 | 9.74 | 1.22 | 0 |
| 1x OccupancyGrid (large) | 1 | 74 | 18.5 | 67.03 | 0.57 | 3.43 | 3.73 | 3.73 | 0.96 | 0 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.97 kB/s** vs **27.63 kB/s** for all 4 — a **75% reduction** on the WS hop.

---

# Transport benchmark — zenoh-ts (direct, Deno)

_4000ms per scenario · ws://localhost:10000 · 2026-06-30 16:11_

| scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | max | std | loss% |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 4x PoseStamped (throughput) | 4 | 1332 | 333 | 27.97 | 0.53 | 3.01 | 4.51 | 5.58 | 0.96 | 0 |
| 1x PoseStamped (on-demand) | 1 | 333 | 83.25 | 6.99 | 0.45 | 3.3 | 5 | 12.49 | 1.19 | 0 |
| 1x OccupancyGrid (large) | 1 | 74 | 18.5 | 67.03 | 0.62 | 1.81 | 3.07 | 3.07 | 0.62 | 0 |

**On-demand bandwidth:** subscribing 1 of 4 topics delivered **6.99 kB/s** vs **27.97 kB/s** for all 4 — a **75% reduction** on the WS hop.

> All three measured end-to-end (publish→client) for a fair compare, all headless under **Deno**. zenoh-ts has no gateway in the read path → true per-client on-demand.
