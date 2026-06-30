# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-06-30 17:30 · stream=lidar · LCM gateway byte-relay · 4000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|---|--:|--:|--:|--:|--:|--:|
| lidar | lan | ws | 9.5 | 1853.11 | 9.98 | 12.58 | 13.73 | 0 |
| lidar | lan | sse | 9.5 | 1853.11 | 16.57 | 18.06 | 18.75 | 0 |
| lidar | lan | poll | 9.5 | 1853.11 | 9.94 | 14.1 | 14.34 | 0 |
| lidar | 4g | ws | 1.25 | 243.83 | 2107.3 | 3389.68 | 3389.68 | 0 |
| lidar | 4g | sse | 0.75 | 146.3 | 2025.24 | 2889.35 | 2889.35 | 0 |
| lidar | 4g | poll | 0.25 | 48.77 | 876.61 | 876.61 | 876.61 | 0 |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
