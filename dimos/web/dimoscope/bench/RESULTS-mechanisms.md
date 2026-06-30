# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-06-30 15:25 · LCM gateway byte-relay · 4×PoseStamped @ 100Hz · 3000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| profile | mechanism | hz | kB/s | p50 | p95 | p99 | max | std | loss% |
|---|---|--:|--:|--:|--:|--:|--:|--:|--:|
| lan | ws | 293.33 | 24.35 | 1.71 | 4.78 | 6.79 | 9.64 | 1.52 | 0 |
| lan | sse | 286.67 | 23.8 | 1.72 | 4.7 | 7.98 | 9.11 | 1.54 | 0 |
| lan | poll | 296 | 24.57 | 2.04 | 4.56 | 6.6 | 7.36 | 1.3 | 0 |
| 3g | ws | 234.67 | 19.48 | 282.08 | 391.63 | 414.08 | 418.68 | 60.51 | 0 |
| 3g | sse | 236 | 19.59 | 282.84 | 383.9 | 408.94 | 418.21 | 59.03 | 0 |
| 3g | poll | 201.33 | 16.71 | 489.48 | 875.25 | 951.07 | 964.2 | 179.69 | 0 |
| 2g | ws | 82.33 | 6.83 | 1460.84 | 1875.79 | 1912.48 | 1912.48 | 322.25 | 0 |
| 2g | sse | 55.67 | 4.62 | 1536.93 | 1788.54 | 1801.27 | 1815.17 | 267.6 | 0 |
| 2g | poll | 0 | 0 | NaN | NaN | NaN | NaN | NaN | NaN |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
