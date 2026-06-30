# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-07-01 00:12 · stream=pose · LCM gateway byte-relay · 3000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|---|--:|--:|--:|--:|--:|--:|
| pose | lan | ws | 300 | 24.9 | 0.45 | 0.92 | 1.11 | 0 |
| pose | lan | sse | 293.33 | 24.35 | 0.52 | 0.95 | 1.23 | 0 |
| pose | lan | poll | 300 | 24.9 | 1.17 | 1.9 | 2.2 | 0 |
| pose | 4g | ws | 280.33 | 23.27 | 83.3 | 115.93 | 127.22 | 0 |
| pose | 4g | sse | 288.33 | 23.93 | 80.77 | 113.42 | 127.92 | 0 |
| pose | 4g | poll | 288 | 23.91 | 111.83 | 166.13 | 177 | 0 |
| pose | 3g | ws | 264 | 21.91 | 275.53 | 385.33 | 421.76 | 0 |
| pose | 3g | sse | 269.33 | 22.36 | 287.13 | 401.19 | 432.47 | 0 |
| pose | 3g | poll | 218.67 | 18.15 | 448.1 | 874.06 | 961.67 | 0 |
| pose | lossy | ws | 292 | 24.24 | 153.24 | 263.31 | 287.9 | 0 |
| pose | lossy | sse | 280 | 23.24 | 165.86 | 273.83 | 295.98 | 0 |
| pose | lossy | poll | 272 | 22.58 | 193.17 | 337.53 | 367.25 | 0 |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
