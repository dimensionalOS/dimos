# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-07-01 09:32 · stream=pose · LCM gateway byte-relay · 3000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|---|--:|--:|--:|--:|--:|--:|
| pose | lan | ws | 328 | 27.23 | 1.34 | 4.63 | 6.74 | 0 |
| pose | lan | sse | 321.33 | 26.67 | 1.99 | 5.25 | 11.73 | 0 |
| pose | lan | poll | 329.33 | 27.34 | 1.52 | 3.55 | 4.57 | 0 |
| pose | wifi | ws | 325.33 | 27.01 | 15.3 | 23.64 | 26.06 | 0 |
| pose | wifi | sse | 323.33 | 26.84 | 15.17 | 22.76 | 27.33 | 0 |
| pose | wifi | poll | 325.33 | 27.01 | 23.8 | 36.3 | 43.85 | 0 |
| pose | 4g | ws | 308 | 25.57 | 83.86 | 113.84 | 125.7 | 0 |
| pose | 4g | sse | 313.33 | 26.01 | 82.88 | 115.5 | 127.35 | 0 |
| pose | 4g | poll | 318.67 | 26.45 | 110.88 | 162.19 | 181.46 | 0 |
| pose | 3g | ws | 269.33 | 22.36 | 284.66 | 386.3 | 413.8 | 0 |
| pose | 3g | sse | 264 | 21.91 | 320.18 | 429.62 | 449.44 | 0 |
| pose | 3g | poll | 251 | 20.83 | 592.17 | 910.68 | 987.94 | 0 |
| pose | lossy | ws | 310.67 | 25.79 | 147.65 | 249.3 | 284.33 | 0 |
| pose | lossy | sse | 302.67 | 25.12 | 194.69 | 320.48 | 344.23 | 0 |
| pose | lossy | poll | 306.67 | 25.46 | 223.5 | 364.67 | 412.35 | 0 |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
