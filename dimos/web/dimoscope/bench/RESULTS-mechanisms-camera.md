# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-06-30 17:34 · stream=camera · LCM gateway byte-relay · 4000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|---|--:|--:|--:|--:|--:|--:|
| camera | lan | ws | 18.25 | 9795.22 | 20.93 | 24.13 | 25 | 0 |
| camera | lan | sse | 18.25 | 9795.22 | 29.46 | 36.28 | 38.67 | 0 |
| camera | lan | poll | 18.25 | 9795.22 | 19.6 | 24.38 | 24.94 | 0 |
| camera | 4g | ws | 0.25 | 134.18 | 2125.4 | 2125.4 | 2125.4 | 0 |
| camera | 4g | sse | 0.25 | 134.18 | 2918.84 | 2918.84 | 2918.84 | 0 |
| camera | 4g | poll | 0 | 0 | NaN | NaN | NaN | NaN |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
