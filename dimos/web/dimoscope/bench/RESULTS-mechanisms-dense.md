# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-06-30 17:32 · stream=dense · LCM gateway byte-relay · 4000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|---|--:|--:|--:|--:|--:|--:|
| dense | lan | ws | 18.25 | 17801.61 | 19.81 | 28.79 | 31.77 | 0 |
| dense | lan | sse | 18.5 | 18045.47 | 41.62 | 48.62 | 52.06 | 0 |
| dense | lan | poll | 18.5 | 18045.47 | 18.57 | 27.49 | 30.89 | 0 |
| dense | 4g | ws | 0.25 | 243.86 | 3802.99 | 3802.99 | 3802.99 | 0 |
| dense | 4g | sse | 0 | 0 | NaN | NaN | NaN | NaN |
| dense | 4g | poll | 0 | 0 | NaN | NaN | NaN | NaN |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
