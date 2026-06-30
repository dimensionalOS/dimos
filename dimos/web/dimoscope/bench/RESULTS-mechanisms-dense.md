# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-07-01 00:13 · stream=dense · LCM gateway byte-relay · 3000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|---|--:|--:|--:|--:|--:|--:|
| dense | lan | ws | 17.33 | 16907.46 | 20.95 | 22.47 | 23.85 | 0 |
| dense | lan | sse | 17.33 | 16907.46 | 25.53 | 31.51 | 71.42 | 0 |
| dense | lan | poll | 17.33 | 16907.46 | 21.09 | 23.14 | 23.41 | 0 |
| dense | 4g | ws | 0 | 0 | NaN | NaN | NaN | NaN |
| dense | 4g | sse | 0 | 0 | NaN | NaN | NaN | NaN |
| dense | 4g | poll | 0 | 0 | NaN | NaN | NaN | NaN |
| dense | 3g | ws | 0 | 0 | NaN | NaN | NaN | NaN |
| dense | 3g | sse | 0 | 0 | NaN | NaN | NaN | NaN |
| dense | 3g | poll | 0 | 0 | NaN | NaN | NaN | NaN |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
