# dimos data-path benchmark — delivery mechanisms × network conditions

_Generated 2026-06-30 17:35 · stream=mixed · LCM gateway byte-relay · 4000ms/run · end-to-end (publish→recv) · netsim TCP proxy._

Latency is one-way ms (publisher and client share a clock). `loss%` is wire drop from seq gaps.

| stream | profile | mechanism | hz | kB/s | p50 | p95 | p99 | loss% |
|---|---|---|--:|--:|--:|--:|--:|--:|
| mixed | lan | ws | 710.75 | 2091.66 | 0.61 | 13.05 | 16.23 | 99.54 |
| mixed | lan | sse | 706.25 | 2041.66 | 0.69 | 16.71 | 20.82 | 99.54 |
| mixed | lan | poll | 712.75 | 2091.83 | 1.03 | 16.74 | 20.29 | 99.54 |
| mixed | 4g | ws | 85.5 | 218.08 | 1582.63 | 3110.48 | 3134.11 | 99.94 |
| mixed | 4g | sse | 68.25 | 164.35 | 2154.95 | 3176.53 | 3189.09 | 99.96 |
| mixed | 4g | poll | 19.25 | 53.9 | 932.65 | 981.58 | 981.58 | 99.99 |
| mixed | 3g | ws | 28.75 | 56.47 | 3225.36 | 3396.2 | 3396.52 | 99.98 |
| mixed | 3g | sse | 4.25 | 1.24 | 410.74 | 415.03 | 415.03 | 100 |
| mixed | 3g | poll | 0 | 0 | NaN | NaN | NaN | NaN |

Profiles (netsim TCP shaping): lan=0ms · wifi=8ms/±3 · 4g=50ms/±15/12Mbps · 3g=150ms/±40/2Mbps · 2g=400ms/±120/280kbps · lossy=80ms/±80/5Mbps.
