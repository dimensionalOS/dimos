# Transport benchmark — Bun↔LCM vs Python↔Zenoh

Both gateways are thin byte-relays behind the **same** `@dimos/topics` SDK (the browser decodes
via `@dimos/msgs`). Same machine, same load: `bench_publisher` emits 4× `PoseStamped` @100 Hz
(`ts`-stamped for end-to-end latency) + an `OccupancyGrid` @20 Hz. 4 s/scenario.

Reproduce:
```bash
bench/run.sh          # Bun↔LCM      (:8090)
bench/run.sh zenoh    # Python↔Zenoh (:8091)
pytest bench/test_bench.py -s    # CI assertion (latency/throughput/on-demand bounds)
```
(Each run also writes `bench/last_run.md` + `bench/results.json`.)

### Bun↔LCM gateway
| scenario | topics | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|
| 4× PoseStamped (throughput) | 4 | 329 | 27.6 | 0.51 | 3.08 | 12.74 |
| 1× PoseStamped (on-demand) | 1 | 82 | 6.87 | 0.57 | 4.82 | 14.87 |
| 1× OccupancyGrid (large) | 1 | 18.5 | 67.0 | 0.63 | 2.38 | 7.87 |

### Python↔Zenoh gateway
| scenario | topics | hz | kB/s | lat p50 | lat p95 | lat max |
|---|--:|--:|--:|--:|--:|--:|
| 4× PoseStamped (throughput) | 4 | 333 | 28.0 | 0.46 | **1.09** | 15.32 |
| 1× PoseStamped (on-demand) | 1 | 83 | 6.97 | 0.42 | **0.82** | 2.95 |
| 1× OccupancyGrid (large) | 1 | 18.5 | 67.0 | 0.38 | **0.81** | 1.01 |

## Findings

1. **"Is Python slow enough to matter?" → No, not here.** The gateway only shuffles bytes (the
   browser decodes), so it's I/O-bound; both runtimes saturate the publisher and sit at
   **sub-millisecond p50 latency** with identical throughput (~330 Hz, ~28 kB/s).

2. **Zenoh has markedly tighter *tail* latency.** p95 is **~3× lower** under Zenoh (1.09 ms vs
   3.08 ms for 4× pose; 0.81 ms vs 2.38 ms for the grid), and max on the large grid is ~1 ms vs
   ~8 ms. LCM is UDP multicast (best-effort, jitter + fragment reassembly for the grid); Zenoh's
   reliable path delivers more consistently. The "better server long-term" instinct points at
   **Zenoh the transport**, not the gateway language.

3. **On-demand subscription cuts bandwidth ~75%** on both (subscribe 1 of 4 → 25% of the bytes).
   Caveat: today both filter at the **WS hop** — they still *receive* every topic. **True
   end-to-end on-demand** (robot→gateway) is Zenoh-only via per-client `declareSubscriber`/
   `undeclare`; wiring that into `gateway_zenoh.py` is the next step and would make Zenoh win the
   bandwidth axis on the robot→gateway link too.

## Takeaway
Keep the **Bun↔LCM gateway** as the fast, zero-Python-dep default (reuses dimoscope + DimSim's
patched `@dimos/lcm`). Use the **Python↔Zenoh gateway** for tighter tail latency, real per-key
QoS, and (next) true end-to-end on-demand. The browser SDK is byte-identical either way — that's
the point of the transport abstraction.
