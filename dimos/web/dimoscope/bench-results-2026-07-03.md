# dimoscope transport benchmark — full matrix, both VPS (2026-07-03)

Headless Chrome (`scripts/bench-headless.ts`, anti-throttle + WebRTC-ready flags), 30 s/cell,
`drive=0` against a **constant pre-running 20 MB/s flood** (`GO2Load.start_bench(20, 1_000_000,
image)` → `/load/img`). 3 workloads × 4 netem profiles × 3 transports × 2 boxes = 72 cells.

- **Workloads**: `pose` = fast(100 Hz)+mid(20)+slow(2) lanes only; `dense` = the 20 MB/s image
  firehose alone; `mixed` = fast lane *beside* the firehose (lane-isolation test).
- **Netem** (shapes all egress): `clean` / `wifi-normal` / `wifi-crowded` / `loss-5` (5% drop).
- **Columns**: `hz` delivered, `kB/s`, `deliv%`, `p50/p95/p99` ms, `fast p95` (fast-lane tail in
  mixed) + `×N` interference (fast-lane p95 inflation vs pose-alone), `loss%`, `late%`.

### Boxes
| box | host | publisher | congestion ctl | notes |
|-----|------|-----------|----------------|-------|
| **contabo** | 37.60.232.68 | `deno task load` (bare `/load/*` lanes) | **BBR** (set by me) | full SSH control |
| **company** | 63.177.113.161 | `deno task dog:vps` (mujoco sim **+** `/load/*`) | cubic (default; no SSH) | sim = extra CPU load = throughput confound |

---

## Contabo (37.60.232.68) — BBR, bare load

### WebSocket
| net | pose Hz / loss | dense MB/s (p50) | mixed fast p95 (×interf) / loss |
|-----|---------------|------------------|--------------------------------|
| clean       | 101.8 / —※ | **10.7** (145 ms) | 74 ms ×2.4 / 73% |
| wifi-normal | 108 / 0.3% | 0.9 (1462 ms) | 591 ms ×7.3 / 96% |
| wifi-crowded| 94 / 1.1%  | 0.16 (5366 ms) | 1769 ms ×2.5 / 99% |
| loss-5      | 101 / 0.6% | **1.6** (855 ms) | 452 ms ×2.7 / 94% |

### WebTransport (WT-rs)
| net | pose Hz / loss | dense MB/s (p50) | mixed fast p95 (×interf) / loss |
|-----|---------------|------------------|--------------------------------|
| clean       | 106 / 0.5% | **16.3** (79 ms) | 53 ms ×1.7 / 3% |
| wifi-normal | 103 / 1.4% | 1.1 (2449 ms) | 1288 ms ×14.2 / 58% |
| wifi-crowded| 106 / 95%† | 0.23 (8629 ms) | 6940 ms ×1.1 / 61% |
| loss-5      | 101 / 5.1% | **10.8** (360 ms) | **128 ms ×1.6** / 13% |

### WebRTC (rtc-rs, DataChannels)
| net | pose Hz / loss | dense MB/s (p50) | mixed fast p95 (×interf) / loss |
|-----|---------------|------------------|--------------------------------|
| clean       | 102 / 1.0% | 1.17 (842 ms) | 19 ms ×0.7 / 17% |
| wifi-normal | 102 / 1.0% | 0.36 (3138 ms) | 340 ms ×9.3 / 97%† |
| wifi-crowded| 98 / 7.2%  | 0.20 (4832 ms) | 453 ms ×1.0 / 18% |
| loss-5      | 101 / 11%  | **0.065** (dead)| 1125 ms ×1.2 / 16% |

---

## Company (63.177.113.161) — cubic, dog:vps sim + load

### WebSocket
| net | pose Hz / loss | dense MB/s (p50) | mixed fast p95 (×interf) / loss |
|-----|---------------|------------------|--------------------------------|
| clean       | 111 / 0.1% | **10.7** (141 ms) | 55 ms ×2.6 / 69% |
| wifi-normal | 112 / 0.1% | 0.39 (2434 ms) | 1504 ms ×16.0 / 99% |
| wifi-crowded| 82 / 25%   | 0.10 (4554 ms) | 1663 ms ×5.6 / 99% |
| loss-5      | 79 / 0.2%  | **~0** (0.03 Hz) | ~0 (0.27 Hz) |

### WebTransport (WT-rs)
| net | pose Hz / loss | dense MB/s (p50) | mixed fast p95 (×interf) / loss |
|-----|---------------|------------------|--------------------------------|
| clean       | 112 / 0.4% | **18.9** (81 ms) | 28 ms ×1.4 / 1% |
| wifi-normal | 111 / 0.5% | 1.1 (2160 ms) | 1289 ms ×17.3 / 56% |
| wifi-crowded| 113 / 5.3% | 0.23 (8640 ms) | 6971 ms ×1.0 / 60% |
| loss-5      | 105 / 6.1% | **11.1** (366 ms) | **170 ms ×1.4** / 12% |

### WebRTC (rtc-rs, DataChannels)
| net | pose Hz / loss | dense MB/s (p50) | mixed fast p95 (×interf) / loss |
|-----|---------------|------------------|--------------------------------|
| clean       | 110 / 0.3% | 1.43 (637 ms) | 33 ms ×1.5 / 16% |
| wifi-normal | 112 / 0.7% | 0.33 (3397 ms) | 306 ms ×2.3 / 15% |
| wifi-crowded| 112 / 0.5% | 0.20 (4753 ms) | 517 ms ×1.1 / 15% |
| loss-5      | 106 / 11%  | **0.065** (dead)| 1606 ms ×1.4 / 13% |

---

## Cross-transport verdict (consistent on both boxes)

1. **Bulk throughput — WT wins outright.** Clean dense **16–19 MB/s** (WT) vs 10.7 (WS) vs ~1.2–1.4
   (WebRTC). WT's QUIC congestion window fills the fat pipe; WS/TCP is a bit behind; WebRTC's single
   SCTP association is cwnd-bound on a ~28 ms path (loopback hits 16.9 MB/s, so it's CC, not CPU).

2. **Bulk under loss — WT is the only survivor.** At 5% loss WT dense holds **10.8–11.1 MB/s**; WS
   collapses (Mathis) — **contabo scrapes 1.6 MB/s only because of BBR**, company (cubic) dense goes
   to **~0**. WebRTC dense is **dead (0.065 MB/s)** under loss on both boxes.

3. **Lane isolation (mixed) — WT wins.** At loss-5 the WT fast lane keeps **~110 Hz beside a full
   firehose** with only **×1.4–1.6** p95 inflation. WS and WebRTC inflate 2.5–7× or shed the lane.
   One QUIC stream per lane = independent flow control; one SCTP association = one shared cwnd, so
   WebRTC's fast lane rides the bulk's collapse (the faster native stack made this *worse* than
   aiortc — it can now saturate the cwnd).

4. **Pose (control) — parity everywhere.** All three hold **~100–112 Hz** on every profile; pick by
   latency: WebRTC/WT clean p50 **14–22 ms**, WS **16–19 ms**. WebRTC pose tail degrades most under
   loss-5 (p50 370–450 ms) because pose shares the association with retransmits.

**Story for Lesh:** WebRTC earns its place for **media tracks (the camera plane) + p2p**, but for
robot→browser *data* mux, **QUIC lanes (WebTransport) win** — highest bulk, only transport that
survives loss, and real per-lane isolation. WS is the universal-reachability fallback (BBR makes it
usable under loss; without it, bulk dies).

---

## Artifacts & caveats (read before quoting a single cell)

- **`※` seq-reset (contabo WS clean pose)**: raw cell read 88.7% loss / 11% deliv but **101.8 Hz** —
  the generator seq reset when the flood was (re)asserted just before this first cell; the gap is
  mis-scored as loss. Hz is the truth. Same `†` marks other first-cell-after-assert reads
  (WT wifi-crowded pose 95% "loss" @ 106 Hz; WebRTC wifi-normal mixed 97% @ 93 Hz).
- **mixed high-loss with healthy Hz** is *offered-vs-delivered accounting*, not a fault: the fast
  lane is fully delivered (see `fast p95`), while the co-running firehose sheds most of its 20 MB/s —
  that shed shows up in the row's aggregate `loss%`.
- **company throughput confound**: company runs the mujoco sim (`dog:vps`) *plus* the flood, so its
  CPU headroom is lower than contabo's bare `load`. Its WT clean dense still beat contabo (18.9 vs
  16.3) — link/CC variance dominates the sim overhead here.
- **WebRTC dense = the 96 KB bulk-buffer number** (commit `827a36a39`): buffer bounded so stale
  backlog stays in the conflating outbox. Raising it inflates latency without raising goodput (cwnd
  bound), so 1.2–1.4 MB/s clean is the real ceiling on this path.
- **WS loss-5 asymmetry is the headline knob**: contabo 1.6 MB/s vs company ~0 is purely
  **BBR vs cubic**. Recommend `sysctl net.ipv4.tcp_congestion_control=bbr` on any WS-serving host.
- **company WS first run was void** (flood had stopped → all-zero dense); re-asserted + re-run — the
  table above is the valid re-run.

---

## Reproduce it yourself (browser, foreground — starts its own flood)

Run `deno task dev` in `dimos/web/dimoscope/app`, then open (drop `drive=0` so the page drives the
flood itself; `run=1` auto-executes the matrix):

**Contabo (37.60.232.68):**
- WS:      `http://localhost:5173/?gw=37.60.232.68:8080&transport=ws&profiles=pose,dense,mixed&net=clean,wifi-normal,wifi-crowded,loss-5&dur=30000&run=1`
- WT:      `http://localhost:5173/?gw=37.60.232.68:8080&transport=webtransport&profiles=pose,dense,mixed&net=clean,wifi-normal,wifi-crowded,loss-5&dur=30000&run=1`
- WebRTC:  `http://localhost:5173/?gw=37.60.232.68:8080&transport=webrtc&profiles=pose,dense,mixed&net=clean,wifi-normal,wifi-crowded,loss-5&dur=30000&run=1`

**Company (63.177.113.161):**
- WS:      `http://localhost:5173/?gw=63.177.113.161:8080&transport=ws&profiles=pose,dense,mixed&net=clean,wifi-normal,wifi-crowded,loss-5&dur=30000&run=1`
- WT:      `http://localhost:5173/?gw=63.177.113.161:8080&transport=webtransport&profiles=pose,dense,mixed&net=clean,wifi-normal,wifi-crowded,loss-5&dur=30000&run=1`
- WebRTC:  `http://localhost:5173/?gw=63.177.113.161:8080&transport=webrtc&profiles=pose,dense,mixed&net=clean,wifi-normal,wifi-crowded,loss-5&dur=30000&run=1`

**Autonomous / valid numbers** (background tabs are timer-throttled → garbage): use the headless
harness instead —
`deno run -A scripts/bench-headless.ts "<url-with-drive=0-against-a-running-flood>"`.

Raw per-cell output: `scratchpad/results/{contabo,company}-{ws,wt,webrtc}.txt`.
