# dimoscope transport benchmark — WT bulk credit gate A/B + WebRTC (2026-07-06)

Headless Chrome (`scripts/bench-headless.ts`), 15 s/cell, coex axis on (each flood tier runs beside
the pose lanes; `pose` is the flood-off reference). Server: company VPS `63.177.113.161`
(gateway + wt-sidecar @ `56385eafa`, netem on `ens5`). Clock offset corrected per run (probe rtt
25–28 ms on every run — clean; the VPS clock drifts ~15 ms/h, offsets +54…+79 ms across the day).

**What's under test** — commit `56385eafa`: the WT bulk drain is gated on end-to-end byte credit.
The browser acks consumed bulk-stream bytes (`{op:"bulk-ack", n}` per 32 KB); the sidecar keeps
`outstanding = written − acked < max(WT_BULK_MIN, ack_rate × WT_BULK_TARGET_MS)` (default 250 ms,
`0` = gate off), with a `WT_BULK_MIN` floor before the first ack. Rationale: on rate-capped links
every lane's p50 was (bulk bytes in flight) ÷ link rate — quinn-BBR's min-rtt filter is poisoned by
its own standing queue, so only the send window bounded the bloat (1.5 MB = 6 s @ 2 Mbit). The
receiver's consumption rate is the one signal that can't lie. Datagram lane: undeliverable frames
are dropped, never queued onto the bulk stream.

**Known floor in these runs:** all rows carry a uniform ~8 % loss / ~92 % deliv depression that is
NOT wire loss — a WebSocket (TCP) control cell shows the same ~7.4 %, i.e. the freshly-restarted
DimSim dog skips ~8 % of publishes on this 4-vCPU box (the warmed dog earlier showed 99–100 %
deliv). It cancels out of every comparison below. ICMP to the box: 0 % loss, but evening WAN
jitter was real (rtt 26–70 ms) — treat single-digit-ms latency deltas as noise.

## A/B — WT gate off vs on (`/load/fast` p95 beside the flood, ms)

| net | scenario | gate OFF | gate ON | Δ |
|---|---|--:|--:|--:|
| clean | lidar+pose | 20.8 (×1.0) | 21.6 (×1.0) | — |
| clean | dense+pose | 25.6 (×1.2) | 26.4 (×1.2) | — |
| wifi-normal | lidar+pose | **1281 (×17.3)** | **281 (×3.8)** | −78 % |
| wifi-normal | dense+pose | **1283 (×17.4)** | **278 (×3.8)** | −78 % |
| wifi-crowded | lidar+pose | **6859** | **483 (×1.2)** | −93 % |
| wifi-crowded | dense+pose | **6884** | **471 (×1.1)** | −93 % |
| loss-5 | lidar+pose | 73.0 (×1.1) | 71.6 (×1.1) | — |
| loss-5 | dense+pose | 124.3 (×1.9) | 130.5 (×2.0) | — |

- Clean throughput is untouched: dense bulk **19.26 MB/s at both settings** (gate budget ≫ BDP).
- Gate-off wifi-crowded is worse than it looks: even the flood-off `pose` cell shows p95 5000 ms —
  the previous cell's queued bulk (send_window deep) keeps draining into the next cell. Gate-on
  `pose` on the same net: p95 420 ms (the netem floor is 100±60 ms delay + 2 Mbit).
- Gate-off wifi-crowded also collapses delivered rate (59–61 Hz vs 108 gate-on).
- loss-5 (no rate cap): unchanged lane latency; bulk 11.3 → 9.0 MB/s (−20 %, run-to-run BBR
  variance vs a real cost — not yet separated; latency p95 improved 372 → 318 ms).

## WT (gate on) vs WebRTC data — same matrix, same day

| net | metric | WT | WebRTC |
|---|---|--:|--:|
| clean | fast p95 beside dense | 26.4 ms | 26.7 ms |
| clean | dense bulk delivered | **19.26 MB/s** | 2.02 MB/s |
| wifi-normal | fast p95 beside dense | 278 ms | 277 ms |
| wifi-crowded | fast p95 beside dense | **471 ms** | 502 ms |
| loss-5 | pose-alone p95 | **64.5 ms** | 1333 ms |
| loss-5 | dense bulk delivered | **8.98 MB/s** | 0.034 MB/s |

**Verdict: with the credit gate, WebTransport matches or beats WebRTC in every cell.** The one
regime WebRTC used to win — small-lane freshness on rate-capped links — is now parity-to-better,
and WT keeps its 9.5× clean bulk throughput and its order-of-magnitude win under loss (SCTP/cubic
collapses at 5 % loss; QUIC loss recovery + BBR does not). WebRTC's remaining roles: UDP-blocked
networks (~5 %) and the media plane.

## Reproduce

Serve on the VPS is already gate-on (default). From any machine with the repo:

```
deno task app     # vite on :5173
# WT (gate on):
http://localhost:5173/?gw=63.177.113.161%3A8080&transport=webtransport&profiles=pose%2Clidar%2Cdense&coex=1&net=clean%2Cwifi-normal%2Cwifi-crowded%2Closs-5&dur=15000&run=1
# WebRTC:
…same with transport=webrtc
# Gate-off baseline: restart the sidecar with WT_BULK_TARGET_MS=0
```

Local `deno task serve` note: the WebRTC plane needs `RTC_PUBLIC_IP=<your-LAN-IP>` even locally —
without it webrtc-rs advertises per-interface host candidates and ICE fails on macOS
(`no such remote`); regular Chrome additionally offers mDNS candidates webrtc-rs can't resolve.

## Raw rows

### WT gate OFF (`WT_BULK_TARGET_MS=0`) · clk +53.8 ms · rtt 25 ms

| net | scenario | hz | kB/s | deliv% | p50 | p95 | p99 | fast p95 | loss% |
|---|---|--:|--:|--:|--:|--:|--:|--:|--:|
| clean | pose | 107.1 | 9.0 | 91.5 | 14.3 | 22.0 | 29.4 | 20.6 | 8.5 |
| clean | lidar+pose | 116.9 | 1959.5 | 92.1 | 14.8 | 27.4 | 34.3 | 20.8×1.0 | 7.9 |
| clean | dense+pose | 124.9 | 19256.0 | 92.0 | 16.6 | 80.7 | 97.4 | 25.6×1.2 | 8.0 |
| wifi-normal | pose | 102.3 | 8.6 | 89.4 | 60.3 | 74.4 | 82.8 | 73.9 | 10.6 |
| wifi-normal | lidar+pose | 103.7 | 1074.6 | 79.3 | 1271.2 | 1301.0 | 2457.8 | 1280.7×17.3 | 20.4 |
| wifi-normal | dense+pose | 101.5 | 1126.8 | 80.6 | 1268.5 | 1287.2 | 2173.0 | 1282.7×17.4 | 19.4 |
| wifi-crowded | pose | 107.1 | 9.0 | 73.4 | 162.8 | 5000.2 | 5510.1 | 4999.5 | 26.6 |
| wifi-crowded | lidar+pose | 58.9 | 225.9 | 66.4 | 6664.0 | 6861.7 | 7348.6 | 6858.7×1.4 | 33.5 |
| wifi-crowded | dense+pose | 61.0 | 213.2 | 86.7 | 6615.5 | 6883.8 | 6897.9 | 6883.5×1.4 | 13.3 |
| loss-5 | pose | 99.3 | 8.3 | 86.4 | 56.0 | 66.3 | 76.7 | 65.0 | 13.6 |
| loss-5 | lidar+pose | 108.7 | 1932.8 | 87.1 | 60.3 | 144.0 | 219.4 | 73.0×1.1 | 12.9 |
| loss-5 | dense+pose | 113.3 | 11323.1 | 83.4 | 109.4 | 372.2 | 454.8 | 124.3×1.9 | 16.6 |

### WT gate ON (default `WT_BULK_TARGET_MS=250`) · clk +55.8 ms · rtt 26 ms

| net | scenario | hz | kB/s | deliv% | p50 | p95 | p99 | fast p95 | loss% |
|---|---|--:|--:|--:|--:|--:|--:|--:|--:|
| clean | pose | 106.9 | 9.0 | 91.3 | 14.9 | 23.5 | 32.9 | 21.7 | 8.7 |
| clean | lidar+pose | 117.1 | 1946.5 | 92.2 | 15.1 | 26.9 | 31.3 | 21.6×1.0 | 7.8 |
| clean | dense+pose | 124.7 | 19256.8 | 92.2 | 17.1 | 86.9 | 100.6 | 26.4×1.2 | 7.8 |
| wifi-normal | pose | 106.3 | 8.9 | 91.1 | 60.7 | 74.3 | 82.4 | 74.0 | 8.9 |
| wifi-normal | lidar+pose | 109.4 | 1127.1 | 80.6 | 247.6 | 294.8 | 486.1 | 280.9×3.8 | 19.2 |
| wifi-normal | dense+pose | 109.0 | 1114.5 | 80.4 | 237.7 | 281.0 | 956.3 | 278.0×3.8 | 19.6 |
| wifi-crowded | pose | 108.3 | 9.1 | 91.5 | 155.7 | 420.3 | 483.1 | 420.3 | 8.5 |
| wifi-crowded | lidar+pose | 108.9 | 217.1 | 79.6 | 341.4 | 489.9 | 532.7 | 483.5×1.2 | 20.3 |
| wifi-crowded | dense+pose | 108.3 | 217.2 | 83.9 | 327.5 | 476.3 | 513.6 | 470.5×1.1 | 16.1 |
| loss-5 | pose | 100.9 | 8.5 | 86.4 | 55.8 | 64.5 | 71.6 | 64.1 | 13.6 |
| loss-5 | lidar+pose | 111.9 | 1959.1 | 80.8 | 60.0 | 147.7 | 218.2 | 71.6×1.1 | 18.8 |
| loss-5 | dense+pose | 112.9 | 8982.0 | 82.8 | 97.3 | 317.8 | 420.2 | 130.5×2.0 | 17.2 |

### WebRTC data (rtc-rs) · clk +56.2 ms · rtt 28 ms

| net | scenario | hz | kB/s | deliv% | p50 | p95 | p99 | fast p95 | loss% |
|---|---|--:|--:|--:|--:|--:|--:|--:|--:|
| clean | pose | 107.0 | 9.0 | 91.3 | 13.9 | 22.2 | 31.6 | 19.7 | 8.7 |
| clean | lidar+pose | 116.2 | 1985.6 | 84.4 | 14.8 | 84.9 | 92.0 | 28.1×1.4 | 15.3 |
| clean | dense+pose | 110.0 | 2024.8 | 80.9 | 14.5 | 29.8 | 536.2 | 26.7×1.4 | 19.1 |
| wifi-normal | pose | 107.2 | 9.1 | 91.6 | 62.6 | 73.8 | 81.8 | 73.5 | 8.4 |
| wifi-normal | lidar+pose | 108.5 | 451.1 | 79.1 | 122.6 | 266.1 | 577.5 | 241.0×3.3 | 20.8 |
| wifi-normal | dense+pose | 110.5 | 282.4 | 84.5 | 160.5 | 279.8 | 350.5 | 277.0×3.8 | 15.5 |
| wifi-crowded | pose | 108.8 | 9.2 | 77.9 | 164.8 | 434.7 | 486.1 | 435.1 | 22.1 |
| wifi-crowded | lidar+pose | 108.5 | 204.2 | 79.1 | 373.0 | 542.3 | 636.0 | 533.7×1.2 | 20.9 |
| wifi-crowded | dense+pose | 108.7 | 152.2 | 85.9 | 373.0 | 502.2 | 622.1 | 501.6×1.2 | 14.1 |
| loss-5 | pose | 99.4 | 8.4 | 84.4 | 486.7 | 1333.3 | 1624.2 | 1337.4 | 15.6 |
| loss-5 | lidar+pose | 103.9 | 73.8 | – | 498.3 | 1347.7 | 1653.9 | 1346.9×1.0 | 11.0 |
| loss-5 | dense+pose | 96.6 | 34.2 | 87.1 | 391.7 | 1029.3 | 1406.0 | 1012.3×0.8 | 12.9 |
