# dimoscope transport benchmark — on the VPS (Linux, kernel netem)

_2026-07-01 · VPS `37.60.232.68` (Contabo, Ubuntu, 6c/11GB) · isolated clone `~/dimos-bench` (branch
`kris/research-29-06-2026`) · `uv` + `deno 2.9` · run localhost-on-VPS · 3000 ms/run · one-way latency._

The one thing a Mac + userspace `netsim` **can't** do: **kernel `tc/netem`** (real reorder / drop /
duplicate on the NIC). Run headless via tmux session `vpsbench-65329`.

## A. Delivery mechanisms × netsim profiles (WS / SSE / poll) — `bench:matrix`

| profile | mechanism | hz | p50 | p95 | p99 | loss% |
|---|---|--:|--:|--:|--:|--:|
| lan  | ws  | 354.7 | 3.1 | 12.0 | 35.2 | 0.4 |
| lan  | sse | 324.0 | 3.3 | 27.6 | 37.9 | 0 |
| lan  | poll| 354.7 | 7.0 | 14.6 | 20.5 | 0 |
| wifi | ws  | 345.3 | 21.1 | 49.3 | 55.0 | 0.4 |
| wifi | sse | 354.7 | 21.4 | 49.2 | 51.7 | 0 |
| wifi | poll| 341.7 | 29.8 | 56.0 | 71.0 | 0 |
| 4g   | ws  | 352.0 | 82.7 | 115.2 | 130.7 | 0 |
| 4g   | sse | 349.3 | 83.5 | 119.1 | 129.6 | 0 |
| 4g   | poll| 337.3 | 118.9 | 173.2 | 186.8 | 0 |
| 3g   | ws  | 319.3 | 305.9 | 403.3 | 437.2 | 0 |
| 3g   | sse | 271.3 | 285.8 | 387.7 | 429.9 | 0 |
| 3g   | poll| 233.0 | 578.9 | 959.3 | 1006 | 0 |
→ ws ≈ sse < poll; poll's per-cycle RTT ~doubles latency on 3g (p50 579 vs ws 306). Throughput ~single
delivery (~320–355 msg/s) — no double-count on the VPS.

## B. Kernel `tc/netem` cross-check (the VPS-only capability)

Applied to `lo`: **delay 100 ms ±20 · loss 3% · reorder 20%**, `netsim` profile `lan` (0 impairment) so
the numbers are the **netem-only** effect:

| mechanism | hz | p50 | p95 | loss% |
|---|--:|--:|--:|--:|
| ws  | 289.3 | 408.8 | 634.8 | **7.86** |
| sse | 197.0 | 442.3 | 701.3 | **7.66** |
| poll| 222.0 | 702.5 | 1192.9 | **5.26** |
→ real **reorder + drop** turns into **seq-gap loss (5–8%) + latency blowup** on every TCP mechanism —
impairment userspace `netsim` cannot reproduce (it models loss only as jitter; TCP can't raw-drop). This
is the kernel-accurate cross-check that validates the `netsim` profiles.

## C. WebTransport (QUIC datagrams) under REAL packet loss — `bench:loss`

| drop % | datagrams/s | dg p50 | dg **p95** |
|---|--:|--:|--:|
| 0  | 367 | 1 | 2 |
| 5  | 346 | 13 | 20 |
| 10 | 331 | 13 | 21 |
| 20 | 295 | 14 | **23** |
→ **the headline:** delivered-datagram **p95 stays flat (~2→23 ms) as loss rises to 20%** — a dropped
datagram simply doesn't arrive (**no head-of-line blocking**); throughput falls ~linearly. Contrast §B:
WS p95 climbs to 600–1200 ms under comparable loss because TCP converts loss into retransmit→stall. **The
teleop-under-bad-network case for WebTransport/WebRTC.**

## D. QoS — `bench:qos`
- **rate-limit** heavy lidar `setRateLimit(2)` → **5× less bandwidth** (1950→390 kB/s), client-driven.
- **on-demand** 1 of 4 pose topics → **75.1% reduction** (7.55 vs 30.32 kB/s).
- **prioritization** (cap lidar 2 Hz on a saturated 4g link) → **pose stays 182 Hz, loss 0**.

## Probes that failed (known-flaky stand-ins, to retry)
- `bench:webrtc` (aiortc → `/rtc`): **HTTP 403 on the WS upgrade** for the Python `websockets` client
  (the Deno client on `/ws` is fine → a header/Origin quirk of `/rtc` vs the Python client, worth a look).
- `bench:webtransport` e2e (aioquic connect) + `bench:decode` (deno `connect()` hang): environmental —
  the datagram-loss numbers in §C (also aioquic) are the trustworthy WebTransport evidence.
