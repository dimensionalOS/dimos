# dimoscope — Benchmark, QoS & Real-WAN

One reference for the quantitative side of dimoscope: the in-browser transport benchmark (with fresh
numbers), the QoS priority story, and the runbook for driving the dog over the real internet from a VPS.

The load source for all of this is the **`go2-load`** dog (`dimos/robot/benchmark/go2_load.py`): fixed
multi-rate `/load/{fast,mid,slow,grid,cloud}` lanes (auto-run) **plus** a crankable `/load/img` flood via
`start_bench`/`stop_bench` `@rpc`. The standalone `load` blueprint is the same module without the sim.

```bash
deno task serve   # the gateway → http://localhost:8080 (+ WT QUIC :8443)
deno task dog     # go2-load: the /load/* lanes + the crankable flood
deno task app     # then open http://localhost:8080/ → Topics tab → Benchmark drawer
```

---

## 1. The benchmark — in the real browser, across transports

The **Topics tab → Benchmark** drawer measures the *live* transport end-to-end (publish→browser latency,
`recvTs − srcTs`) across the workload profiles, then lets you copy the results as Markdown. Pick a
transport from the topbar dropdown (WebSocket · SSE · HTTP poll · WebRTC data · WebTransport, or Auto),
crank the **load generator** up the ladder (`light 2 → camera 11 → dense 20 → depth-hd 50 → raw-1080p 180
→ firehose 300` MB/s on `/load/img`), and sweep. The `pose` profile measures the small `/load/{fast,mid,slow}`
lanes (always live); the heavy profiles measure `/load/img` at whatever the generator is emitting.

### Results (localhost loopback · end-to-end · maxHz=∞ · 2026-07-01)

These are **loopback** numbers (no network loss) — they characterize the *relay/browser pipeline ceiling*
and the *small-vs-bulk isolation*, not WAN behavior (for that see §3).

**Small lanes + a 20 MB/s bulk stream — WebTransport vs WebSocket:**

| transport | scenario | hz | MB/s | p50 ms | p95 ms | p99 ms | loss% |
|---|---|--:|--:|--:|--:|--:|--:|
| **Auto → WebTransport** | pose (fast+mid+slow) | 98 | 0.008 | **2.14** | 7.45 | 15.84 | 0.25 |
| Auto → WebTransport | dense (img @20 MB/s) | 1 | 0.95 | 1612 | 2660 | 2660 | **73.3** |
| Auto → WebTransport | mixed | 16 | 0.01 | 165 | 344 | 430 | 78.0 |
| **WebSocket** | pose (fast+mid+slow) | 104 | 0.009 | **0.57** | 1.70 | 4.0 | 0 |
| WebSocket | dense (img @20 MB/s) | 18.5 | **17.6** | 6.3 | 8.8 | 12.3 | **0** |
| WebSocket | mixed | 128 | 17.9 | 0.73 | 7.3 | 8.8 | 0 |

**WebSocket up the overload ladder (small lanes stay alongside the bulk):**

| offered | scenario | hz | MB/s delivered | p50 ms | p95 ms | p99 ms | loss% |
|---|---|--:|--:|--:|--:|--:|--:|
| 180 MB/s | pose | 107 | 0.009 | **0.68** | 5.8 | 23.1 | 0 |
| 180 MB/s | raw-1080p (img) | 33.5 | **192** | 29 | 37 | 40 | 0 |
| 300 MB/s | pose | 106 | 0.009 | **0.97** | 8.6 | 14.2 | 0 |
| 300 MB/s | firehose (img) | 28.5 | **272** | 77 | 104 | 122 | **13.6** |

### What the numbers say

- **The small high-rate lanes stay crisp under any bulk load.** `pose` holds ~100 Hz at **0.6–2 ms p50 and
  ~0% loss** on both transports — even with a 20–300 MB/s stream running beside it. The important topic
  survives the firehose.
- **On loopback, WebSocket wins bulk handily — but read *why* before concluding anything about WT.** WS
  sustains **~18 / 192 / 272 MB/s** at 20 / 180 / 300 offered with 0% loss up to ~190 MB/s. The WT bulk row
  looks terrible (dense: 1 Hz, 1.6 s, "73% loss"), but that is **not** datagram shedding: the transport
  size-routes frames (small ≤1100 B → datagrams; larger → a *new reliable unidirectional QUIC stream per
  frame*, `gateway/transports/webtransport.py`), so the 1 MB images ride reliable streams through **aioquic
  (pure-Python QUIC)** — a CPU-bound per-message-stream path that is the real loopback bottleneck. The
  "73% loss" is the SDK's seq-gap metric misreading a *backlog of delayed-but-not-dropped* frames (reliable
  streams don't drop). WS wins here because the browser + uvicorn use C-optimized TCP, not because WT is
  fundamentally worse. WT's *small-frame* datagram path is fine locally (pose: 98 Hz, 2 ms).
- **WebTransport's real advantage — no head-of-line blocking — only appears under packet loss**, which
  loopback doesn't have (so TCP/WS never stalls and looks perfect). Measure it under loss: a real WAN (§3),
  or inject loss locally on the link (Linux `tc qdisc add dev lo root netem loss 5%`; macOS `dnctl`/dummynet
  via `pfctl`). That, not a fat lossless link, is where WT (UDP, no HoL) pulls ahead of WS (TCP HoL).
- **The Python byte-relay is not the bottleneck.** One gateway process + WS sustains **~280–290 MB/s** before
  saturating; latency climbs 0.7 → 77 ms and loss 0 → 14% only as offered load goes 20 → 300 MB/s. That's
  the pipeline ceiling, and the tab survives it (WS backpressure, ~44 MB heap — no hard crash).
- **The crash/ceiling case is real and visible.** At 300 MB/s the bulk lane plateaus (~272 MB/s) with
  double-digit loss and 10–100× latency — exactly the "big data that stresses the browser" tier, recorded
  rather than hidden. Push higher / add real loss to force an actual stall.

Re-run any of this yourself: `deno task dog`, open the Benchmark drawer, set the generator tier, pick the
matching workload profile, **Run sweep**, **copy Markdown**. `?gw=host:port` points it at a remote VPS;
`?dur=ms` tunes the window.

---

## 2. QoS — declare importance, enforce it at the bottleneck

**The claim:** on a constrained browser link, *declaring* some topics more important and *enforcing* it at
the gateway keeps pose/teleop crisp while bulk (lidar/camera) degrades gracefully — instead of everything
getting equally janky.

**Model: declaration → enforcement → transport.**
1. **Client declares** a lane per topic (sane defaults, configurable) — `packages/web/src/qos.ts`.
2. **Gateway enforces** at the per-client egress — `gateway/qos.py`'s priority outbox, wired into
   `gateway/data.py`: under backpressure it drains high-priority topics first and conflates/drops the
   lowest-priority `best_effort` topics first — never the high-priority ones.
3. **Transport reinforces** (optional): put high-priority small topics on a no-HoL primitive (WebTransport
   datagrams) so they can't queue behind bulk within one connection.

**The four lanes** (auto-assigned by name/type via `defaultLane`, overridable per-topic; grounded in DDS / ROS 2):

| lane | reliability | priority | under load | typical topics |
|---|---|---|---|---|
| **command** | reliable, ordered | highest | never dropped | cmd_vel, goals, teleop |
| **sensor** | best-effort | high | latest-wins | pose, odom, imu, tf, joints |
| **default** | reliable | normal | bounded queue | general |
| **bulk** | best-effort | low | conflates / sheds first | lidar, camera, pointcloud, maps |

`gateway/qos.py` replaces the single FIFO with `{priority-class → {topic → slot}}`: `best_effort` topics get
a **latest-only slot** (a backed-up lidar overwrites itself, never grows a queue); `reliable` topics get a
bounded deque (DDS `keep_last`). The writer drains by **weighted round-robin** (high classes get most of the
budget; low classes keep a non-starving floor). Convergence of established patterns — Foxglove ws-bridge
(bounded per-client queue), Reactive Streams `onBackpressureLatest` (conflation), DDS/Zenoh (reliability +
priority bands) — mirrored at the browser-link boundary the bus QoS can't reach.

**The A/B result** (pose @100 Hz light+high-priority · a heavy low-priority stream · link paced to ~4 Mbps
via `EGRESS_KBPS`), scheduler **OFF (FIFO)** vs **ON (priority outbox)**:

| mode | pose hz | **pose p50** | pose p95 | heavy hz |
|---|--:|--:|--:|--:|
| **OFF** (FIFO) | 55 | **1294 ms** | 2419 ms | 120 |
| **ON** (priority outbox) | 99.5 | **4 ms** | 9 ms | 116 |

Pose latency collapses **1294 ms → 4 ms (~300×)** and rate is restored (55 → 99.5 Hz) while the heavy stream
keeps flowing (conflated to its freshest frame). **The catch that makes it real:** priority only bites if the
backlog sits *in the gateway outbox* — so the gateway paces each client's writer to its link budget
(**egress shaping**, `EGRESS_KBPS`); without it, kernel/proxy socket buffers absorb the backlog downstream
and FIFO it (bufferbloat defeats any gateway QoS). Same discipline as WebRTC's `bufferedAmount` watermark.

**Client override** — lanes are *defaults*; a subscriber overrides any topic's QoS and it flows over the wire
(`subscribe` op carries `priority`/`reliability`/`depth`, resolved client-side from `topic.setQos({lane})`).
Declaring `pose=bulk` + `lidar=command` from the client **flips** which stream survives the saturated link.
Operator config (`qos.rules.json`, copy `qos.rules.example.json`) classifies custom per-blueprint topics by
name/type glob; the subscriber always wins over it.

**Caveat:** priority only matters under saturation — on a fat link both modes look identical (correctly).

---

## 3. Real-WAN — the dog over the internet from a VPS

Run the gateway + `go2-load` on a VPS with a public IP and drive it from your browser over the real path
(RTT, reorder, drop). The JS SDK doesn't change: the app + Benchmark drawer read `?gw=host:port`.

**Serve the app from `http://localhost` (Mac Vite + `?gw=VPS`), not from `http://<public-IP>`.** WebTransport
requires a **secure context** — `localhost` qualifies, a bare public-IP HTTP origin does **not** (the
`WebTransport` API is simply `undefined` there, so the app silently uses WebSocket). Keeping the page on
`localhost` also lets `ws://`/`http://` reach the VPS IP without a mixed-content block.

### WAN results (2026-07-01, measured — Mac ⇄ VPS `37.60.232.68`)

Deploy: `go2-load` runs on the Linux VPS (the forkserver worker pool that fails on macOS works on Linux);
`/load/*` streams over the real internet, **and the full dog renders headless** — dimsim launches Chromium
(`DIMSIM_RENDER=cpu`, SwiftShader software WebGL), publishes `/color_image` + `/camera_info`, and the
rendered scene shows in the browser over WAN. *(Gotcha: this box's **IPv6 route to the playwright CDN is
broken**, so `playwright install chromium` — which dimsim runs on first launch — hangs on the IPv6 download.
Fix: fetch the browser + headless-shell over **IPv4** (`wget -4 …chrome-linux64.zip` /
`…chrome-headless-shell-linux64.zip`) into `~/.cache/ms-playwright/chromium{,_headless_shell}-<build>/` with
empty `INSTALLATION_COMPLETE`+`DEPENDENCIES_VALIDATED` markers, or set the system to prefer IPv4. Then dimsim
renders. Use **cam: jpeg** over WAN — the webrtc cam mode needs ICE, which doesn't establish over a raw IP.)*
For a headless benchmark **without** the sim, the standalone `deno task load:flood` source is lighter.

`pose` = the small `/load/{fast,mid,slow}` lanes; `dense` = a 20 MB/s `/load/img` flood. **Latency is NaN /
unreliable over WAN** — end-to-end latency compares a VPS timestamp to a Mac timestamp, and the cross-machine
**clock skew** swamps the signal (only *relative*, same-clock comparisons are meaningful). So read **hz +
loss%**:

| transport | scenario | clean hz | clean loss% | @5% loss+40ms hz | @loss loss% |
|---|---|--:|--:|--:|--:|
| **WebSocket** | pose | 125 | 0 | 132 | **0** |
| WebSocket | dense (20 MB/s) | 24 | 0 | 24 (~23 MB/s) | **0** |
| **WebTransport** | pose | 133 | 0.19 | 116 | **4.9** |
| WebTransport | dense (20 MB/s) | **0.75** | **77** | **0** | — |

**What the WAN test actually showed (honestly):**
- **WebSocket is the more robust transport here.** TCP carries both the 20 MB/s bulk *and* the small lanes
  through 5% loss with **0% data loss** (retransmits absorb it; latency rises, which we can't quantify over
  WAN). It "just works" over a raw IP.
- **WebTransport's bulk path collapses** — `/load/img` gets **0.75 Hz / 77%** even on the clean path and **0
  Hz** under loss. This is the **aioquic (pure-Python QUIC) per-message-stream** bottleneck (each >1100 B
  frame opens a new reliable uni-stream), the same ceiling seen on loopback — *not* a WAN effect.
- **WebTransport's datagram path (small lanes) works** — pose holds ~120–133 Hz; under 5% loss it drops ~4.9%
  (datagrams shed lost packets instead of retransmitting — no HoL, but the data is gone). Reliable-vs-unreliable
  is the real tradeoff; the theoretical *no-HoL latency* win couldn't be shown here because latency is
  unmeasurable over WAN (clock skew).
- **The UDP transports are finicky over a raw IP.** WebTransport connects from a `localhost` page via
  self-signed cert-hash pinning, but the QUIC handshake fails if packet loss hits *during* the handshake
  (bring the connection up first, then apply loss). WebRTC-data did not establish (ICE needs a public host
  candidate / STUN). **TCP transports (WS/SSE/poll) connect reliably by raw IP; the UDP ones want real infra**
  — a domain + CA TLS for WebTransport (via Caddy/Let's Encrypt, no hash-pinning), STUN/TURN for WebRTC.

**Bottom line:** over this real WAN, the byte-relay + **WebSocket sustains ~23 MB/s through 5% loss with zero
data loss** and is the safe default; WebTransport's win needs (a) a non-Python QUIC bulk path and (b) a domain
+ TLS to be demonstrated cleanly. The honest takeaway matches §1: transport choice is nuanced, and WS is a
strong, robust baseline.

### VPS — run the dog (Ubuntu; needs `uv`; `deno` only to build the app)

**Firewall — open the ports** (already done on this box): TCP `8080`; UDP `:8443` (WebTransport) + the WebRTC
ICE range. `sudo ufw allow 8080/tcp; sudo ufw allow 8443/udp; sudo ufw allow 32768:60999/udp`.

The simulated dog is **heavier than `--extra web`**: `go2-load` composes the *smart* `unitree_go2` (mapping +
navigation + perception), and dimsim launches a headless-Chromium renderer (Deno + Playwright Chromium,
auto-cloned `paul-nechifor/DimSim`). Install the sim stack, not just the web extra:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh                 # uv
cd ~/code/dimos && git fetch && git checkout kris/research-29-06-2026 && git pull
uv sync --extra unitree --extra sim --extra mapping             # the dog stack (pin exact extras to your box;
                                                                # `--extra all` is the sledgehammer fallback)

cd dimos/web/dimoscope
uv run python -m gateway &                                       # :8080 (app + /ws /sse /poll /rtc /media /cert) · QUIC :8443
DIMOS_TRANSPORT=zenoh DIMSIM_RENDER=cpu \
  uv run dimos --simulation dimsim run go2-load &                # the dog, headless CPU render
```

`DIMSIM_RENDER=cpu` forces software rendering on a headless box (default is `gpu`); dimsim always runs
`--headless` and needs `git` + outbound internet on first run (repo + Chromium download).

**Light fallback** (if the full sim is too heavy on the box): `uv sync --extra web`, then either
`uv run dimos run load` (the standalone `/load/*` source, coordinator-wired, keeps the interactive
`start_bench` crank) or `DIMOS_TRANSPORT=lcm uv run python scenarios/bench.py` (ultra-light, env-tuned). Both
give a WAN benchmark with no dog viz.

**Open the firewall** (the part people forget): **TCP** the HTTP/WS port (`8080`); **UDP** `:8443`
(WebTransport/QUIC) **and** the WebRTC ICE range. e.g. `sudo ufw allow 8080/tcp; sudo ufw allow 8443/udp;
sudo ufw allow 32768:60999/udp`.

### Mac — point the browser at the VPS (zero new code)

```bash
deno task app     # Vite on :5173, served from your Mac
# open  http://localhost:5173/?gw=<VPS_IP>:8080          → the app (drive the dog over WAN)
# Topics tab → Benchmark → sweep each transport for real-path hz/latency/loss
```

Keep the client page on `http://localhost` (so it may reach `ws://`/`http://` on the VPS IP without a
mixed-content block). WebTransport's self-signed cert is ECDSA ≤14 days — the service regenerates it and the
client always fetches the current hash from `/cert`.

### Later — a physical Go2 over the internet (coworker's path)

No code change: run the gateway on a machine *next to the real dog* (`uv sync --extra unitree`, robot IP +
per-device AES-128 key), point `go2-load` / `unitree_go2` at the hardware, forward `8080/tcp` + `8443/udp`
from that network, and any browser drives it via `http://localhost:5173/?gw=<their-ip>:8080`. Same app, same
`SafetyEgress` (velocity clamp + TTL deadman + stop-on-disconnect).

---

Sources: [ROS 2 QoS](https://design.ros2.org/articles/qos.html) ·
[Zenoh QoS](https://docs.rs/zenoh/latest/zenoh/qos/index.html) ·
[Foxglove ws-bridge](https://github.com/foxglove/ros-foxglove-bridge) ·
[Reactive Streams backpressure](https://reactivex.io/documentation/operators/backpressure.html)
