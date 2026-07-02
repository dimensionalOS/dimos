# dimoscope — Benchmark, QoS & Real-WAN

Benchmarks, the QoS model, and the runbook for driving a robot over the real internet. The load
source throughout is `go2-load` (`dimos/robot/benchmark/go2_load.py`): multi-rate
`/load/{fast,mid,slow,grid,cloud}` lanes plus a `/load/img` flood cranked via
`start_bench`/`stop_bench` `@rpc`. The standalone `load` blueprint is the same module without the sim.

```bash
deno install && deno task build   # once — the app bundle the gateway serves at /
deno task serve                   # gateway + WT sidecar → http://localhost:8080 (+ QUIC :8443)
deno task load                    # /load/* lanes + the crankable flood (`deno task dog` = the full dimsim dog)
# open http://localhost:8080/ → Topics tab → Benchmark drawer
```

---

## 1. The benchmark — in the real browser, across transports

The Topics tab → Benchmark drawer measures the live transport end-to-end (publish→browser,
`recvTs − srcTs`) as a **matrix sweep** — netem profiles × workloads × maxHz × repeats, one cell per
condition, asserted server-side per group and restored after (also on Stop). Transports come from the
topbar dropdown; the generator ladder runs `lidar 2 → camera 11 → dense 20 → depth-hd 50 →
raw-1080p 180 → firehose 300` MB/s on `/load/img`, and with **auto-drive** on the sweep sets that
publisher config itself per heavy profile (`start_bench`, manual state restored) — the profile name
IS the offered load, and every row reports **offered kB/s / deliv%** derived from the per-topic seq
span (works without the RPC too). `pose` measures the always-on small lanes; `mixed` is the
beside-bulk coexistence case — expand any row (▸) for **per-lane stats** (pose fresh beside the img
backlog is the fair-data-plane evidence) and **1 s sparklines** (ramp, outage recovery). The
`all-lanes` + `on-demand` pair in one sweep prints the on-demand bandwidth cut (~75% on the WS hop).
Results **append across sweeps and transport switches** (the cross-wire table builds live), persist
to a run history (localStorage; time-series stripped on disk — the JSON export keeps it), and a
pinned baseline adds Δ suffixes on p95 (relative %) and loss (percentage-point) for matching
`scenario+netem+maxHz` cells.

Methodology (`packages/web/src/bench.ts`): a warmup window is discarded (subscribe ramp-up, QoS
negotiation), then `durMs` of measurement, then a grace window so a delayed frame counts as late, not
lost — loss% is only what never arrived (NaN under client rate limits, where gaps are intentional).
Rates divide by elapsed wall clock. An NTP-style `{op:"ping"}` probe (lowest-RTT of 5) corrects clock
offset before each sweep, so end-to-end latency holds across machines. Every row is stamped with the
wire that actually carried it (`wire: …` — the Auto WT→WS fallback is silent otherwise), the offered
load, page origin, and UA.

### Results (localhost loopback · end-to-end · maxHz=∞ · 2026-07-02 · `31a1f92b7`)

Loopback: no network loss, so these show the relay/browser ceiling and lane isolation, not WAN
behavior (§3). loss% marked `*` is a source-side seq floor (2–12%, varies per run, identical on TCP
where wire loss is impossible); unmarked bulk loss% is the outbox conflating what the wire can't carry.

**Small lanes + a 20 MB/s bulk stream — WebTransport (WT-rs) vs WebSocket:**

| transport | scenario | hz | MB/s | p50 ms | p95 ms | p99 ms | loss% | late% |
|---|---|--:|--:|--:|--:|--:|--:|--:|
| **WebTransport** | pose (fast+mid+slow) | 94.7 | 0.008 | **1.02** | 11.4 | 18.0 | 7.8* | 0 |
| WebTransport | dense (img @20 MB/s) | 18.8 | **18.3** | **9.6** | 26.9 | 30.1 | **0** | 0 |
| WebTransport | mixed | 123.7 | 18.3 | 1.08 | 12.8 | 22.8 | 3.5* | 0 |
| **WebSocket** | pose (fast+mid+slow) | 98.2 | 0.008 | **0.88** | 11.4 | 20.3 | 5.3* | 0 |
| WebSocket | dense (img @20 MB/s) | 18.5 | **18.0** | 21.2 | 33.1 | 35.2 | **0** | 0 |
| WebSocket | mixed | 115.2 | 18.1 | 2.94 | 26.8 | 33.6 | 8.9* | 0 |

**Up the overload ladder (the small lanes ride alongside the bulk flood):**

| offered | transport | scenario | hz | MB/s delivered | p50 ms | p95 ms | p99 ms | loss% | late% |
|---|---|---|--:|--:|--:|--:|--:|--:|--:|
| 180 MB/s | WebSocket | pose | 100.7 | 0.008 | 1.39 | 11.6 | 15.2 | 2.4* | 0 |
| 180 MB/s | WebSocket | raw-1080p (img) | 7.75 | 45.4 | 146 | 165 | 175 | 71 | 0 |
| 180 MB/s | WebTransport | pose | 94.5 | 0.008 | **0.97** | 11.0 | 19.0 | 4.1* | 0 |
| 180 MB/s | WebTransport | raw-1080p (img) | 26.5 | **155** | 159 | 225 | 232 | 5.4 | 0 |
| 300 MB/s | WebSocket | pose | 92.7 | 0.008 | 3.35 | 17.8 | 26.4 | 2.9* | 0 |
| 300 MB/s | WebSocket | firehose (img) | 3.75 | 36.6 | 289 | 359 | 412 | 85 | 0 |
| 300 MB/s | WebSocket | firehose, sustained 20 s | 3.85 | 37.6 | 282 | 328 | 412 | 85.4 | 0 |

Takeaways:

- pose holds ~95–100 Hz at ~1 ms p50 beside 20–180 MB/s of bulk on both transports (3.4 ms at
  300 MB/s offered). On WebTransport the isolation is structural: pose rides datagrams, bulk rides one
  reliable length-prefixed uni-stream, inside one connection.
- At robot-realistic bulk (≤20 MB/s) the transports tie on throughput (~18 MB/s, 0% loss); WT-rs
  delivers it at p50 9.6 ms vs WS 21 ms.
- Past that they diverge: WT-rs moves 155 of 180 MB/s offered; WS tops out around 40–45 MB/s of
  incompressible bulk and the outbox conflates the rest to the freshest frame (shedding by design —
  the tab stays interactive through the 300 MB/s firehose). permessage-deflate burns CPU on the random
  payloads; disabling it on `/ws` is an open follow-up.
- Loopback has no loss, so TCP never stalls here. The no-HoL-under-loss case is §3 — or inject loss
  locally (Linux `tc qdisc add dev lo root netem loss 5%`; macOS `dnctl`/dummynet).

Re-run: pick workloads (+ netem/maxHz axes), **Run sweep**, **copy Markdown** — or paste an
export's `repro:` URL. The whole config drives from the URL: `?gw=host:port` targets a remote
gateway, `?transport=<id>` pins the transport, `?profiles=pose,dense,mixed` selects workloads,
`?dur=ms` tunes the window, `?maxHz=0,60` is the QoS axis (comma list; 0 = ∞),
`?net=clean,loss-5` the netem axis, `?reps=N` repeats, `?load=<tier>` / `?loadHz=&loadBytes=` /
`?loadKind=cloud` the generator config, `?drive=0` turns auto-drive off, and **`?run=1` executes on
load** (3 s cancel chip — netem flips are gateway-wide). Bench URLs land on the bench tab
(`?tab=worldview|topics` picks the page explicitly), and each export embeds its `repro:` URL, so
every published table is one paste away from re-measurement.

---

## 2. QoS — declare importance, enforce it at the bottleneck

On a constrained browser link, declaring some topics more important keeps pose/teleop fresh while
bulk degrades gracefully, instead of everything getting equally stale.

Declaration → enforcement → transport:
1. Client declares a lane per topic (defaults by name/type, overridable) — `packages/web/src/qos.ts`.
2. Gateway enforces at the per-client egress — `gateway/qos.py`'s priority outbox: under backpressure
   it drains high-priority topics first and sheds the lowest-priority best-effort topics first.
3. Transport reinforces (optional): high-priority small topics on a no-HoL primitive (WT datagrams)
   can't queue behind bulk within one connection.

The four lanes (auto-assigned via `defaultLane`, grounded in DDS / ROS 2):

| lane | reliability | priority | under load | typical topics |
|---|---|---|---|---|
| **command** | reliable, ordered | highest | never dropped | cmd_vel, goals, teleop |
| **sensor** | best-effort | high | latest-wins | pose, odom, imu, tf, joints |
| **default** | reliable | normal | bounded queue | general |
| **bulk** | best-effort | low | conflates / sheds first | lidar, camera, pointcloud, maps |

`gateway/qos.py` replaces the single FIFO with `{priority class → {topic → slot}}`: best-effort
topics keep a latest-only slot (a backed-up lidar overwrites itself, never grows a queue), reliable
topics a bounded deque (DDS `keep_last`), drained by weighted round-robin with a non-starving floor
for the lowest class. Prior art: Foxglove's bounded per-client queue, Reactive Streams'
`onBackpressureLatest`, DDS/Zenoh reliability + priority bands — applied at the browser link, the one
boundary bus QoS can't reach.

The A/B (pose @100 Hz high-priority · a heavy low-priority stream · link paced to ~4 Mbps),
scheduler OFF (FIFO) vs ON:

| mode | pose hz | **pose p50** | pose p95 | heavy hz |
|---|--:|--:|--:|--:|
| **OFF** (FIFO) | 55 | **1294 ms** | 2419 ms | 120 |
| **ON** (priority outbox) | 99.5 | **4 ms** | 9 ms | 116 |

Priority only bites if the backlog sits in the outbox, so the gateway paces each client's writer to
its link budget (`EGRESS_KBPS`, applied to the `/ws` writer and the WT drain); without pacing,
kernel/proxy socket buffers absorb the backlog downstream and FIFO it.

Lanes are defaults. The `subscribe` op carries per-topic `priority`/`reliability`/`depth`
(`topic.setQos(...)` client-side) and the subscriber's declaration wins; operator config
(`qos.rules.json`, see `qos.rules.example.json`) classifies custom topics by name/type glob in
between. Under a fat link both scheduler modes measure identical — priority only matters at saturation.

---

## 3. Real-WAN — the dog over the internet from a VPS

Run the gateway + `go2-load` on a VPS with a public IP; the app + Benchmark drawer read
`?gw=host:port`. Serve the page from `http://localhost` (Mac Vite), not from the public IP:
WebTransport needs a secure context — on a bare-IP HTTP origin the API is `undefined` and the app
silently uses WebSocket.

### WAN deploy notes (2026-07-01, Mac ⇄ a public-IP VPS)

`go2-load` runs on the Linux VPS; `/load/*` streams over the real internet and the dog renders
headless — dimsim launches Chromium (`DIMSIM_RENDER=cpu`, SwiftShader), publishes `/color_image` +
`/camera_info`. If `playwright install chromium` hangs on first launch, the usual cause is a broken
IPv6 route to the CDN: fetch the browser + headless-shell over IPv4 (`wget -4 …chrome-linux64.zip` /
`…chrome-headless-shell-linux64.zip`) into `~/.cache/ms-playwright/chromium{,_headless_shell}-<build>/`
with empty `INSTALLATION_COMPLETE`+`DEPENDENCIES_VALIDATED` markers. Use **cam: jpeg** over WAN — the
webrtc cam mode needs ICE, which doesn't establish over a raw IP. For a headless benchmark without the
sim, `deno task load:flood` is lighter.

### Network profiles — simulate deployment conditions from the browser

A `net:` select in the topbar flips server-side `tc netem` profiles on the gateway host —
`clean · wifi-normal (10mbit/40ms/0.3%) · wifi-crowded (2mbit/100ms/1.5% bursty) · wifi-edge
(700kbit/200ms/5% bursty) · disaster (200kbit/500ms/8%) · loss-3/loss-5 (loss, no bw cap)` — amber
while anything is shaped (the condition affects teleop/camera/WorldView, not just the bench). The
BenchDrawer's Network section is the **sweep axis**: multi-select profiles and the matrix asserts
each per group (settle, verified from the POST body, re-asserted against the self-heal on long
matrices, pre-sweep profile restored after — also on Stop), plus momentary UDP/all outage buttons
(legal mid-run: failover is data). Every result cell is stamped `net:<profile>`.

Opt-in and scoped: install the root wrapper once with `deno task netem:install` (= `sudo install
-m755 gateway/scripts/dimos-netem /usr/local/bin/` + a sudoers entry for `$USER` scoped to that one
script), then run the gateway with `NETEM_CTL=1`. Shaping hits the gateway's egress — TCP 8080 plus
**all egress UDP** (WebTransport's :8443 and WebRTC's ICE-negotiated ports; a port-scoped UDP match
would let WebRTC bypass the shaper). SSH stays untouched; box-level UDP (DNS, VPNs) is shaped while
a profile is active, and every apply self-heals after 15 min. Apply loss **after** the transport is
connected: QUIC handshakes fail under loss.

The browser (topbar select + bench sweep axis) is one of three equivalent controls over the same
wrapper — all views show whatever is actually active, from any of them:

```bash
sudo dimos-netem wifi-crowded    # on the gateway box — apply · `clean` clears · `status` prints
curl -X POST http://<gw>:8080/netem -H 'content-type: application/json' -d '{"profile":"loss-5"}'
curl http://<gw>:8080/netem      # from anywhere — the endpoint the browser panel uses
```

Payloads are incompressible by design: the generators publish random bytes, about the entropy of real
camera/lidar data. WebSocket negotiates permessage-deflate, which squeezes regular payloads ~1000:1 —
a compressible bench payload measures the compressor, not the transport (and asymmetrically: QUIC/WT
has no equivalent). Sanity check: the netem band byte counters must move ≈ the bytes the browser reports.

### The profile matrix (2026-07-02 · 20 MB/s offered flood · clock-synced end-to-end latency)

WebTransport is served by the native WT-rs sidecar (`gateway/wt-sidecar/` — Rust,
[wtransport](https://github.com/BiagioFesta/wtransport)/quinn). It owns only the UDP `:8443` listener
(rows self-identify as `wire: dimoscope/WT-rs`); the Python gateway keeps the bus tap, `SafetyEgress`
and QoS config and feeds it over a unix socket (`gateway/pipe.py`). Teleop/goal/rpc flow browser →
sidecar → pipe → the same `SafetyEgress`; a dying session (or a dying sidecar) still zero-twists the
robot.

Both transports, back-to-back in one session on a clean-install deployment (fresh clone → `uv sync
--extra web` → `deno install && deno task build` → `deno task serve` + `deno task load`). pose = the
small lanes alone; bulk = the steady-state flood; crowded rows use a 20 s window (a 1 MB frame needs
~4.1 s of wire time at 2 mbit):

| net profile | transport | pose hz / loss% / p50 / p99 (ms) | bulk (1 MB frames) |
|---|---|---|---|
| clean | **WT-rs** | 111 / 0.2 / **22** / **34** | **19.0 MB/s** / 0% / **p50 56 ms** |
| clean | WS | 101 / 0.7 / 25 / 79 | 11.5 MB/s / 40% shed / p50 230 ms |
| wifi-normal | **WT-rs** | 106 / 0 / **56** / **78** | **1.22 MB/s** *(≈ the 10 mbit cap)* |
| wifi-normal | WS | 85 / 2.9 / 57 / **347** | 0 frames *(Mathis)* |
| wifi-crowded | **WT-rs** | 106 / 0.7 / 129 / 463 | 195 kB/s *(= the 2 mbit cap)* / p50 7.3 s per frame; mixed: pose keeps **107 Hz / p99 249** beside it |
| wifi-crowded | WS | 111 / 0 / 120 / 153 | 1 frame / 20 s; mixed: **0.49 Hz total / 99.5% loss** — bulk owns the TCP pipe |
| **loss-5** | **WT-rs** | 106 / 4.9 / **51** / **65** | **10.7 MB/s** / 49% shed |
| **loss-5** | WS | 113 / 0 / 60 / **163** | **0** *(Mathis at 5% < one frame)* |

- loss-5: QUIC keeps both lanes alive — pose p99 65 ms shedding 4.9%, bulk 10.7 MB/s via per-stream
  retransmission. TCP delivers every pose frame at a 163 ms retransmit tail and moves zero bulk.
  Freshness vs completeness: pose/teleop on datagrams, commands/bulk on reliable streams.
- TCP bulk dies by Mathis, not bandwidth: 0.3% random loss already stops 1 MB frames at wifi-normal.
  WT-rs tracks the shaped caps exactly (1.22 MB/s, 195 kB/s).
- Clean-path bulk is wire-limited on WT-rs — 19.0 of 20 MB/s at p50 56 ms, pose at p50 22 ms on the
  same connection. WS manages 11.5 MB/s at p50 230 ms (congestion control + deflate CPU).
- wifi-crowded is the coexistence case: pose alone is queue-dominated on both transports, but with
  bulk sharing the connection WS collapses to 0.49 Hz / 99.5% loss while WT-rs keeps pose at 107 Hz /
  p99 249 ms beside cap-pinned bulk. One QUIC connection isolates its lanes; one TCP pipe doesn't.
  The fix at this tier is §2's rate-limits/conflation, not transport choice.
- QUIC handshakes fail under loss (connect first, then apply); WebRTC-data needs STUN/TURN; Auto
  falls back to WS when WT can't establish. The outage buttons (`UDP 3s/10s`, `all 10s`) cover
  fallback/reconnect timing.

Recommendation: `Auto (WT→WS)` — WT-rs where UDP works, WebSocket everywhere else.

Two design constraints behind those numbers (`wt-sidecar/src/main.rs`):
- Separate datagram and bulk drain tasks per session: datagrams never `await` behind a
  flow-control-stalled 1 MB bulk write, so pose stays fresh while bulk saturates a shaped link.
- BBR congestion control: cubic builds a standing queue on a shaped link (bufferbloat) that delays
  every datagram by the queue depth; BBR keeps inflight ≈ BDP.

### VPS — run the dog (Ubuntu; needs `uv` + `rustup`; `deno` only to build the app)

Firewall: TCP `8080`; UDP `:8443` (WebTransport) + the WebRTC ICE range.
`sudo ufw allow 8080/tcp; sudo ufw allow 8443/udp; sudo ufw allow 32768:60999/udp`.

The simulated dog is heavier than `--extra web`: `go2-load` composes the smart `unitree_go2` (mapping +
navigation + perception), and dimsim launches a headless-Chromium renderer (Deno + Playwright Chromium,
auto-cloned `paul-nechifor/DimSim`). Install the sim stack, not just the web extra:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh                  # uv
GIT_LFS_SKIP_SMUDGE=1 git clone <your-dimos-remote> && cd dimos  # skip LFS blobs: ~176 MB instead of ~13 GB
uv sync --extra unitree --extra sim --extra mapping             # the dog stack (pin exact extras to your box;
                                                                # `--extra all` is the sledgehammer fallback)

cd dimos/web/dimoscope
cargo build --release --manifest-path gateway/wt-sidecar/Cargo.toml   # the WT sidecar, once (~2 min)

DIMOS_TRANSPORT=zenoh uv run python -m gateway &                 # :8080 — RPC backend must match the blueprints'
gateway/wt-sidecar/target/release/wt-sidecar &                   # owns QUIC/UDP :8443; /cert serves its hash (503 until up)
DIMOS_TRANSPORT=zenoh DIMSIM_RENDER=cpu \
  uv run dimos --simulation dimsim run go2-load &                # the dog, headless CPU render
```

`DIMSIM_RENDER=cpu` forces software rendering on a headless box (default `gpu`); dimsim needs `git` +
outbound internet on first run. The first RPC after a gateway start (e.g. the drawer's `Start load`)
can take ~10 s while zenoh establishes the `/rpc/*` routes — later calls are instant.

Light fallback if the sim is too heavy: `uv sync --extra web`, then `uv run dimos run load` (the
standalone `/load/*` source, keeps the `start_bench` crank) or `DIMOS_TRANSPORT=lcm uv run python
scenarios/bench.py` (ultra-light, env-tuned).

### Mac — point the browser at the VPS

```bash
deno task app     # Vite on :5173, served from your Mac
# open  http://localhost:5173/?gw=<VPS_IP>:8080          → the app (drive the dog over WAN)
# Topics tab → Benchmark → sweep each transport for real-path hz/latency/loss
```

WebTransport's self-signed cert is ECDSA ≤10 days; the service regenerates it and the client always
fetches the current hash from `/cert`.

### A physical Go2 over the internet

No code change: run the gateway on a machine next to the real dog (`uv sync --extra unitree`, robot
IP + per-device AES-128 key), point `go2-load` / `unitree_go2` at the hardware, forward `8080/tcp` +
`8443/udp`, and any browser drives it via `http://localhost:5173/?gw=<their-ip>:8080`. Same app, same
`SafetyEgress`.

### Environment reference (gateway + sidecar)

| env | default | what |
|---|---|---|
| `HOST` / `PORT` | `0.0.0.0` / `8080` | HTTP/WS bind — app + `/ws /sse /poll /rtc /media /cert /health /netem` |
| `WT_PORT` | `8443` | WebTransport QUIC/UDP port (read by the sidecar; the app assumes 8443) |
| `WT_PIPE` | `/tmp/dimoscope-wt.sock` | gateway↔sidecar unix socket (gateway listens, sidecar reconnects) |
| `WT_CERT_HASH_FILE` | `/tmp/dimoscope-wt-cert.hash` | sidecar writes its cert SHA-256 here; `/cert` serves it |
| `EGRESS_KBPS` | off | pace each client's egress so priority bites in the outbox, not the socket buffer |
| `QOS_RULES` | `qos.rules.json` | topic-glob → lane override map (see `qos.rules.example.json`) |
| `NETEM_CTL` | off | enable `/netem` (Linux + the `dimos-netem` sudo wrapper above) |
| `ZENOH_KEY` | `**` | zenoh key-expr the bus tap subscribes |
| `DIMOS_TRANSPORT` | platform default | the RPC bridge's backend — must match the blueprint's (topics tap both buses; teleop publishes to both; RPC is request/response and can't). `deno task serve` pins `zenoh`, same as every blueprint task |
| `DIMOS_LCM_HOST` / `DIMOS_LCM_PORT` | `239.255.76.67` / `7667` | LCM multicast group the tap joins |
| `STATIC_DIR` | `app/dist` | the built app served at `/` |

---

Sources: [ROS 2 QoS](https://design.ros2.org/articles/qos.html) ·
[Zenoh QoS](https://docs.rs/zenoh/latest/zenoh/qos/index.html) ·
[Foxglove ws-bridge](https://github.com/foxglove/ros-foxglove-bridge) ·
[Reactive Streams backpressure](https://reactivex.io/documentation/operators/backpressure.html)
