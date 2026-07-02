# dimoscope — Benchmark, QoS & Real-WAN

The quantitative side of dimoscope: the in-browser transport benchmark, the QoS priority story, and the
runbook for driving the dog over the real internet from a VPS.

The load source for all of it is the `go2-load` dog (`dimos/robot/benchmark/go2_load.py`): fixed
multi-rate `/load/{fast,mid,slow,grid,cloud}` lanes plus a crankable `/load/img` flood via
`start_bench`/`stop_bench` `@rpc`. The standalone `load` blueprint is the same module without the sim.

```bash
deno install && deno task build   # once — the app bundle the gateway serves at /
deno task serve                   # gateway + WT sidecar → http://localhost:8080 (+ QUIC :8443)
deno task load                    # /load/* lanes + the crankable flood (no sim; `deno task dog` = the full dimsim dog)
# open http://localhost:8080/ → Topics tab → Benchmark drawer
```

---

## 1. The benchmark — in the real browser, across transports

The Topics tab → Benchmark drawer measures the live transport end-to-end (publish→browser latency,
`recvTs − srcTs`) across the workload profiles, then lets you copy the results as Markdown. Pick a
transport from the topbar dropdown (WebSocket · SSE · HTTP poll · WebRTC data · WebTransport, or Auto),
crank the load generator up the ladder (`light 2 → camera 11 → dense 20 → depth-hd 50 → raw-1080p 180
→ firehose 300` MB/s on `/load/img`), and sweep. The `pose` profile measures the small `/load/{fast,mid,slow}`
lanes, which are always live; the heavy profiles measure `/load/img` at whatever the generator emits.
Running the `all-lanes` + `on-demand` pair in one sweep makes the Markdown export print the on-demand
saving: the WS-hop bandwidth cut of subscribing 1 lane vs all of them (the README's ~75%).

**Methodology** (`packages/web/src/bench.ts`): each scenario discards a warmup window (subscribe ramp-up
and QoS negotiation never pollute the stats), measures for `durMs`, then drains a grace window so a frame
that was merely delayed past the window counts as late, not lost. loss% is only what never arrived, and is
NaN on rate-limited runs where gaps are intentional. Rates divide by the wall clock actually elapsed, not
the nominal window. Before each sweep the client runs an NTP-style `{op:"ping"}` clock probe
(`client.estimateClockOffset()`, lowest-RTT of 5) and corrects `recvTs − srcTs` by the offset, so
end-to-end latency stays meaningful when the gateway is on another machine. Each result row is stamped
with the wire that actually carried it (Auto → `(wire: …)`, otherwise the WT→WS fallback is silent), and
the Markdown export carries the offered load, page origin, and UA.

### Results (localhost loopback · end-to-end · maxHz=∞ · 2026-07-02 · `31a1f92b7`)

Loopback numbers, no network loss. They show the relay/browser ceiling and whether bulk drags down the
small lanes — not WAN behavior (see §3). loss% marked `*` is a source-side seq floor (2–12%, varies per
run, identical on TCP where wire loss is impossible) — not transport loss. Unmarked bulk loss% is the
outbox conflating what the wire can't carry.

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

### Takeaways

- The small high-rate lanes stay crisp beside any flood: `pose` holds ~95–100 Hz at ~1 ms p50 on both
  transports with 20–180 MB/s of bulk running (3.4 ms at 300 MB/s offered). On WebTransport that's
  structural — pose rides QUIC datagrams, isolated from the bulk stream inside one connection.
- At robot-realistic bulk (≤20 MB/s) both transports carry the full dense flood at ~18 MB/s with 0%
  loss; WT-rs does it at p50 9.6 ms vs WS 21 ms. WT's bulk rides one persistent reliable uni-stream
  (big frames length-prefixed; small ≤1100 B frames go as datagrams — `gateway/wt-sidecar`).
- Past that the roles are clear: WT-rs moves 155 of 180 MB/s offered (p99 232 ms); WS tops out at
  ~40–45 MB/s of incompressible bulk and the outbox conflates the rest to the freshest frame (the
  loss% column — shedding by design, the tab stays interactive). WebTransport is the bulk workhorse;
  WebSocket is the universal control/fallback wire.
- History note: this table's previous revision said "WS relays ~255 MB/s". That was measured before
  `032a9f4cb` — the generators published all-zeros frames and WS permessage-deflate shrank them
  ~1000:1, so the wire carried kilobytes while the browser reported hundreds of MB/s. These are the
  first honest loopback WS numbers. Deflate still burns CPU on the now-random payloads; disabling it
  on `/ws` is an open follow-up that should raise the WS ceiling.
- WebTransport's other advantage — no HoL blocking under packet loss — doesn't show on loopback: with
  zero loss, TCP never stalls. Measure it on a real WAN (§3), or inject loss locally (Linux `tc qdisc
  add dev lo root netem loss 5%`; macOS `dnctl`/dummynet via `pfctl`).
- The Python relay comfortably carries robot-realistic bulk on either wire; past ~45 MB/s the Rust
  sidecar is the path. The tab survives the 300 MB/s firehose either way (conflation + backpressure —
  no hard crash).

Re-run any of this yourself: `deno task dog`, open the Benchmark drawer, set the generator tier, pick the
matching workload profile, **Run sweep**, **copy Markdown**. `?gw=host:port` points it at a remote VPS;
`?dur=ms` tunes the window.

---

## 2. QoS — declare importance, enforce it at the bottleneck

On a constrained browser link, declaring some topics more important and enforcing it at the gateway
keeps pose/teleop crisp while bulk (lidar/camera) degrades gracefully, instead of everything getting
equally janky.

Declaration → enforcement → transport:
1. Client declares a lane per topic (sane defaults, configurable) — `packages/web/src/qos.ts`.
2. Gateway enforces at the per-client egress — `gateway/qos.py`'s priority outbox, wired into
   `gateway/data.py`: under backpressure it drains high-priority topics first and conflates/drops the
   lowest-priority `best_effort` topics first, never the high-priority ones.
3. Transport reinforces (optional): put high-priority small topics on a no-HoL primitive (WebTransport
   datagrams) so they can't queue behind bulk within one connection.

The four lanes (auto-assigned by name/type via `defaultLane`, overridable per-topic; grounded in DDS / ROS 2):

| lane | reliability | priority | under load | typical topics |
|---|---|---|---|---|
| **command** | reliable, ordered | highest | never dropped | cmd_vel, goals, teleop |
| **sensor** | best-effort | high | latest-wins | pose, odom, imu, tf, joints |
| **default** | reliable | normal | bounded queue | general |
| **bulk** | best-effort | low | conflates / sheds first | lidar, camera, pointcloud, maps |

`gateway/qos.py` replaces the single FIFO with `{priority-class → {topic → slot}}`: `best_effort` topics
get a latest-only slot (a backed-up lidar overwrites itself, never grows a queue); `reliable` topics get a
bounded deque (DDS `keep_last`). The writer drains by weighted round-robin — high classes get most of the
budget, low classes keep a non-starving floor. Nothing novel here: Foxglove's ws-bridge keeps a bounded
per-client queue, Reactive Streams calls the conflation `onBackpressureLatest`, DDS/Zenoh have the
reliability + priority bands. The same patterns, applied at the browser link — the one boundary bus QoS
can't reach.

The A/B (pose @100 Hz light+high-priority · a heavy low-priority stream · link paced to ~4 Mbps via
`EGRESS_KBPS`), scheduler OFF (FIFO) vs ON (priority outbox):

| mode | pose hz | **pose p50** | pose p95 | heavy hz |
|---|--:|--:|--:|--:|
| **OFF** (FIFO) | 55 | **1294 ms** | 2419 ms | 120 |
| **ON** (priority outbox) | 99.5 | **4 ms** | 9 ms | 116 |

Pose latency collapses 1294 ms → 4 ms (~300×) and rate is restored (55 → 99.5 Hz) while the heavy stream
keeps flowing, conflated to its freshest frame. The catch: priority only bites if the backlog sits in the
gateway outbox. That's why the gateway paces each client's writer to its link budget (`EGRESS_KBPS`,
applied to both the `/ws` writer and the WebTransport drain). Without pacing, kernel and proxy socket
buffers absorb the backlog downstream and FIFO it — bufferbloat defeats any gateway QoS. Same discipline
as WebRTC's `bufferedAmount` watermark.

Lanes are defaults. A subscriber overrides any topic's QoS and it flows over the wire (the `subscribe` op
carries `priority`/`reliability`/`depth`, resolved client-side from `topic.setQos({lane})`); declaring
`pose=bulk` + `lidar=command` from the client flips which stream survives the saturated link. Operator
config (`qos.rules.json`, copy `qos.rules.example.json`) classifies custom per-blueprint topics by
name/type glob; the subscriber always wins over it.

Priority only matters under saturation — on a fat link both modes look identical, correctly.

---

## 3. Real-WAN — the dog over the internet from a VPS

Run the gateway + `go2-load` on a VPS with a public IP and drive it from your browser over the real path
(RTT, reorder, drop). The JS SDK doesn't change: the app + Benchmark drawer read `?gw=host:port`.

Serve the app from `http://localhost` (Mac Vite + `?gw=VPS`), not from `http://<public-IP>`: WebTransport
needs a secure context, and `localhost` qualifies where a bare public-IP HTTP origin doesn't (the
`WebTransport` API is simply `undefined` there, so the app silently uses WebSocket). Keeping the page on
`localhost` also lets `ws://`/`http://` reach the VPS IP without a mixed-content block.

### WAN results (2026-07-01, measured — Mac ⇄ a small public-IP VPS over the real internet)

Deploy: `go2-load` runs on the Linux VPS; `/load/*` streams over the real internet, and the full dog
renders headless — dimsim launches Chromium (`DIMSIM_RENDER=cpu`, SwiftShader software WebGL), publishes
`/color_image` + `/camera_info`, and the rendered scene shows in the browser over WAN. *(If `playwright install chromium` — which dimsim runs on
first launch — hangs, the usual cause is a broken IPv6 route to the CDN: fetch the browser + headless-shell
over **IPv4** (`wget -4 …chrome-linux64.zip` / `…chrome-headless-shell-linux64.zip`) into
`~/.cache/ms-playwright/chromium{,_headless_shell}-<build>/` with empty
`INSTALLATION_COMPLETE`+`DEPENDENCIES_VALIDATED` markers, or prefer IPv4 system-wide. Use **cam: jpeg**
over WAN — the webrtc cam mode needs ICE, which doesn't establish over a raw IP.)*
For a headless benchmark **without** the sim, the standalone `deno task load:flood` source is lighter.

### Network profiles — simulate deployment conditions from the browser

The BenchDrawer's Network section flips server-side `tc netem` profiles on the gateway host —
`clean · wifi-normal (10mbit/40ms/0.3%) · wifi-crowded (2mbit/100ms/1.5% bursty) · wifi-edge
(700kbit/200ms/5% bursty) · disaster (200kbit/500ms/8%) · loss-3/loss-5 (loss with no bw cap, the HoL
isolation tiers)` — plus momentary UDP/all **outage** buttons. Every result row and the copied Markdown are
stamped `net:<profile>`, so a table is self-describing across workload × transport × network condition.

Opt-in and scoped: install the root wrapper once with `deno task netem:install` (= `sudo install -m755
gateway/scripts/dimos-netem /usr/local/bin/` + a sudoers entry for `$USER` scoped to that one script),
then run the gateway with `NETEM_CTL=1`. Shaping hits **only** the gateway's egress ports
(8080/8443 via a prio+u32 band), so SSH and the rest of the box are untouched, and every apply self-heals
after 15 min. Apply loss **after** the transport is connected — QUIC handshakes fail under loss.

The browser panel is one of three equivalent controls — all drive the same wrapper and stay in sync
(the panel shows whatever is currently active):

```bash
sudo dimos-netem wifi-crowded    # on the gateway box — apply · `clean` clears · `status` prints
curl -X POST http://<gw>:8080/netem -H 'content-type: application/json' -d '{"profile":"loss-5"}'
curl http://<gw>:8080/netem      # from anywhere — the same endpoint the browser panel uses
```

Payloads are incompressible by design: the load generators publish random bytes, about the entropy of
real camera/lidar data. WebSocket negotiates permessage-deflate, which squeezes regular payloads ~1000:1,
so a compressible bench payload measures the compressor, not the transport — and asymmetrically, since
QUIC/WT has no equivalent. Sanity check on any run: the netem band byte counters must move ≈ the bytes
the browser reports.

### The profile matrix (2026-07-02 · 20 MB/s offered flood · clock-synced end-to-end latency)

WebTransport is served by the native WT-rs sidecar (`gateway/wt-sidecar/` — Rust,
[wtransport](https://github.com/BiagioFesta/wtransport)/quinn). It owns only the UDP `:8443` listener
(rows self-identify as `wire: dimoscope/WT-rs`); the Python gateway keeps the bus tap, `SafetyEgress`
and QoS config and feeds it over a unix socket (`gateway/pipe.py`). Teleop/goal/rpc flow browser →
sidecar → pipe → the same `SafetyEgress`; a dying session (or a dying sidecar) still zero-twists the
robot.

Both transports, measured back-to-back in one session:

| net profile | transport | pose hz / loss% / p50 / p99 (ms) | dense (1 MB frames) |
|---|---|---|---|
| clean | **WT-rs** | 97 / – / **18.0** / 46 | **19.1 MB/s** / 0% / **p50 37 ms** |
| clean | WS | 92 / – / 19.7 / 56 | 17.4 MB/s / p50 92 ms |
| wifi-normal | **WT-rs** | 109 / 2.3 / **45** / **61** | **975 kB/s** *(≈ the 10 mbit cap)* |
| wifi-normal | WS | 108 / 2.9 / 53 / 119 | 195 kB/s *(Mathis)* |
| wifi-crowded | **WT-rs** | 101 / 2.4 / 235 / 332 | 244 kB/s *(≈ the 2 mbit cap)* |
| wifi-crowded | WS | 105 / 1.0 / 90 / 126 | 1 frame / 4 s — and **0 msgs of anything** once bulk shares the pipe |
| **loss-5** | **WT-rs** | 106 / 4.9 / **41** / **54** | **9.3 MB/s** |
| **loss-5** | WS | 112 / 0 / 53 / **141** | **0** *(Mathis at 5% < one frame)* |

*(clean pose loss% omitted: a ~13% ingress-loss floor from the flood generator sits upstream of both
transports — the WS control shows the identical floor.)*

Reading the matrix:
- loss-5 is the HoL case: QUIC keeps both lanes alive at 5% random loss — pose p99 54 ms (datagrams,
  shedding 4.9%) and bulk 9.3 MB/s (per-stream retransmission). TCP delivers every pose frame but with a
  141 ms retransmit tail, and its bulk moves zero bytes. Freshness vs completeness: teleop/pose on
  datagrams, commands/bulk on reliable streams.
- TCP bulk dies by Mathis, not by the pipe. At wifi-normal the cap is 1.25 MB/s but TCP under 0.3%
  random loss yields ~195 kB/s; at 5% loss it can't move a single 1 MB frame. Random loss is poison for
  reliable bulk regardless of bandwidth. WT-rs tracks the shaped cap exactly (975 / 244 kB/s).
- Bulk is wire-limited on WT-rs: 19.1 MB/s of the 20 MB/s offered flood on the clean path, frame
  p50 37 ms, over the same single QUIC connection that carries pose at p50 18 ms.
- wifi-crowded is queue-dominated for every transport: pose-in-isolation rides the shaped queue
  (WT-rs p99 332 ms; TCP's in-order smoothing reads better at 126 ms) — but the moment bulk shares the
  connection, WS delivers nothing at all while WT-rs still tracks the cap. At this tier the fix is §2's
  rate-limits/conflation, not transport choice.
- UDP over raw IP remains finicky: QUIC handshakes fail under loss (connect first, then apply);
  WebRTC-data needs STUN/TURN; Auto falls back to WS when WT can't establish (the wire tag in each row
  shows which transport carried the run). Outage buttons (`UDP 3s/10s`, `all 10s`) exist for
  fallback/reconnect timing.

Recommendation: `Auto (WT→WS)` — WT-rs where UDP works, WebSocket everywhere else.

Two design constraints behind those numbers (`wt-sidecar/src/main.rs`):
- Separate datagram and bulk drain tasks per session: datagrams never `await` behind a
  flow-control-stalled 1 MB bulk write, so pose stays fresh while bulk saturates a shaped link.
- BBR congestion control: cubic builds a standing queue on a shaped link (bufferbloat) that delays
  every datagram by the queue depth; BBR keeps inflight ≈ BDP.

### VPS — run the dog (Ubuntu; needs `uv` + `rustup`; `deno` only to build the app)

Firewall — open the ports: TCP `8080`; UDP `:8443` (WebTransport) + the WebRTC ICE range.
`sudo ufw allow 8080/tcp; sudo ufw allow 8443/udp; sudo ufw allow 32768:60999/udp`.

The simulated dog is heavier than `--extra web`: `go2-load` composes the smart `unitree_go2` (mapping +
navigation + perception), and dimsim launches a headless-Chromium renderer (Deno + Playwright Chromium,
auto-cloned `paul-nechifor/DimSim`). Install the sim stack, not just the web extra:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh                 # uv
git clone <your-dimos-remote> && cd dimos                        # or cd your checkout; use the branch under test
uv sync --extra unitree --extra sim --extra mapping             # the dog stack (pin exact extras to your box;
                                                                # `--extra all` is the sledgehammer fallback)

cd dimos/web/dimoscope
cargo build --release --manifest-path gateway/wt-sidecar/Cargo.toml   # the WT sidecar, once (~2 min)

uv run python -m gateway &                                       # :8080 (app + /ws /sse /poll /rtc /media /cert)
gateway/wt-sidecar/target/release/wt-sidecar &                   # owns QUIC/UDP :8443; /cert serves its hash (503 until up)
DIMOS_TRANSPORT=zenoh DIMSIM_RENDER=cpu \
  uv run dimos --simulation dimsim run go2-load &                # the dog, headless CPU render
```

`DIMSIM_RENDER=cpu` forces software rendering on a headless box (default is `gpu`); dimsim always runs
`--headless` and needs `git` + outbound internet on first run (repo + Chromium download).

Light fallback if the full sim is too heavy on the box: `uv sync --extra web`, then either
`uv run dimos run load` (the standalone `/load/*` source, coordinator-wired, keeps the interactive
`start_bench` crank) or `DIMOS_TRANSPORT=lcm uv run python scenarios/bench.py` (ultra-light, env-tuned).
Both give a WAN benchmark with no dog viz.

### Mac — point the browser at the VPS (zero new code)

```bash
deno task app     # Vite on :5173, served from your Mac
# open  http://localhost:5173/?gw=<VPS_IP>:8080          → the app (drive the dog over WAN)
# Topics tab → Benchmark → sweep each transport for real-path hz/latency/loss
```

Keep the client page on `http://localhost` (so it may reach `ws://`/`http://` on the VPS IP without a
mixed-content block). WebTransport's self-signed cert is ECDSA ≤10 days — the service regenerates it and the
client always fetches the current hash from `/cert`.

### A physical Go2 over the internet

No code change: run the gateway on a machine *next to the real dog* (`uv sync --extra unitree`, robot IP +
per-device AES-128 key), point `go2-load` / `unitree_go2` at the hardware, forward `8080/tcp` + `8443/udp`
from that network, and any browser drives it via `http://localhost:5173/?gw=<their-ip>:8080`. Same app, same
`SafetyEgress` (velocity clamp + TTL deadman + stop-on-disconnect).

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
| `DIMOS_LCM_HOST` / `DIMOS_LCM_PORT` | `239.255.76.67` / `7667` | LCM multicast group the tap joins |
| `STATIC_DIR` | `app/dist` | the built app served at `/` |

---

Sources: [ROS 2 QoS](https://design.ros2.org/articles/qos.html) ·
[Zenoh QoS](https://docs.rs/zenoh/latest/zenoh/qos/index.html) ·
[Foxglove ws-bridge](https://github.com/foxglove/ros-foxglove-bridge) ·
[Reactive Streams backpressure](https://reactivex.io/documentation/operators/backpressure.html)
