# QoS under stress — proving the data-path prioritization is good tech

**The one claim this proves:** on a constrained browser link, *declaring* some topics more important than
others and *enforcing* it at the gateway keeps pose/teleop crisp while bulk (lidar/camera) degrades
gracefully — instead of everything getting equally janky. Run it:

```bash
deno task demo:qos                 # synthetic load (deterministic, no sim stack)
SIM=replay deno task demo:qos      # a real DimOS sim robot (also: mujoco, dimsim)
```

## The model: declaration → enforcement → transport

QoS is two things people conflate. **Declaration** (a topic's `reliability`/`durability`/`priority`) is
cheap and does nothing on its own. **Enforcement** is the hard part: importance only matters at a
**bottleneck**, where something must choose what to send and what to drop. For a browser, that bottleneck
is the **gateway's per-client outbound loop** — where the robot's fast bus meets the slow browser link.
On a fat LAN there's no contention, so priority never fires; it only matters (and is only testable) under
**saturation**.

So the chain is:

1. **Client declares** a lane per topic (sane defaults, configurable) — `packages/topics/src/qos.ts`.
2. **Gateway enforces** at the per-client egress — `servers/qos_sched.py`'s priority outbox, wired into
   `servers/data.py`. Under backpressure it drains high-priority topics first and **conflates/drops the
   lowest-priority `best_effort` topics first — never the high-priority ones.**
3. **Transport reinforces** (optional): put high-priority small topics on a no-head-of-line primitive
   (WebTransport datagrams), so they can't queue behind bulk even within one connection.

## The lanes (sane defaults, configurable) — grounded in DDS / ROS 2

ROS 2 ships *named QoS profiles* (SensorData, Default, Services) so you declare a sane default per topic
and override when needed. We mirror that with four lanes, auto-assigned by topic/type (`defaultLane`),
overridable per-topic:

| lane | reliability | priority | under load | typical topics |
|---|---|---|---|---|
| **command** | reliable, ordered | highest | never dropped | cmd_vel, goals, teleop |
| **sensor** | best-effort | high | latest-wins | pose, odom, imu, tf, joints |
| **default** | reliable | normal | bounded queue | general |
| **bulk** | best-effort | low | conflates / sheds first | lidar, camera, pointcloud, maps |

## How the enforcement works (the per-client priority outbox)

`servers/qos_sched.py` replaces the data plane's single FIFO queue with `{priority-class → {topic →
slot}}`. `best_effort` topics get a **latest-only slot** (a backed-up lidar overwrites itself — it never
grows a queue); `reliable` topics get a **bounded deque** (DDS `keep_last` depth). The writer drains by
**weighted round-robin**: high classes get most of the budget, low classes keep a floor so they never
starve to exactly zero. The shedding is explicit (bounded memory) — it doesn't rely on `send()` blocking.

**This is not novel — that's why it's clean.** It's the convergence of three established patterns:
- **Foxglove ws-bridge** — per-client bounded queue, drops oldest data-plane message under a slow client,
  separate control queue. (We add priority classes + conflation.)
- **Reactive Streams `onBackpressureLatest` / Kotlin `conflate`** — keep the latest, drop intermediates
  for sensor/UI data. (Our `best_effort` latest-slot, per topic = per key.)
- **DDS / Zenoh** — `reliability` + priority bands + `CongestionControl::Drop`. (We mirror it at the
  browser-link boundary the bus QoS can't reach.)

Refinements over a naive version: **DRR not strict priority** (no starvation), **explicit bounded outbox**
(don't depend on `send()` backpressure), **bound the reliable lane too** (`keep_last`).

## The A/B result

Same sim load (pose @ 100 Hz, light + high-priority · a ~2 MB/s lidar/img stream, heavy + low-priority),
the same constrained link, gateway scheduler **OFF (FIFO, today's baseline)** vs **ON (priority outbox)**.
From `deno task demo:qos` (synthetic, link paced to 4 Mbps):

| mode | pose hz | **pose p50** | pose p95 | lidar hz |
|---|--:|--:|--:|--:|
| **OFF** (FIFO) | 55 | **1294 ms** | 2419 ms | 120 |
| **ON** (priority outbox) | 99.5 | **4 ms** | 9 ms | 116 |

**Pose latency collapses 1294 ms → 4 ms (~300×) and its rate is restored to full (55 → 99.5 Hz) — while
the lidar keeps flowing (120 → 116 Hz, now conflated to its freshest frame).** OFF, the heavy stream fills
the per-client FIFO and pose queues ~1.3 s behind it; ON, pose drains first and the heavy stream conflates
to latest instead of building a stale backlog. *Important data survives a bad link* — and nothing sits
stale, because conflation keeps even the bulk lane fresh.

### The catch that makes it real: keep the queue AT the gateway

Gateway-egress priority only bites if the backlog actually sits in the gateway outbox. Our first attempt
shaped the link with an external TCP proxy (`netsim`) — and **ON ≈ OFF (both ~1700 ms)**, because the
kernel/proxy socket buffers (~256 KB ≈ 0.5 s) absorbed the backlog *downstream* of the outbox and FIFO-ed
it. The fix is **egress shaping**: pace each client's writer to its link budget (`EGRESS_KBPS`), so the
queue — and thus the priority decision — stays in the outbox. This is the same discipline as WebRTC's
`bufferedAmount` watermark (don't overfill the transport); without it, bufferbloat defeats any gateway
QoS. (A large single frame also head-of-line-blocks the writer, so bulk streams are fragmented — which is
why heavy data belongs on its own lane/transport, e.g. WebTransport streams.)

## Honest caveats
- Priority only matters under saturation — on a fat link both modes look identical (correctly).
- The loss-shedding is at the gateway egress (the browser-link bottleneck); the bus (Zenoh) does its own
  priority upstream, which this mirrors at the new boundary.
- `netsim` shapes the TCP byte stream; the per-client outbox is what reorders/sheds by priority *before*
  the bytes hit it. For a raw-drop UDP comparison see `bench/netsim-udp.ts` + `docs/benchmarks.md` §8.

Sources: [ROS 2 QoS](https://design.ros2.org/articles/qos.html) ·
[Zenoh QoS](https://docs.rs/zenoh/latest/zenoh/qos/index.html) ·
[Foxglove ws-bridge](https://github.com/foxglove/ros-foxglove-bridge) ·
[Reactive Streams backpressure](https://reactivex.io/documentation/operators/backpressure.html)
