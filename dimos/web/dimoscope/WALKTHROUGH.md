# dimoscope — guided walkthrough (learn dimos by reading this build)

This is the teaching companion. Read it top to bottom, run the commands, and
open the referenced files. Each section maps to a **concept** and a **curriculum
rung**. By the end you'll understand how dimos puts data on a bus and how a web
client consumes it.

---

## The one idea that makes everything click

dimos is a **bus of typed streams**. Modules publish/subscribe **messages** on
**topics** over **LCM** (UDP multicast). A message on the wire is:

```
[ "/odom#geometry_msgs.PoseStamped" \0 ][ 8-byte type-hash ][ fields… ]
        topic # type                        ↑ self-describing
```

Because the 8-byte hash identifies the type, a client can decode **any** topic
it has never seen — that's what makes a *generic* viewer possible. `@dimos/msgs`
holds a hash→class registry and does this for us in the browser.

The browser can't speak UDP multicast, so we need **one** native process to
relay bus↔browser. That's the only "hard" part, and it's ~50 lines.

---

## Rung 1 — Module, Stream, Transport, the bus  ·  `bridge.ts`

Read `bridge.ts`. The realization that shaped the whole project: the dimos
TS example uses `@dimos/lcm`, but it only uses it to *receive* and *send* UDP
packets. `node:dgram` does multicast UDP in Deno (and Node), and **all** LCM
framing happens in `@dimos/msgs` — so the bridge is a **dumb byte relay** that
never parses a packet, and needs no `@dimos/lcm` at all.

- bus → browser: `udp.on("message", buf => clients.forEach(ws => ws.send(buf)))`
- browser → bus: `message(ws, m) => udp.send(m)`

> **LeRobot bridge:** a dimos *Module* (`In[T]`/`Out[T]` ports) is the unit of
> computation, like a LeRobot `Robot` with `get_observation`/`send_action` —
> except dimos is async pub/sub over a bus, not a synchronous Python loop.

**Run & observe:**
```bash
deno install && deno task bridge      # [lcm] listening … [ws] serving ws://localhost:8080
```

---

## Rung 2 — the wire format & decode-by-hash  ·  `app/src/bus.ts`

The real robot is `examples/simplerobot/simplerobot.py` (a dimos Module). It
publishes `geometry_msgs/PoseStamped` on channel `/odom#geometry_msgs.PoseStamped`
over LCM — the type suffix is the convention dimos uses on the wire.

`app/src/bus.ts` is the consumer core: for each packet it calls
`decodePacket()` → `{ channel, data }`, splits `channel` on `#` to learn the
**topic** and **type** (topic discovery, no config), and routes the decoded
object to subscribers. The 8-byte type hash inside the payload is what lets
`@dimos/msgs` decode a topic it's never seen.

**Run & observe (headless, no browser):**
```bash
.venv/bin/python examples/simplerobot/simplerobot.py --headless   # real dimos robot
deno task bridge                                                  # the relay
deno task wsprobe                                                 # prints /odom seen in 3s
```
This is exactly how the pipe was verified against real dimos. ✅

## Bridge note — LCM fragmentation

dimos fragments any LCM message over ~1.4KB into multiple `LC03` datagrams.
`@dimos/msgs.decodePacket` only handles single `LC02` packets, so `bridge.ts`
**reassembles fragments** (keyed by LCM sequence number) into a synthetic `LC02`
packet before forwarding — the browser is unchanged. This is why `/map`
(OccupancyGrid, ~3.7KB → 3 fragments) shows up at all.

---

## Rung 3 — visualize a stream  ·  `app/src/widgets/WorldView.tsx`

`WorldView` is one canvas fusing three topics: `/map` (grey cells), `/scan`
(cyan lidar points), `/odom` (yellow robot + blue trail). It reads plain
decoded JS objects — note `quatToYaw()` (quaternion → heading) and the
world-meters → screen-pixels transform. The OccupancyGrid is row-major
`data[row*width + col]`; cell world-position = `origin + (col+0.5)*res, …`.

**React glue:** `app/src/useBus.tsx` exposes `useTopic("/odom")` →
latest decoded message; the widget re-renders as data flows.

---

## Rung 4 — close the loop (publish)  ·  `app/src/widgets/TeleopPad.tsx`

`bus.publishTwist(lin, ang)` builds a `geometry_msgs.Twist`, encodes it to
`/cmd_vel`, and `ws.send`s it. The bridge drops it on the bus; real
`simplerobot.py` receives it and drives. Bidirectional streams in ~10 lines.

> **LeRobot bridge:** this is teleoperation — conceptually `lerobot-teleoperate`
> sending actions, but as a published ROS message instead of a function call.

---

## Rung 5 — author your OWN Module  ·  `examples/fakesensors.py`

This is the real dimos payoff: `examples/fakesensors.py` (repo root) is a hand-written dimos
`Module` with `odom: In[PoseStamped]` and `map: Out[OccupancyGrid]`. It's run
**standalone** by assigning `LCMTransport` by hand — the same low-friction
pattern as `examples/simplerobot/simplerobot.py` (no Blueprint required).
`LCMTransport("/map", OccupancyGrid)` auto-produces the channel
`/map#nav_msgs.OccupancyGrid` the browser already understands.

> Needs the dimos env (`uv venv --python 3.12 && uv pip install dimos`). The next
> rung up is composing modules with `autoconnect(...)` in a Blueprint — see the plan.
> (Note: importing `OccupancyGrid` pulls `matplotlib`, which builds a font cache on
> first run — one-time, ~seconds; run with `PYTHONUNBUFFERED=1` to see prints.)

---

## Rung 6 — the generic framework  ·  `app/src/widgets/registry.ts`, `App.tsx`

`registry.ts` maps a **message type → widget** (`PoseStamped → PoseReadout`,
everything else → `JsonInspector`). `App.tsx` lists discovered topics and opens
the right widget per type. Adding support for a new message type is **one line**
in the registry — that's the difference between a demo and a *framework*.

---

## What's verified vs. what's next

**Verified against REAL dimos on this machine (no Deno, no mocks):**
- `dimos` wheel installs (`uv pip install dimos`) and `dimos.core`/`dimos.msgs` import — no torch/CUDA (✅)
- real `simplerobot.py` `/odom` → bridge → WebSocket → `@dimos/msgs` decode (wsprobe ✅)
- real `fakesensors.py` `/map` (OccupancyGrid, fragmented) → reassembled by the bridge → decoded (✅)
- teleop reverse path: browser `/cmd_vel` → bridge → bus → simplerobot moves (✅)
- the React app type-checks (`tsc --noEmit` ✅) and builds (`vite build` ✅)

**Left / next sessions:**
- **Replay / nav stack:** `dimos --replay run unitree-go2` needs the heavy stack
  (`dimos[unitree]` → torch/open3d/transformers, multi-GB; smart stack imports torch
  via `memory2`). Deferred deliberately to keep things simple.
- **Rung 7 (stretch):** MuJoCo `unitree-g1-basic-sim` (mjpython + model download);
  camera/full-lidar are bigger fragmented messages — reassembly already handles them.
- **Rung 8 (stretch):** an agent panel (Skills/MCP) — the layer LeRobot has no
  equivalent for.
