# Scenario blueprints — 3 real-world data profiles, end-to-end

Three self-contained dimos publisher blueprints, each a **different real-world robot use-case** with a
**completely different topic namespace, message-type set, and data profile**. They exist to show the
whole dimoscope data path — discover → visualize → type → benchmark — working across *very* different
workloads, and to demonstrate **run-one / stop / run-another**: the gateway taps the bus, so the browser's
live topic set just follows whatever is publishing.

| scenario | namespace | topics (type · rate · size) | data-path axis | primary viewer |
|---|---|---|---|---|
| **scope-nav** — mobile navigation | `/nav/*` | `pose` PoseStamped·100 Hz·tiny · `path` Path·5 Hz · `cloud` PointCloud2·10 Hz·~200 KB · `map` OccupancyGrid·1 Hz·~40 KB | **size-mix** (the QoS-priority story) | WorldView 2D (arrow+trail, path, points, grid) |
| **scope-arm** — manipulation | `/arm/*` | `joint_states` JointState·250 Hz · `ee_pose` PoseStamped·100 Hz · `imu` Imu·500 Hz·tiny · `trajectory` JointTrajectory·2 Hz | **message-rate ceiling** (~850 msg/s) | Bench monitor (hz/latency) + ee_pose arrow |
| **scope-cam** — perception | `/cam/*` | `rgb` Image·30 Hz·~550 KB · `depth` Image·15 Hz · `points` PointCloud2·10 Hz·~1 MB · `detections` Detection2DArray·30 Hz | **bandwidth / bufferbloat** (the WebTransport story) | CameraView (rgb) + WorldView points |

Each reuses **standard `dimos.msgs` types**, so the app auto-renders them by *type* (no app code per
scenario) and the `packages/topics/scripts/genTypes.ts` codegen types every topic with **0 untyped**.

## Run it

Prereqs — a gateway + the app (two terminals, from `dimos/web/dimoscope`):

```bash
deno task serve      # the single Python service: bus tap + /ws + /media + /sse + /poll + /rtc + WebTransport
deno task app        # the Vite app → http://localhost:5173
```

Then run **one** scenario (a third terminal); Ctrl-C to stop, then run another:

```bash
./scenarios/run.sh nav      # → /nav/*   (mobile navigation)
./scenarios/run.sh arm      # → /arm/*   (manipulation)
./scenarios/run.sh cam      # → /cam/*   (perception)
```

Open `http://localhost:5173`. The sidebar discovers the live topics; **WorldView** (2D) draws the spatial
ones, **CameraView** shows `/cam/rgb`, and clicking any topic shows it as JSON in the **Inspector**. The
**Bench** tab plots per-topic hz/latency for anything (best for the high-rate `/arm/*` streams).

Transport is `DIMOS_TRANSPORT=zenoh` by default (matches `deno task serve`/`sim`); `DIMOS_TRANSPORT=lcm`
also works — the gateway taps both.

### Switching scenarios — what "just works" and one caveat

Stop a scenario and start another: the **data** switches live (the previous topics go silent, the new
ones flow — the Bench monitor, Inspector and CameraView follow immediately). **Caveat:** the gateway's
discovery registry (`gateway/bus.py` `Bus.topics`) never evicts, so topic **names** accumulate for the
life of the gateway process, and WorldView's "first topic of a type" pick can latch onto a now-stale
name. For a pristine topic list / clean WorldView auto-pick, **restart `deno task serve`** between
scenarios (a fresh bus registry). The data-level switch needs no restart.

## Types — one typed map per blueprint

Each scenario can be turned into its own typed topic map via the SDK's codegen (`generateTypes()` in
`packages/topics/scripts/genTypes.ts`), fed the topic/type pairs discovered while the matching scenario is live on the bus.
Each is a clean `DimosTopics` map (0 untyped) — verified live:

```ts
"/nav/pose": geometry_msgs.PoseStamped;   "/arm/imu": sensor_msgs.Imu;
"/nav/cloud": sensor_msgs.PointCloud2;     "/arm/trajectory": trajectory_msgs.JointTrajectory;
"/cam/points": sensor_msgs.PointCloud2;    "/cam/detections": vision_msgs.Detection2DArray;
```

> **Coordination:** naming each map distinctly (`NavTopics`/`ArmTopics`/`CamTopics`) and binding them
> into the app hooks belongs to the typed-client work (a `--name` flag on `packages/topics/scripts/genTypes.ts` + an
> `app/src/topics/index.ts` barrel). This scenario deliverable only produces the *inputs* — it doesn't
> edit the type pipeline. The disjoint namespaces mean the three maps union cleanly (no shared key).

## Benchmark them

Run a scenario, then open the app's **Bench** tab (or `/bench.html`) to measure its live topics across
transports — the three profiles are clearly distinct: `nav` = size-mix (~1.9 MB/s), `arm` = message-rate
ceiling (~566 msg/s, tiny bytes), `cam` = bandwidth (~26 MB/s), each stressing a different axis of the
data path. For the standard `/bench/*` synthetic load, use `deno task scope:bench` (see `bench.py`).

## Remote / VPS

The scenarios are just bus publishers, so streaming from a VPS needs no code change: run the gateway + one
scenario **on the VPS**, then open the app pointed at it —

```
http://localhost:5173/?gw=<vps-host>:8080
```

All five transports honor `?gw`; the page auto-uses `wss` when served over HTTPS. WebTransport
additionally needs the QUIC port (`WT_PORT`, default 8443) reachable + the `/cert` hash endpoint.

## Recommended QoS lanes (handoff to the QoS-rules map)

The scenario namespaces are exactly the "custom per-blueprint topics" the gateway's QoS rules classify.
Suggested `qos.rules.json` entries (owned by the QoS work — this is a recommendation, not an edit):

```jsonc
{
  "/nav/pose": "sensor",   "/arm/ee_pose": "sensor",   "/arm/imu": "sensor",
  "/arm/joint_states": "sensor",
  "/nav/map": "bulk",  "/nav/cloud": "bulk",  "/cam/rgb": "bulk",
  "/cam/depth": "bulk", "/cam/points": "bulk"
  // (command lane is for teleop/cmd_vel; default lane catches the rest, e.g. /nav/path, detections)
}
```

## Files

```
scenarios/common.py       shared scaffolding (transport helpers, seq/ts stamping, make_image, runner)
scenarios/nav.py|arm.py|cam.py   the 3 blueprints (Module + Out[T] streams + __main__ launcher)
scenarios/run.sh          launch one:  ./scenarios/run.sh <nav|arm|cam>
scenarios/bench.py        the /bench/* synthetic load source for the in-browser bench (deno task scope:bench)
```

## Notes on message choices

- **`/arm/imu` (not a force/torque wrench):** `geometry_msgs.WrenchStamped` is a plain dataclass with no
  LCM wire codec, so it can't cross the bus. A wrist `sensor_msgs.Imu` is the canonical high-rate arm
  telemetry and encodes cleanly — same "tiny, extreme-rate" role.
- **PointCloud2** wraps Open3D, so `nav`/`cam` have a slightly slower cold start (allow a brief warm-up
  before measuring). Built once via `from_numpy`, then restamped per publish.
- **`/cam/detections`** carries only a stamped header (empty detection list) by design — it's the small
  "must survive the bulk" topic in the bandwidth scenario; the payload isn't the point.
- **`/arm/trajectory`** (`JointTrajectory`) has no `ts`/`frame_id` (it uses `.timestamp`), so the bench
  reports n/a latency/loss for it — it still streams and inspects.
