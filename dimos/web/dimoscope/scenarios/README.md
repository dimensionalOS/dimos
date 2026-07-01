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
scenario) and `dtop gen-types` types every topic with **0 untyped**.

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
discovery registry (`servers/bus.py` `Bus.topics`) never evicts, so topic **names** accumulate for the
life of the `serve.py` process, and WorldView's "first topic of a type" pick can latch onto a now-stale
name. For a pristine topic list / clean WorldView auto-pick, **restart `deno task serve`** between
scenarios (a fresh bus registry). The data-level switch needs no restart.

## Types — one typed map per blueprint

Each scenario generates its own typed topic map with the SDK's codegen (run the matching scenario first
so it's live on the bus). Note the gateway port: `serve.py` is `:8080`, so pass `--url`:

```bash
./scenarios/run.sh nav &                                             # publish /nav/* on the bus
deno task dtop gen-types --url ws://localhost:8080/ws --out app/src/topics/nav.gen.ts
```

Repeat with `arm` → `app/src/topics/arm.gen.ts`, `cam` → `app/src/topics/cam.gen.ts`. Each is a clean
`DimosTopics` map (0 untyped) — verified live:

```ts
"/nav/pose": geometry_msgs.PoseStamped;   "/arm/imu": sensor_msgs.Imu;
"/nav/cloud": sensor_msgs.PointCloud2;     "/arm/trajectory": trajectory_msgs.JointTrajectory;
"/cam/points": sensor_msgs.PointCloud2;    "/cam/detections": vision_msgs.Detection2DArray;
```

> **Coordination:** naming each map distinctly (`NavTopics`/`ArmTopics`/`CamTopics`) and binding them
> into the app hooks belongs to the typed-client work (a `--name` flag on `cli/genTypes.ts` + an
> `app/src/topics/index.ts` barrel). This scenario deliverable only produces the *inputs* — it doesn't
> edit the type pipeline. The disjoint namespaces mean the three maps union cleanly (no shared key).

## Benchmark them

`scenarios/bench.sh` measures each scenario's live topics over the WS transport and writes
`scenarios/RESULTS.md` (starts/stops only its own publisher processes):

```bash
bash scenarios/bench.sh          # → scenarios/RESULTS.md  (one row per scenario)
# Env: GW_URL=ws://localhost:8080/ws · DUR_MS=6000 · WARM_S=18 · DIMOS_TRANSPORT=zenoh
```

Run it against a freshly-started `serve.py` on a quiet bus for the cleanest numbers. Sample run (WS,
6 s/scenario) — the three profiles are clearly distinct:

| scenario | topics | msgs | hz | kB/s | p50 | p95 | p99 | loss% |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| **nav** | 4 | 528 | 88 | **1876** | 0.66 | 3.72 | 5.34 | 0 |
| **arm** | 4 | 3396 | **566** | 157 | 0.43 | 2.77 | 4.18 | 0.06 |
| **cam** | 4 | 433 | 72 | **26357** | 0.98 | 5.28 | 15.02 | 0 |

`nav` = size-mix (~1.9 MB/s), `arm` = message-rate ceiling (**566 msg/s**, tiny bytes), `cam` = bandwidth
(**26 MB/s**) — each stresses a different axis of the data path, at sub-ms–15 ms p50–p99 latency.

## Remote / VPS

The scenarios are just bus publishers, so streaming from a VPS needs no code change: run `serve.py` + one
scenario **on the VPS**, then open the app pointed at it —

```
http://localhost:5173/?gw=<vps-host>:8080
```

All five transports honor `?gw`; the page auto-uses `wss` when served over HTTPS. WebTransport
additionally needs the QUIC port (`WT_PORT`, default 8443) reachable + the `/cert` hash endpoint.

## Recommended QoS lanes (handoff to the QoS-rules map)

The scenario namespaces are exactly the "custom per-blueprint topics" the gateway's QoS rules classify.
Suggested `servers/qos.rules.json` entries (owned by the QoS work — this is a recommendation, not an edit):

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
scenarios/bench_scope.ts  measure one scenario's topics (reuses the SDK's measureScenario)
scenarios/bench.sh        benchmark all 3 → RESULTS.md
```

## Notes on message choices

- **`/arm/imu` (not a force/torque wrench):** `geometry_msgs.WrenchStamped` is a plain dataclass with no
  LCM wire codec, so it can't cross the bus. A wrist `sensor_msgs.Imu` is the canonical high-rate arm
  telemetry and encodes cleanly — same "tiny, extreme-rate" role.
- **PointCloud2** wraps Open3D, so `nav`/`cam` have a slightly slower cold start (the `bench.sh` warm-up
  accounts for it). Built once via `from_numpy`, then restamped per publish.
- **`/cam/detections`** carries only a stamped header (empty detection list) by design — it's the small
  "must survive the bulk" topic in the bandwidth scenario; the payload isn't the point.
- **`/arm/trajectory`** (`JointTrajectory`) has no `ts`/`frame_id` (it uses `.timestamp`), so the bench
  reports n/a latency/loss for it — it still streams and inspects.
