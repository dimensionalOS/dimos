# Pure Modules

A traditional robotics module subscribes to topics, keeps state on `self`,
and publishes from callbacks. It works — but the interesting logic ends up
welded to live infrastructure: you can't run it on yesterday's recording,
you can't unit-test it without a pub/sub bus, and two runs over the same
data won't reproduce the same behavior.

A `PureModule` splits the same job into three declarations:

- **when it runs** — one input marked `tick()` fires the ticks;
- **how every other input is sampled at that moment** — `latest()`,
  `interpolate()`, `window()`;
- **what it computes** — `step()`, a pure function of the aligned inputs.

```diagon mode=GraphDAG
camera -> align
pose -> align
imu -> align
align -> tick
tick -> step
step -> outputs
```

```results
┌──────┐┌────┐┌───┐
│camera││pose││imu│
└┬─────┘└┬───┘└┬──┘
┌▽───────▽─────▽┐
│align          │
└┬──────────────┘
┌▽───┐
│tick│
└┬───┘
┌▽───┐
│step│
└┬───┘
┌▽──────┐
│outputs│
└───────┘
```

Because `step` never touches ports, threads, or `self`, the same class runs
**live** on pub/sub ports and **offline** over stored
[memory2 streams](/dimos/memory2/intro.md) — and cannot tell the
difference. Replay, time-travel debugging, migration, and parallel
execution stop being features you build into a module and become
properties of the runtime.

This page walks through the offline experience, which is also how you
develop: record once, then iterate in a plain Python session.

## A tiny robot, recorded

Let's fabricate two seconds of robot data — in real life this is a
`SqliteStore` recorded on the robot, here an in-memory store is enough.
The camera runs at 10 fps, the pose at 25 Hz (driving +x at exactly
1 m/s), and the IMU at 100 Hz:

```python session=pure ansi=false no-result
import logging
logging.disable(logging.CRITICAL)  # keep doc output clean

from dimos.memory2.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Pose import Pose

store = MemoryStore()
store.start()

camera = store.stream("camera", str)
pose = store.stream("pose", Pose)
imu = store.stream("imu", float)

for i in range(20):  # 10 fps, offset 5 ms from the pose clock
    camera.append(f"frame-{i:02d}", ts=0.005 + i * 0.1)

for i in range(51):  # 25 Hz, x = t because the robot drives 1 m/s
    t = i * 0.04
    pose.append(Pose(t, 0, 0, 0, 0, 0, 1), ts=t)

for i in range(201):  # 100 Hz
    imu.append(0.1 * (i % 5), ts=i * 0.01)
```

Notice the three streams don't share a clock — no two observations land on
the same timestamp. That's the reality pure modules are designed around.

## Your first pure module

The declaration reads top to bottom: tick on every camera frame,
interpolate the pose *to the frame's capture time*, batch all IMU samples
from the last 100 ms:

```python session=pure ansi=false
from dimos.core.stream import In, Out
from dimos.memory2.puremodule import PureModule, tick, interpolate, window

class Snapshot(PureModule):
    image: In[str] = tick()
    pose: In[Pose] = interpolate()
    imu: In[float] = window(0.1)

    described: Out[str]

    def step(self, image: str, pose: Pose, imu: list[float], ts: float) -> str:
        return f"{image} at x={pose.position.x:.3f}m with {len(imu)} imu samples"

out = Snapshot.over(image=camera, pose=pose, imu=imu)
for o in out.to_list()[:4]:
    print(f"t={o.ts:.3f}  {o.data}")
```

```results
t=0.005  frame-00 at x=0.005m with 1 imu samples
t=0.105  frame-01 at x=0.105m with 10 imu samples
t=0.205  frame-02 at x=0.205m with 10 imu samples
t=0.305  frame-03 at x=0.305m with 10 imu samples
```

Two things to notice:

- `step` parameters bind to the inputs **by name**, and the annotations
  are plain types — this is just a function. You can call
  `Snapshot.offline().step("f", Pose(1,0,0,0,0,0,1), [], 0.0)` in a test
  with no infrastructure at all.
- `x` equals the tick time exactly. The robot drives 1 m/s, so a pose
  *interpolated to the frame's capture time* must satisfy `x == t`. The
  25 Hz pose stream never sampled those instants — alignment built them.

`over()` returns a regular lazy stream: `.to_list()` runs it, `.save()`
persists it, filters slice it. Nothing executed until we asked.

## The sampler language

| Sampler                      | Value at tick time `t`                                     |
|------------------------------|------------------------------------------------------------|
| `tick()`                     | the observation that fired the tick                        |
| `latest(max_age=None)`       | newest observation with `ts <= t` (hold), missing if stale |
| `interpolate(tolerance=0.5)` | lerp/slerp between the observations bracketing `t`         |
| `window(seconds)`            | every observation in `(t - seconds, t]`, as a list         |

"At what state do we call the module" is always some combination of these.
Tick on poses with the latest image instead? Swap the samplers — the step
doesn't change shape:

```python session=pure ansi=false
from dimos.memory2.puremodule import latest

class PoseDriven(PureModule):
    pose: In[Pose] = tick()            # 25 ticks/s now
    image: In[str] = latest()          # most recent frame, whatever it is

    described: Out[str]

    def step(self, pose: Pose, image: str | None) -> str:
        return f"x={pose.position.x:.2f} sees {image or 'nothing yet'}"

rows = PoseDriven.over(pose=pose, image=camera).to_list()
print(rows[0].data)   # first tick precedes the first frame
print(rows[-1].data)
```

```results
x=0.00 sees nothing yet
x=2.00 sees frame-19
```

## When data is missing

The step's *type signature* is the policy. `image: str | None` above said
"missing is fine, give me None". A non-optional parameter instead **drops
the tick** — the module simply doesn't run without its required inputs:

```python session=pure ansi=false
gps = store.stream("gps", str)
for i in range(3):
    gps.append(f"fix-{i}", ts=i * 0.4)  # gps dies after t=0.8

class NeedsGps(PureModule):
    image: In[str] = tick()
    gps: In[str] = latest(max_age=0.5)  # a fix older than 0.5s is no fix

    described: Out[str]

    def step(self, image: str, gps: str) -> str:  # gps required
        return f"{image} located via {gps}"

located = NeedsGps.over(image=camera, gps=gps).to_list()
print(f"{located[-1].data}  <- last locatable frame")
print(f"{len(located)} of 20 frames located; the rest were dropped")
```

```results
frame-12 located via fix-2  <- last locatable frame
13 of 20 frames located; the rest were dropped
```

Offline, drops are summarized in a log line (and `over(_strict=True)`
raises instead — replay tests should fail loudly). Live, they're counted
per reason in the module's health stream.

## State, without `self`

Recurrent state (filters, gait phase, anything Kalman-shaped) is declared,
not hidden: name the first parameter `state` and return
`(new_state, output)`. Returning `None` as the output emits nothing — so
ticks double as filters:

```python session=pure ansi=false
class SpeedEstimator(PureModule):
    pose: In[Pose] = tick()
    speed: Out[float]

    initial_state = None  # (previous ts, previous x)

    def step(self, state, pose: Pose, ts: float):
        if state is None:
            return (ts, pose.position.x), None  # first tick: just remember
        prev_ts, prev_x = state
        v = (pose.position.x - prev_x) / (ts - prev_ts)
        return (ts, pose.position.x), v

speeds = SpeedEstimator.over(pose=pose)
values = [o.data for o in speeds.to_list()]
print(f"{len(values)} estimates, all {min(values):.2f}..{max(values):.2f} m/s")
```

```results
50 estimates, all 1.00..1.00 m/s
```

The runtime threads the state through the ticks. Because it's an explicit
value rather than attributes on `self`, a snapshot of it *is* the module's
full resume point — which is what makes restarts, migration, and
time-rewind mechanical.

## Multiple outputs

Declare several `Out` ports and return a dict keyed by port name. Emitting
a **subset** is allowed — each port fires independently, so a module can
publish a command every tick and an alert only sometimes:

```python session=pure ansi=false
class Navigator(PureModule):
    pose: In[Pose] = tick()

    cmd: Out[str]
    alerts: Out[str]

    def step(self, pose: Pose, ts: float) -> dict:
        outs = {"cmd": f"forward x={pose.position.x:.2f}"}
        if pose.position.x > 1.8:
            outs["alerts"] = f"approaching boundary at t={ts:.2f}"
        return outs

rows = Navigator.over(pose=pose).to_list()
fired = [r for r in rows if "alerts" in r.data]
print(f"{len(rows)} ticks; alerts on {len(fired)}; last: {rows[-1].data}")
```

```results
51 ticks; alerts on 5; last: {'cmd': 'forward x=2.00', 'alerts': 'approaching boundary at t=2.00'}
```

The rules: one `Out` port → return the bare value (`None` emits nothing);
several → return `{port: value}` (missing keys stay quiet, unknown keys
raise). Deployed live, each entry publishes to its own port. Offline,
`over()` yields one observation per tick whose data is the dict — slice a
single output back out with a map:

```python session=pure ansi=false
boundary_alerts = Navigator.over(pose=pose) \
    .filter(lambda o: "alerts" in o.data) \
    .map_data(lambda o: o.data["alerts"])
print(boundary_alerts.last().data)
```

```results
approaching boundary at t=2.00
```

(That dict-row shape is a known asymmetry with the live per-port streams —
a run handle with one stream per output is on the design table.)

## Modules compose like streams

A module's output stream is a normal memory2 stream, so modules chain into
pipelines — with stream operators slotting in between — and results save
back into the store next to the data they came from:

```python session=pure ansi=false
class SpeedAlert(PureModule):
    speed: In[float] = tick()
    alert: Out[str]

    def step(self, speed: float, ts: float) -> str | None:
        return f"speeding at t={ts:.2f}" if speed > 0.9 else None

alerts = SpeedAlert.over(speed=SpeedEstimator.over(pose=pose))

saved = store.stream("alerts", str)
alerts.save(saved).drain()
print(f"{saved.count()} alerts stored, e.g. {saved.last().data!r}")
```

```results
50 alerts stored, e.g. 'speeding at t=2.00'
```

The whole chain ran in one lazy pass: each pose tick flowed through
`SpeedEstimator`, became a speed, ticked `SpeedAlert`, and landed in the
store.

## The same class, live

Deployment is the part the module never sees. In a blueprint, each `In`
becomes a pub/sub port feeding a stream in the module's store, `step` runs
on a worker thread, and outputs publish to the `Out` ports:

```python skip
class Follower(PureModule):
    image: In[Image] = tick()
    pose: In[PoseStamped] = interpolate()
    cmd_vel: Out[Twist]

    backpressure = KeepLast()  # slow step? always process the freshest tick

    def step(self, image: Image, pose: PoseStamped) -> Twist:
        return chase(image, pose)

blueprint.add(Follower, expected_hz={"image": 30, "pose": 50}, min_output_hz=10)
```

Two deployment choices matter:

- **The store.** Default is a `NullStore` — inputs/outputs behave as
  live-only streams. Swap in a `SqliteStore` and the running module
  records every input and output as a side effect; develop against that
  recording with `over()` the next morning.
- **The backpressure policy.** `KeepLast()` (default) gives controller
  semantics: a slow step always sees the freshest tick and skipped ticks
  are *counted, not warned about* — for a 30 fps camera and a 100 ms
  step, dropping two thirds of ticks is the system working as designed.
  `Unbounded()` gives recorder semantics: never drop.

## Contracts, not log spam

Health is judged against declared contracts — input rates, output rate —
and reported as state transitions, not per-drop warnings. You never build
a monitor yourself: when a module starts, it constructs one from its own
config (`min_output_hz`, `expected_hz`, ...) and feeds it from the tick
loop; it lives at `module.health_monitor` and its snapshots land in the
`_health` stream. But since it's plain bookkeeping with an injectable
clock, we can hand-feed a detached one right here and watch the exact
messages a deployed `Follower` would produce — the first argument is just
the module name stamped on them:

```python session=pure ansi=false
from dimos.memory2.health import HealthMonitor

# what Follower's start() builds internally, driven by hand:
t = 0.0
monitor = HealthMonitor("follower", min_output_hz=10.0, warmup_s=0.0,
                        interval_s=1.0, clock=lambda: t)

for _ in range(3):  # a bad second: step emitted only 3 outputs vs the 10 Hz contract
    monitor.on_step(duration_s=0.12, ages={}, emitted=True)
t += 1.0
health = monitor.maybe_report()
print(health.state, "-", health.violations[0])

for _ in range(15):  # a good second
    monitor.on_step(duration_s=0.05, ages={}, emitted=True)
t += 1.0
print(monitor.maybe_report().state)
```

```results
DEGRADED - output 3.0 Hz < contract 10 Hz
OK
```

Deployed, those same snapshots append to a `_health` stream in the module
store every second (drop rates by reason, step p50/p99, input staleness),
transitions log once with the violated contract, and a recording captures
the health stream *next to the data it explains* — so a post-incident
notebook can plot the drop ratio against the very frames that were
dropped.

---

# Reference

The tutorial above shows the feel; this part states the exact rules.

## Declaration & binding

One input must carry `tick()`; an `In` port with no sampler defaults to
`latest()`. Input ports may not be named `ts` or `state` (reserved).
`step` parameters bind to inputs **by name**; the annotation picks the
shape:

- `Image` → `obs.data`; `Observation[Image]` → the full observation
  (`ts`, `pose`, `tags`);
- `X | None` → missing becomes `None`; missing on a non-optional
  parameter **drops the tick**;
- `window()` inputs: `list[X]` or `list[Observation[X]]`;
- reserved names: `ts: float` is the tick time; `state` (first parameter)
  makes the module a Mealy machine — `step(self, state, ...)` must return
  `(new_state, output)`, with the initial value from the `initial_state`
  attribute.

Declarations are validated at first use: a missing/duplicate `tick()`,
a `step` parameter that matches no input, or an
`Annotated[In[X], sampler]` port (unsupported — use the default-value
syntax) all raise `TypeError`.

## Alignment semantics

| Sampler                      | Value at tick time `t`                                     | Delays the tick?                                      |
|------------------------------|------------------------------------------------------------|-------------------------------------------------------|
| `tick()`                     | the observation that fired the tick                        | — (it *is* the tick)                                  |
| `latest(max_age=None)`       | newest obs with `ts <= t`; missing if older than `max_age` | never                                                 |
| `interpolate(tolerance=0.5)` | lerp/slerp between the obs bracketing `t`                  | live: until the next obs arrives (~one sample period) |
| `window(seconds)`            | list of obs in `(t - seconds, t]`                          | never                                                 |

`interpolate()` understands numbers, `Pose`, and `PoseStamped` (position
lerp + quaternion slerp; observation poses are interpolated too); other
types degrade to nearest-neighbor. When no bracket exists (stream ended,
tick before the first sample) it falls back to the nearest observation
within `tolerance`, else the value is missing.

**Offline is exact**: events are merged in timestamp order, so a run over
the same recording produces the same ticks every time. **Live is
best-effort**: observations are timestamped on arrival (`msg.ts` when
present), slight cross-stream jitter is tolerated, and `interpolate()`
inputs add one sample period of latency to each tick.

## Outputs

- **One `Out` port** — `step` returns the value; returning `None` emits
  nothing, so ticks double as filters.
- **Several `Out` ports** — return `{port_name: value}`. A *partial* dict
  is allowed (omitted ports stay quiet that tick); unknown keys raise
  `TypeError`.
- **No `Out` ports** — the return value is ignored.

Live, every dict entry publishes to its own port (and is appended to that
port's stream in the module store). Offline, `over()` yields one
observation per tick — the bare value for single-output modules, the
`{port: value}` dict for multi-output ones. Output observations derive
from the trigger observation (its `ts`, `pose`, `tags`).

## Running offline: `over()`

Pass one stream per declared input, by name; each must iterate in
ascending `ts` (stored streams in insertion order do; otherwise prepend
`.order_by("ts")`). Don't pass `.live()` streams — deploy the module for
that. Called on the class, `over` builds a machinery-free instance via
`offline()` (pass config there: `Follower.offline(gain=2.0).over(...)`).
Drops are summarized in a log line; `over(_strict=True)` raises on the
first tick dropped for missing required inputs — replay determinism tests
should fail loudly.

## Running live: deployment knobs

The store: `NullStore` by default — inputs/outputs behave as live-only
streams (`.map`/`.transform`/`.live()` work; history and search are
empty). Override `make_store()` to return a `SqliteStore` and the module
records **every input and output** while it runs — recording is a
deployment choice, not module code.

Backpressure: live, resolved ticks flow through a `BackpressureBuffer`
between the alignment thread and the step thread —

| Policy | Semantics |
|---|---|
| `KeepLast()` (default) | controller: always step the freshest tick, count the skipped |
| `Unbounded()` | recorder/indexer: never drop, memory-bounded only by consumption |
| `Bounded(n)` / `DropNew(n)` | bounded queue dropping oldest / rejecting newest |

Every queue in the path is bounded: the tick buffer by policy, alignment
buffers by pruning, and ticks waiting for interpolation brackets by
`max_pending_ticks` (config, default 64) so a dead `interpolate()` input
can't accumulate ticks forever (evictions count as `drops_blocked`).

Health: counters always (ticks resolved/stepped, drops by reason, step
p50/p99, per-input Hz, ages of consumed `latest()` values, output rates);
a `_health` stream snapshot every `health_interval_s`; one warmup line
comparing observed input rates to `expected_hz`; transition-logged
`DEGRADED`/`STALLED` with the violated contracts, throttled reminders
every `unhealthy_log_every_s`, recovery logged with duration. Stalls must
persist `stall_after_s` and distinguish "ticks queued but none stepped
(step stuck?)" from "inputs flowing but no ticks resolving (interpolate
input dead?)". Contracts split deliberately: semantic tolerances live in
the declaration (`max_age`, `tolerance`); *rates* live in deployment
config (`expected_hz`, `min_output_hz`) because sim, replay, and the
robot differ. The real SLO is output freshness and rate — alert on the
contract, read drop counters to diagnose why.

## Where next

- [Design notes](/dimos/memory2/puremodule.md) — the why behind
  backpressure and health, replay fidelity under drops, the state
  persistence plan, and the deferred list.
- [Memory intro](/dimos/memory2/intro.md) — the stream API `over()`
  returns.
- [Temporal alignment](/docs/usage/data_streams/temporal_alignment.md) —
  the rx-level alignment this generalizes.
