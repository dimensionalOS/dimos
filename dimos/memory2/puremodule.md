# Pure Modules

> New here? Start with the
> [gentle, runnable introduction](/docs/usage/pure_modules.md) — this page
> is the reference.

A `PureModule` separates a module into two declarations and one pure function:

- **when it runs** — one input marked `tick()` fires the ticks;
- **how every other input is sampled at that moment** — `latest()`,
  `interpolate()`, `window()`;
- **what it computes** — `step()`, a pure function of the aligned inputs
  (and, optionally, an explicit recurrent state).

Because `step` never touches ports, threads, or `self`-state, the same class
runs **live** on pubsub ports and **offline** over stored memory2 streams —
and can't tell the difference. That property is what buys replay,
time-travel debugging, restarts that resume where they left off, migration
across processes/machines, and parallel execution.

```python skip
from dimos.core.stream import In, Out
from dimos.memory2.puremodule import PureModule, tick, interpolate, latest

class Follower(PureModule):
    image: In[Image] = tick()              # 30 fps -> 30 ticks/s
    pose: In[PoseStamped] = interpolate()  # 50 Hz, slerped to frame time
    imu: In[Imu] = latest(max_age=0.1)     # newest, None if stale

    cmd_vel: Out[Twist]

    def step(self, image: Image, pose: PoseStamped, imu: Imu | None) -> Twist:
        return chase(image, pose)
```

## The alignment language

Sensors don't share a clock: cameras run at 30 fps, odometry at 50 Hz, IMUs
at 200 Hz. "Call the module with an image and *the pose at image time*"
requires a policy, and the policy is the whole declaration:

| Sampler | Value at tick time `t` | Delays the tick? |
|---|---|---|
| `tick()` | the observation that fired the tick | — (it *is* the tick) |
| `latest(max_age=None)` | newest obs with `ts <= t`; missing if older than `max_age` | never |
| `interpolate(tolerance=0.5)` | lerp/slerp between the obs bracketing `t` | live: until the next obs arrives (~one sample period) |
| `window(seconds)` | list of obs in `(t - seconds, t]` | never |

An `In` port with no sampler defaults to `latest()`. `interpolate()`
understands numbers, `Pose`, and `PoseStamped` (position lerp + quaternion
slerp; observation poses are interpolated too); other types degrade to
nearest-neighbor. When no bracket exists (stream ended, tick before the
first sample) it falls back to the nearest observation within `tolerance`.

Your "at what state do we call the module" examples translate as:

| Intent | Declaration |
|---|---|
| tick on every image, poses interpolated | `image = tick()`, `pose = interpolate()` |
| tick on every pose, image latest-or-None | `pose = tick()`, `image = latest()`, param `image: Image \| None` |
| tick on every image, all IMU since 100ms before | `imu = window(0.1)`, param `imu: list[Imu]` |

## Binding rules

`step` parameters bind to inputs **by name**; the annotation picks the shape:

- `Image` → `obs.data`; `Observation[Image]` → the full observation
  (`ts`, `pose`, `tags`);
- `X | None` → missing becomes `None`; missing on a non-optional
  parameter **drops the tick**;
- `window()` inputs: `list[X]` or `list[Observation[X]]`;
- reserved names: `ts: float` is the tick time; `state` (first parameter)
  makes the module a Mealy machine.

## Outputs

- **One `Out` port** — `step` returns the value; returning `None` emits
  nothing, so ticks double as filters.
- **Several `Out` ports** — return `{port_name: value}`. A *partial* dict
  is allowed (omitted ports stay quiet that tick — e.g. a command every
  tick, an alert occasionally); unknown keys raise `TypeError`.
- **No `Out` ports** — the return value is ignored.

Where outputs land differs by mode: **live**, every dict entry publishes
to its own port (and is appended to that port's stream in the module
store); **offline**, `over()` yields one observation per tick — the bare
value for single-output modules, the `{port: value}` dict for
multi-output ones (slice one output back out with
`.filter(lambda o: "alerts" in o.data).map_data(lambda o: o.data["alerts"])`).
That dict-row asymmetry is a known open point — the planned alternative
is a run handle exposing one store-backed stream per output.

## Offline: develop on recorded memory

Record a session (any storage-backed store), then iterate on the module in
a notebook — no LCM, no processes, deterministic:

```python skip
from dimos.memory2.store.sqlite import SqliteStore

db = SqliteStore(path="walk_2026_06_11.db")

out = Follower.over(image=db.streams.image, pose=db.streams.pose,
                    imu=db.streams.imu)

out.to_list()                                  # run it
out.map_data(lambda o: o.data.linear.x).to_list()  # poke at results
out.save(db.stream("cmd_vel_v2")).drain()      # or persist them
```

`over()` composes with the whole stream API — replay a slice with
`db.streams.image.after(t0).before(t1)`, downsample, quality-filter, etc.
Called on the class, `over` needs no module machinery at all
(`Follower.offline(some_config=...)` when the step reads `self.config`).
Don't pass `.live()` streams to `over()` — deploy the module for that.

Offline alignment is *exact*: events are merged in timestamp order, so a
run over the same recording produces the same ticks every time.

## Live: the same class on ports

Deployed in a blueprint, each `In` port feeds a memory2 stream in the
module's store and ticks run on a worker thread; outputs publish to the
`Out` ports. The store is a `NullStore` by default — inputs behave as
live-only streams (`.map`/`.transform`/`.live()` work; history and search
are empty). Override `make_store()` to return a `SqliteStore` and the
module records **every input and output** while it runs — recording is a
deployment choice, not module code. (This is the path to subsuming
`Recorder`: a recorder is a PureModule deployment with a storage-backed
store and no step.)

Live alignment is best-effort: observations are timestamped on arrival
(`msg.ts` when present) and slight cross-stream jitter is tolerated;
`interpolate()` inputs add one sample period of latency to each tick.

## Backpressure: the tick is the unit of load

The system has two regimes, and the store converts between them:

- **Pull (offline / stored streams)** — backpressure is intrinsic. The
  consumer's iteration is the clock; a chained pipeline computes one tick
  at a time and nothing accumulates beyond the (pruned) alignment buffers.
- **Push (live ports)** — sensors can't be paused, so backpressure is a
  declared *drop/coalesce policy*, and the natural unit is the tick:
  secondaries are cheap to ingest, all the expense is `step()`.

Live, resolved ticks flow through a `BackpressureBuffer` between the
alignment thread and the step thread:

```python skip
class Follower(PureModule):
    backpressure = KeepLast()    # default — controller semantics
    # backpressure = Unbounded() # recorder/indexer semantics: never drop
    # backpressure = Bounded(8)  # bounded queue, drops oldest
```

`KeepLast` means a slow step always processes the *freshest* tick and the
skipped ones are counted — for a 30 fps camera and a 100 ms step,
dropping ~2/3 of ticks is the system working as designed. Every queue in
the path is bounded: the tick buffer by policy, alignment buffers by
pruning, and ticks waiting for interpolation brackets by
`max_pending_ticks` (config, default 64) so a dead `interpolate()` input
can't accumulate ticks forever (evictions count as `drops_blocked`).

One honest consequence: with drops, a live run processes a *subsample* of
triggers, so replaying raw inputs offline (which processes all of them)
diverges for stateful modules. Exact replay-of-a-run requires recording
the resolved tick rows — designed next step, not built.

## Health: drops are metrics, not errors

Per-drop warnings at sensor rate are noise. The module follows the
mature ladder instead — count always, report continuously, log on
transitions, alert on *contracts*:

- **Counters** (always): ticks resolved/stepped, drops by reason
  (`backpressure`, `missing_input`, `blocked`), step p50/p99, per-input
  observed Hz, age of consumed `latest()` values, output rates.
- **`_health` stream**: an aggregated snapshot every
  `health_interval_s` (1 s) appended to the module store — live-only on a
  NullStore, *recorded next to the data it explains* on a SqliteStore, so
  a post-incident notebook plots drop ratio against the very frames that
  were dropped.
- **Contracts** are split deliberately: semantic tolerances live in the
  declaration (`latest(max_age=…)`, `interpolate(tolerance=…)`); *rates*
  live in deployment config (`expected_hz={"pose": 50}`,
  `min_output_hz=10`) because sim, replay, and the robot differ.
- **Messages**: one warmup line after `health_warmup_s` comparing
  observed input rates to expectations ("pose 12.1 Hz (expected 50 —
  LOW)"); one WARN on entering `DEGRADED`/`STALLED` naming the violated
  contracts; a throttled reminder every `unhealthy_log_every_s` while
  unhealthy; one INFO on recovery with the outage duration. Stalls
  (`STALLED`) distinguish "ticks queued but none stepped (step stuck?)"
  from "inputs flowing but no ticks resolving (interpolate input dead?)"
  and must persist `stall_after_s` before firing — a single slow step is
  not a stall.

The real SLO is output freshness and rate, not drop count — alert on the
contract, read the drop counters to diagnose *why*. Offline, `over()`
logs a per-field drop summary, and `_strict=True` raises on the first
drop instead (replay determinism tests should fail loudly).

## State, explicitly

If a module needs recurrence (gait phase, filters, RNN hidden state),
declare it — don't hide it in `self`:

```python skip
class GaitController(PureModule):
    pose: In[PoseStamped] = tick()
    cmd_vel: Out[Twist]

    initial_state = GaitState(phase=0.0)

    def step(self, state: GaitState, pose: PoseStamped) -> tuple[GaitState, Twist]:
        ...
        return new_state, twist
```

The runtime threads the state through the ticks (it's `scan` over the tick
stream). Because state is a value, snapshotting it per tick gives
time-rewind and live migration — the snapshot stream is the designed next
step, not yet built.

## Not yet designed / deliberately deferred

- `every(hz)` clock triggers and multi-input triggers (`on_any`) — only
  one `tick()` input for now.
- State snapshot streams (time-travel, suspend/revive, migration) — the
  Mealy form is the hook; persistence isn't wired yet.
- A live timeout policy for `interpolate()` when its input dies (currently
  ticks wait; on shutdown they resolve via the nearest-fallback).
- Modules that *query* memory (semantic search) — that's an impure
  capability and stays on `MemoryModule` for now.
- `Annotated[In[X], sampler]` syntax — rejected for now because core
  `Module` introspection doesn't unwrap `Annotated`; the default-value
  syntax is canonical.
