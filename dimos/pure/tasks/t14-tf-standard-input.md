# T14 — tf as a standard input (retire the TF service for pure modules)

Status: spec, not started. Owner: unassigned.
Prereq context: T8 (`t8-rim.md` §9 tf side channel), T11 tf buffer (`dimos/pure/tfbuffer.py`).

## The bug that motivates this

In a coordinator deployment, a pure module with a **required** `pm.tf()` port drops
**every tick, silently**. Found 2026-07-20 chasing "clicking doesn't work" in
`go2_nav --replay`:

- `Planner.position: Transform = pm.tf("{frame_id}", "{body_frame}")` — required.
- `Aligner._fire` resolves tf via `TfContext`; unresolved required chain →
  `stats.ticks_dropped += 1; return None`. No log, no error, no emission.
- So the planner never emits a path, and no click can ever matter.

Why it's unresolvable: `rim._build_tf_context` (rim.py ~line 730) only builds a tf
**feed** when `m.i.tf` has been bound:

```python
tf_port = ports_of(module).i._ports.get("tf")  # created only if m.i.tf touched
if tf_port is not None and (transport is not None or source is not None): ...
```

but `legacy.py`'s `start()` only iterates `spec.in_type.fields()` — the *bundle*
fields. `tf` is a synthetic stream port, not a bundle field, so the bridge never
binds it. Result: `feed=None`, a fresh empty `TfBuffer`, nothing ever ingested.

Contrast — why `--pure` works: `NavStack.over()` wires `odom_tf.tf` in-graph, so the
buffer is fed. Only the **bridge/coordinator** path is broken.

Also explains the whole observed pattern:

| module | tf in port | coordinator result |
|---|---|---|
| `VoxelMapper2.pose` | `pm.tf(..., default=None)` optional | resolves None, scans insert untransformed → map works |
| `PureCostMapper` | none | works |
| `Planner.position` | `pm.tf(...)` **required** | every tick dropped → no path, ever |

## The direction (Ivan, 2026-07-20)

> "tf service needs to totally go, I think we should process TFMessage ourselves as
> an actual standard input"

Pure modules should consume tf as an **ordinary subscribed stream of `TFMessage`** —
no `PubSubTF`/`LCMTF` service, no `receive_msg` callback rail, no second buffer.
The pure side already supports this: `TfBuffer.ingest` accepts a `TFMessage`
(iterates `.transforms`) or a single `Transform` (`tfbuffer.py` ~line 287).

## Scope

1. **Bridge binds `m.i.tf`** (`dimos/pure/legacy.py`). When the pure `In` declares
   any `TfSpec` field, the generated actor must expose a `TFMessage` input and bind
   it: `self._pure.i.tf.transport = <that stream>`. Note `tf` is reserved on the
   `Module` surface (`Module.tf` property, and legacy.py's `_MODULE_SURFACE` guard),
   so the actor-side attribute needs a different name (e.g. `tf_in`) pinned to the
   tf topic.
2. **Topic/type**: `TFMessage` on the active backend's tf topic — LCM `/tf`
   (`LCMPubsubConfig`), Zenoh `dimos/tf` (`ZenohPubsubConfig`), see
   `dimos/protocol/tf/tf.py` ~lines 439-457, backend chosen by
   `transport_factory.tf_backend()`. Prefer resolving the topic from the same
   config rather than hardcoding, so the backend switch keeps working.
3. **tf_out stays as is** for now — `legacy.py` already asserts `tf_out()` ports
   onto the TF topic (see its comment ~line 207). Ensure in/out agree on topic+type.
4. **Retire the service for the pure path** — pure modules must not need
   `Module.tf`/`PubSubTF` at all. Full removal of the TF service across legacy
   modules is out of scope here; this task removes the *pure* dependency on it.

## Acceptance

- `python -m dimos.pure.modules.go2_nav --replay go2_hongkong_office` renders a
  **path**, and clicking a goal in the viewer re-plans. (Costmap already works.)
- A required `pm.tf()` port resolves in a coordinator deployment; add a test that
  fails on today's code (a bridged module with a required tf port emits rows).
- `--pure` behavior unchanged; `dimos/pure` suite stays green.

## Related, do not conflate

- **Silent tick drops.** A module dropping 100% of ticks on an unresolved required
  field logs nothing (`stats.ticks_dropped` only). This hid the bug for hours — same
  family as two other invisibility bugs fixed 2026-07-20 (swallowed worker loggers;
  dangling subscriptions). Worth a warning when a module has dropped N consecutive
  ticks with none emitted. Small, separable, high value.
- **`control=True`** (landed 2026-07-20): `pm.latest(default=None, control=True)` on
  `Planner.goal_point` re-stamps out-of-band control input to the tick frontier —
  a browser-stamped click is ~75 days ahead of replay data-time and would otherwise
  never resolve. Independent of this task; both are needed for clicking to work.
