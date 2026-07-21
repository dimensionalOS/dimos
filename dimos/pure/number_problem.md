# The number problem: scalars out of pure modules → rerun graphs by default

Ivan (2026-07-21): "I want to easily output numbers from my modules, but they
should end up as Float32 or some message that rerun can render by default
(into graphs etc)." Design settled in-session; implementation NOT started.

## Grounding facts (verified)

- `dimos/msgs/std_msgs/` has thin LCM wrappers (`Int32`, `Int8`, `UInt32`,
  `Bool`, `Header`) — **no `Float32`/`Float64`**, and none have `to_rerun()`.
  `dimos_lcm.std_msgs` has the full ROS set incl. `Float32`/`Float64` wire types.
- `rr.Scalars` is already the house pattern for timeseries
  (`dimos/visualization/rerun/resource_stats.py`).
- `render_fields` (`dimos/pure/modules/rerun_tap.py`) logs every sink In field
  **at row ts** but only if the value `hasattr(value, "to_rerun")` — bare
  numeric fields are silently skipped. That's why `n_voxels: int` /
  `n_scans` / `n_loops` (already on VoxelMapper2/PGOVoxelMapper Out) render
  nowhere today.
- Plumbing rule: a number reaches a sink like anything else — exported from
  the graph Out, matched by name to a sink In port (`latest(default=None)`).
- LCM `Float32` has **no ts slot** (classic ROS gap). In-process this doesn't
  matter (row ts wins); live per-field egress it is the SAME standing issue as
  the `TwistUnstamp` "live rim egress needs ts" RATIFY item (see
  project memory / T13 Phase B notes) — fold there, don't fork a parallel fix.

## The design: two consumption paths, different machinery

### Path 1 — pure eval / sinks: bare numbers just plot (tiny, do first)

Module authors write `speed: float`, `n_voxels: int` on Out — no wrapping.
One new branch in `render_fields`:

```python
if value is not None and hasattr(value, "to_rerun"):
    rec.log(f"{entity_path}/{name}", row.ts, value.to_rerun())   # today
elif isinstance(value, (int, float)) and not isinstance(value, bool):
    rec.log(f"plots/{name}", row.ts, rr.Scalars(float(value)))    # NEW
```

- Log scalars under `plots/…`, NOT `world/…` — keep timeseries panels out of
  the 3D view.
- Instantly graphs the existing numeric fields from the PGO work once they're
  exported + given sink ports.

### Path 2 — live topics: `Float32`/`Float64` wrapper msgs

Live, every Out field is a topic and needs an LCM codec — a bare `float` port
cannot publish. Add `dimos/msgs/std_msgs/Float32.py` (+ `Float64.py`)
following the exact `Int32` wrapper pattern, plus
`to_rerun() -> rr.Scalars(float(self.data))`. Any renderer dispatching on
`to_rerun` then plots them by default; "publish a number" live is
`speed: Float32` on Out.

### Deferred (deliberately)

- **Rim auto-boxing**: bare `float` Out fields auto-wrapped into `Float32` at
  live egress so module code never mentions the wrapper. Nice sugar, but it
  sits behind the stamped-egress decision (TwistUnstamp RATIFY item) — do not
  build before that's settled.
- Legacy live renderer auto-subscribing scalar topics (rerun_config side) —
  separate follow-up.

## Tie-ins

- T15 debug recorder (`tasks/t15-debug.md`) and T9 health emit numbers
  (step_ms, drop counts, rates) — they ride Path 1's rendering for free.
- `api_improvements.md` holds the wider retrospective this came out of.

## Status / next actions

1. [ ] `render_fields` numeric branch (minutes; `rerun_tap.py`).
2. [ ] `Float32`/`Float64` wrappers with `to_rerun` (minutes; `std_msgs/`).
3. [ ] Park auto-boxing note with the TwistUnstamp stamped-egress item.

Recommendation was to land 1+2 immediately; Ivan had not yet said go when
this file was written.
