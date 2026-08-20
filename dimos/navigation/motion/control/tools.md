# Tools

The follower's laws, in python and in bit-exact rust. What is NOT here is the loop that
scores them: closed-loop episodes (referee world -> planner -> controller -> matched
MuJoCo sim) need the plant, so the `python -m ...motion.control` runner, `referee/` and
`research/` stayed on `ivan/feat/trajectory_ctrl` together with `simulation/`. Every
number quoted below was measured there; re-measure there.

Run from the repo root.

```bash
# python law vs rust law, tick for tick, over the whole seeded sweep
uv run pytest dimos/navigation/motion/control -q
uv run mypy dimos/navigation/motion/control

# the rust laws (this is also the onboard build for the RK3588)
uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/Cargo.toml
cargo test --release --manifest-path dimos/navigation/motion/control/rust/Cargo.toml

# on the robot, a track is chosen in the blueprint, not on a CLI:
#   TrajectoryFollower.blueprint(track="hinted")   go2-zenoh-motion
#   TrajectoryFollower.blueprint(track="blind")    go2-zenoh-motion-blind
dimos run go2-zenoh-motion
```

## Tracks and laws

A **track** is an input regime — what the follower is allowed to know, and the
pace it is held to. `tracks.py` is the one place both are written down, and the
CLI, the episode config, the judge and the deployed adapter all name a track
rather than a law:

| track    | eval              | `update()` receives                        | law      |
|----------|-------------------|--------------------------------------------|----------|
| `hinted` | `--score`         | pose + path + per-waypoint clearance array | `hinted` |
| `blind`  | `--score --blind` | pose + path (stamps still carry precision) | `blind`  |

Laws live one per module in `laws/` (`rust/src/laws/` for their ports), over
shared facilities: `geom` (arc length, progress, fan detection, carrot,
clearance governor) and `stamps` (the `profile.py` wire dialect). `seed` is the
permanent baseline — every A/B is against it and every autoresearch lab seeds
from it, so it does not absorb research results. A research generation lands by
replacing its own track's law and flipping one line in `TRACKS`.

`blind` is `motion-tc-autoresearch@blind_research01` (evo exp_0013): the gait
slip inverse (`walk_command`), constant time headway, and the governor read off
the path stamps. `hinted` is `@hint_research01` (evo exp_0045): a walk-threshold
envelope (0.45/0.95 commanded, carried by the law's own config default), a
tangent feedforward with foot correction in place of the aim-at-the-carrot term,
a brake-feasible preview, a pinch-escape leg, and a self-rate-limited command.

Both tracks hit the same plant dead zone and solved it differently on purpose —
blind with an actuator inverse, hinted by moving its envelope — because the two
labs are researched independently and sharing a mechanism biases the other's
search. Both calibrations are properties of the POLICY BLOB, not of a law: on a
different gait re-run the referee's `probe_walk_slip.py` (on the sim branch) and
re-key before driving hardware.

`hinted` is the only law that keeps state (one tick of its own previous command,
so it can ramp at the plant's slew). It is a `#[pyclass]` on the rust side,
`reset()` must make a used instance indistinguishable from a fresh one, and its
parity is replayed as a sequence rather than asserted on a single tick.

Each `*-rs` law is a port of its python twin, not a variant: `test_rust_parity.py`
holds every pair to 1e-9 per twist component over 240 seeded cases (observed
spread: 0, asserted separately).

Score per world = `gate * (100*arrived + 10*precision + 1*(pace+composure)/2)`,
max 111 (referee scale). Gate 0 on wall contact or fall; refusal counts as
arrival on `expect="refuse"` worlds. Precision is judged in clearance space:
the share of ticks the body spent under the embodiment's 0.05 m floor against
truth — deviation with room around it is free, the same deviation beside a
wall is the violation. Under replanning, cross-track is scored against the
plan that was ACTIVE at each tick (a replan never amnesties past drift) and
`churn` measures how far the follower forced the deterministic planner to
re-route -- plan flip-flop reads as churn, not as clean tracking. Columns:
min body clearance, below-floor fraction, cross-track p95 (diagnostic),
churn, tilt p99.
