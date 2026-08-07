# Tools

Closed-loop episodes: referee world -> evolved planner -> controller -> matched sim.
Run from the repo root. Blue floor boxes in the viewer = the plan's expected body poses.

```bash
# list scenario names / watch one (curated or generated)
python -m dimos.navigation.motion.control --ls
python -m dimos.navigation.motion.control --view -s corridor
python -m dimos.navigation.motion.control --view --gen 8 -s gen003

# slow motion, receding horizon
python -m dimos.navigation.motion.control --view -s zigzag_room --speed 0.5 --replan-hz 5

# score the curated 16 (per-world rows + summary JSON)
python -m dimos.navigation.motion.control --score

# + N generated worlds / machine-readable
python -m dimos.navigation.motion.control --score --gen 8
python -m dimos.navigation.motion.control --score --json

# other planner, policy, or controller (candidates load as module:factory)
python -m dimos.navigation.motion.control --score --planner target-py
python -m dimos.navigation.motion.control --score --controller my.candidate:make
python -m dimos.navigation.motion.control --view -s slalom --policy ml-trajectory-research/freewalk_mcf.bin

# the rust laws (onboard build for the RK3588) -- build once, then run one
uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/Cargo.toml
python -m dimos.navigation.motion.control --score --controller seed-rs

# domain randomization (per-episode mechanism draws)
python -m dimos.navigation.motion.control --score --dr --seed 3

# the blind track: no clearance array, and --controller defaults to its law
python -m dimos.navigation.motion.control --score --blind
# ...against the seed on the same track, which is the A/B that shows the gain
python -m dimos.navigation.motion.control --score --blind --controller seed

# what the gait actually delivers (provenance for walk_gain/walk_slip)
python -m dimos.navigation.motion.control.referee.probe_walk_slip

# tests and types
python -m pytest dimos/navigation/motion/control -q
python -m mypy dimos/navigation/motion/control
cargo test --release --manifest-path dimos/navigation/motion/control/rust/Cargo.toml
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
different gait re-run `probe_walk_slip.py` and re-key before driving hardware.

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
