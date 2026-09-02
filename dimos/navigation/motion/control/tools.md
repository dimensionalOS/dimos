# Tools

The follower's laws, in python and in bit-exact rust. The closed-loop scoring
(episodes against the matched MuJoCo plant) is not on this branch; every number
quoted below was measured there.

Run from the repo root.

```bash
# python law vs rust law, tick for tick, over the whole seeded sweep
uv run pytest dimos/navigation/motion/control -q
uv run mypy dimos/navigation/motion/control

# the rust laws (this is also the onboard build for the RK3588)
uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/py/Cargo.toml
cargo test --release -p dimos-motion2-tc

# on the robot the follower runs the same law (adapter/rust/src/follower.rs)
dimos run go2-zenoh-motion
```

## Laws

The follower runs `hinted` -- the default of `TrajectoryFollowerConfig.controller`,
and the one law the native twin has. `seed` is the permanent baseline: every A/B
is against it and every search seeds from it, so it does not absorb research
results. A research generation lands by replacing `laws/hinted.py` and its port.

Laws live one per module in `laws/` (`rust/src/laws/` for their ports), over
shared facilities: `geom` (arc length, progress, fan detection, carrot,
clearance governor) and `stamps` (the `profile.py` wire dialect).

`hinted`: the gait slip inverse (`walk_command`), constant time headway, and
the governor read off the path stamps. Its `clearance` argument is the lab's
channel -- an explicit room array outranks the stamps -- and the follower never
hands one; on the robot the stamps are the only room the law sees.

The slip calibration is a property of the POLICY BLOB, not of the law
(`walk_gain`/`walk_slip` on the embodiment): on a different gait re-probe it
and re-key before driving hardware.

Neither law keeps state; a stateful generation would be a `#[pyclass]` on the
rust side with a `reset()` that makes a used instance indistinguishable from a
fresh one, and its parity replayed as a sequence rather than asserted on a
single tick.

Each `make_rust` law is a port of its python twin, not a variant:
`test_rust_parity.py` holds every pair to 1e-9 per twist component over 240
seeded cases (observed spread: 0, asserted separately).
