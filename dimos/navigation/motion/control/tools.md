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
uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/Cargo.toml
cargo test --release --manifest-path dimos/navigation/motion/control/rust/Cargo.toml

# on the robot, a track is chosen in the blueprint, not on a CLI:
#   TrajectoryFollower.blueprint(track="hinted")   go2-zenoh-motion
#   TrajectoryFollower.blueprint(track="blind")    go2-zenoh-motion-blind
dimos run go2-zenoh-motion
```

## Tracks and laws

A **track** is an input regime — what the follower is allowed to know.
`tracks.py` is the one place it is written down, and the blueprint and the
deployed adapter name a track rather than a law:

| track    | `update()` receives                        | law      |
|----------|--------------------------------------------|----------|
| `hinted` | pose + path + per-waypoint clearance array | `hinted` |
| `blind`  | pose + path (stamps still carry precision) | `blind`  |

Laws live one per module in `laws/` (`rust/src/laws/` for their ports), over
shared facilities: `geom` (arc length, progress, fan detection, carrot,
clearance governor) and `stamps` (the `profile.py` wire dialect). `seed` is the
permanent baseline — every A/B is against it and every search seeds from it, so
it does not absorb research results. A research generation lands by replacing
its own track's law and flipping one line in `TRACKS`.

`blind`: the gait slip inverse (`walk_command`), constant time headway, and the
governor read off the path stamps. `hinted`: a walk-threshold envelope (the
embodiment's `gait_band`), a tangent feedforward with foot correction in place
of the aim-at-the-carrot term, a brake-feasible preview, a pinch-escape leg,
and a self-rate-limited command.

Both tracks hit the same plant dead zone and solved it differently on purpose —
blind with an actuator inverse, hinted by moving its envelope — because the two
tracks are researched independently and sharing a mechanism biases the other's
search. Both calibrations are properties of the POLICY BLOB, not of a law
(`walk_gain`/`walk_slip`/`gait_band` on the embodiment): on a different gait
re-probe them and re-key before driving hardware.

`hinted` is the only law that keeps state (one tick of its own previous command,
so it can ramp at the plant's slew). It is a `#[pyclass]` on the rust side,
`reset()` must make a used instance indistinguishable from a fresh one, and its
parity is replayed as a sequence rather than asserted on a single tick.

Each `make_rust` law is a port of its python twin, not a variant:
`test_rust_parity.py` holds every pair to 1e-9 per twist component over 240
seeded cases (observed spread: 0, asserted separately).
