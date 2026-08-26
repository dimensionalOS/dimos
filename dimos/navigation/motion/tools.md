# Tools

Three packages: `planner/` plans, `control/` follows a plan, `adapter/`
deploys both as dimos modules for the go2-zenoh blueprints. Run from the repo
root; per-package `tools.md` has the full menus.

Neither side's benchmark is on this branch: the planner's referee (worlds, gold
oracle, judge) and the control side's, which drives a MuJoCo plant, both stayed
on `ivan/feat/trajectory_ctrl` with the sim (see [README.md](README.md), "What
is not here"). What ships here is what runs on a robot, and the tests that hold
it: python-vs-rust parity on the laws, the crate's own behavioural invariants,
and the module wiring.

## planner

```bash
# the crate's behavioural invariants: routes an open world, refuses a sealed
# box, never hops a thin wall, same answer every call, no cross-call memo.
# --no-default-features drops pyo3, which the rlib these test does not need
# (plain `cargo test` runs the doc-tests and nothing else).
cargo test --release --no-default-features --test invariants \
    --manifest-path dimos/navigation/motion/planner/rust/Cargo.toml

# rebuild the extension after editing rust/
uv run maturin develop --uv --release -m dimos/navigation/motion/planner/rust/Cargo.toml
```

## control ([control/tools.md](control/tools.md))

```bash
# python law and rust law, tick for tick, over the whole sweep
uv run pytest dimos/navigation/motion/control

# rebuild the crate after editing rust/
uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/Cargo.toml
```

## adapter ([adapter/tools.md](adapter/tools.md))

```bash
# the loop on a real go2 (onboard laptop, robot runs the go2web zenoh bridge):
# raycaster -> MLS carrot -> local planner -> follower -> cmd_vel
dimos run go2-zenoh-motion

# bake the robot-side host (runs ON the robot, next to the go2web bridge);
# toolchain prereqs + deploy notes: docs/platforms/quadruped/go2/motion.md
dimos bake motion_planner trajectory_follower cmd_vel_mux go2_tf \
    -o motion-host --builder zigbuild --target aarch64-unknown-linux-gnu.2.31
```

Both batteries score gate-times-pillars: the planner against the SE(2) gold
oracle (`gate * (100 + 10 + 1)`, max 111) here, the controller against physics
on the sim branch — per track: blind `gate * (100 + 10 + 0.5 + 0.5)` (max 111, cruise 0.35 =
the stamp-encoding mid-band it may not exceed), hinted
`gate * (100 + 10 + 5 + 0.5)` (max 115.5, cruise 0.75 = the plant ceiling:
a speed gradient worth chasing, since the hint permits outrunning the
encoding).
