# Tools

Three packages, one loop: `planner/` plans, `simulation/` is the matched Go2
MuJoCo env, `control/` executes plans in it and judges the execution.
`adapter/` deploys the loop: planner + follower as dimos modules for the
go2-zenoh blueprints. Run from the repo root; per-package `tools.md` has the
full menus.

`planner/` and `control/` are each a runner (`python -m ...planner` /
`...control`), a `referee/` that scores whatever the runner picked, and
`research/{auto,ml}` where candidates come from. Any candidate is a
`module:factory` string, so all of them go through the same judge:

```bash
python -m dimos.navigation.motion.planner --score --planner gold
python -m dimos.navigation.motion.control --score --controller seed
python -m dimos.navigation.motion.control --score --controller my.candidate:make
```

## planner ([planner/referee/README.md](planner/referee/README.md))

```bash
# score the evolved rust planner: curated 16, or the full 56-world battery
python -m dimos.navigation.motion.planner --score
python -m dimos.navigation.motion.planner --score --gen 40 --jobs 8

# watch a plan in rerun (2D referee view: truth, cloud, sweep, gold,
# required-precision circles along the candidate path)
python -m dimos.navigation.motion.planner --spawn --score -s corridor_reverse

# rebuild the crate after editing rust/
uv run maturin develop --uv --release -m dimos/navigation/motion/planner/rust/Cargo.toml
```

## simulation ([simulation/tools.md](simulation/tools.md))

```bash
# fitted sim next to the recorded robot (green ghost box)
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --view --ghost --fitted --start 6

# score sim vs recording (noise-floor units)
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --eval --fitted
```

## control ([control/tools.md](control/tools.md))

```bash
# watch the go2 execute a plan (blue floor boxes = expected body poses)
python -m dimos.navigation.motion.control --view -s corridor

# judge the controller on the curated 16 (+ generated with --gen N)
python -m dimos.navigation.motion.control --score          # hinted track
python -m dimos.navigation.motion.control --score --blind  # blind track + its law
```

## adapter ([adapter/tools.md](adapter/tools.md))

```bash
# the loop on a real go2 (onboard laptop, robot runs the go2web zenoh bridge):
# raycaster -> MLS carrot -> local planner -> follower -> cmd_vel
dimos run go2-zenoh-motion

# validate against reality: replay a mem2 recording through the planner,
# precision circles over real Point-LIO clouds
python -m dimos.navigation.motion.adapter.replay mid360_athens_stairs.db --spawn

# bake the robot-side host (runs ON the robot, next to the go2web bridge);
# toolchain prereqs + deploy notes: deployment_plan.md "Baking the motion host"
dimos bake motion_planner trajectory_follower cmd_vel_mux go2_tf \
    -o motion-host --builder zigbuild --target aarch64-unknown-linux-gnu.2.31
```

Both batteries score gate-times-pillars: the planner against the SE(2) gold
oracle (`gate * (100 + 10 + 1)`, max 111), the controller against physics —
per track: blind `gate * (100 + 10 + 0.5 + 0.5)` (max 111, cruise 0.35 =
the stamp-encoding mid-band it may not exceed), hinted
`gate * (100 + 10 + 5 + 0.5)` (max 115.5, cruise 0.75 = the plant ceiling:
a speed gradient worth chasing, since the hint permits outrunning the
encoding).
