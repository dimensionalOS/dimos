# Tools

Three packages: `planner/` plans, `control/` follows a plan, `adapter/`
deploys both as dimos modules for the go2-zenoh blueprints. Run from the repo
root; per-package `tools.md` has the full menus.

`planner/` is a runner (`python -m ...planner`) over a `referee/` that scores
whatever the runner picked. Any candidate is a `module:factory` string, so all
of them go through the same judge:

```bash
python -m dimos.navigation.motion.planner --score --planner gold
python -m dimos.navigation.motion.planner --score --planner my.candidate:make
```

The control side has no runner here: its referee drives a MuJoCo plant, so it
stayed with the sim on `ivan/feat/trajectory_ctrl` (see [README.md](README.md),
"what is not here"). What ships here is the law and its bit-exact rust.

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

# validate against reality: replay a mem2 recording through the planner,
# precision circles over real Point-LIO clouds
python -m dimos.navigation.motion.adapter.replay mid360_athens_stairs.db --spawn

# bake the robot-side host (runs ON the robot, next to the go2web bridge);
# toolchain prereqs + deploy notes: deployment_plan.md "Baking the motion host"
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
