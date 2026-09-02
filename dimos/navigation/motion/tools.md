# Tools

Three packages: `planner/` plans, `control/` follows a plan, `adapter/`
deploys both as dimos modules for the go2-zenoh blueprints. Run from the repo
root; per-package `tools.md` has the full menus.

What is here is what runs on a robot, and the tests that hold it:
python-vs-rust parity on the laws, the crate's own behavioural invariants, and
the module wiring. The benchmarks are not on this branch ([README.md](README.md),
"Not on this branch").

## planner

```bash
# the crate's behavioural invariants: routes an open world, refuses a sealed
# box, never hops a thin wall, same answer every call, no cross-call memo.
cargo test --release --test invariants -p dimos-motion2-target

# rebuild the extension after editing rust/
uv run maturin develop --uv --release -m dimos/navigation/motion/planner/rust/py/Cargo.toml
```

## control ([control/tools.md](control/tools.md))

```bash
# python law and rust law, tick for tick, over the whole sweep
uv run pytest dimos/navigation/motion/control

# rebuild the crate after editing rust/
uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/py/Cargo.toml
```

## adapter ([adapter/tools.md](adapter/tools.md))

```bash
# the loop on a real go2 (onboard laptop, robot runs the go2web zenoh bridge):
# raycaster -> MLS carrot -> local planner -> follower -> cmd_vel
dimos run go2-zenoh-motion

# bake AND install the robot-side host (runs ON the robot, next to the go2web
# bridge). Needs the dev shell for the rust cross toolchain.
deploy/deploy.sh <ssh-host>
```
