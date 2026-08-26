# Tools

The motion stack on a robot: dimos modules wrapping the evolved planner
(`MotionPlanner`) and the pursuit controller (`TrajectoryFollower`), wired into
the go2-zenoh blueprints as `go2-zenoh-motion` — raycaster local map -> MLS
global path -> carrot -> local planner -> follower -> cmd_vel.

```bash
# on the onboard laptop, robot running the go2web zenoh bridge
dimos run go2-zenoh-motion

# the rust planner must be built first
uv run maturin develop --uv --release -m dimos/navigation/motion/planner/rust/Cargo.toml

# tests and types
python -m pytest dimos/navigation/motion/adapter -q
python -m mypy dimos/navigation/motion/adapter
```

MLS stays the global planner but its path is remapped to `planner_path` and only
feeds the carrot: a point `goal_lookahead_m` (5 m) of arc along it. The local
planner replans to the carrot at `replan_hz` over the raycaster's `local_map`;
a refusal is a single-pose stub the follower holds on while MLS reroutes. The
follower annotates the path with clearance from the local map (computed
on-robot, off the planner's own obstacle model) and stops
through a goal latch that ignores sub-tolerance carrot jitter. Both modules take odometry as LIO stamps it — at
the sensor — and resolve it into `base_link` off tf (`navigation/tf_pose.py`),
dropping messages until the mount leg arrives.

The controller stays a pure pose+path -> twist law behind the
`TrajectoryController` protocol (`control/controller.py`) — that object, not the
module shell, is what gets ported to rust for on-robot deployment. Candidates
plug in as `controller="module:factory"`, planners as `planner="module:factory"`.
