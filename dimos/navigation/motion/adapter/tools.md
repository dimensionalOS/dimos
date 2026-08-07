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

# replay a real recording through the planner: rerun view with the
# required-precision circles (radius = clearance hint, red/yellow/green =
# creep/governed/full-speed) computed from the REAL Point-LIO clouds
python -m dimos.navigation.motion.adapter.replay mid360_athens_stairs.db --spawn
python -m dimos.navigation.motion.adapter.replay mid360_athens_stairs.db --save athens.rrd --every 3

# post-mortem a recording of the live stack: map churn, plan flips, planner
# replay + input ablation, input ages. Writes an rrd + svgs under recordings/.
python -m dimos.navigation.motion.adapter.diagnose ml-trajectory-research/20260805-033007.zenoh.mcap
python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only churn --spawn
python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only replay --z-offset 0.25

# view the rrd it wrote (or use --spawn above to stream live)
rerun recordings/<rec>-diagnose.rrd
# in the 3D view: world/map grey cloud (1:4 subsampled), world/appeared green /
# world/disappeared red voxels per frame (full resolution). Three pose sources:
# world/requested blue = the plan the robot published (+ wireframe bodies with
# real yaw), world/robot + world/track green = what the body actually drove
# (box at 30 Hz, track line), world/replay white = the same tick re-planned
# offline. world/carrot = the goal dot. A frozen box + carrot with no replay
# line = a hold.

# tests and types
python -m pytest dimos/navigation/motion/adapter -q
python -m mypy dimos/navigation/motion/adapter
```

MLS stays the global planner but its path is remapped to `planner_path` and only
feeds the carrot: a point `goal_lookahead_m` (5 m) of arc along it. The local
planner replans to the carrot at `replan_hz` over the raycaster's `local_map`;
a refusal is a single-pose stub the follower holds on while MLS reroutes. The
follower annotates the path with clearance from the local map (the sim judge's
room hint, computed on-robot) and stops through a goal latch that ignores
sub-tolerance carrot jitter. Both modules take odometry as LIO stamps it — at
the sensor — and resolve it into `base_link` off tf (`navigation/tf_pose.py`),
dropping messages until the mount leg arrives.

The controller stays a pure pose+path -> twist law behind the
`TrajectoryController` protocol (`control/controller.py`) — that object, not the
module shell, is what gets ported to rust for on-robot deployment. Candidates
plug in as `controller="module:factory"`, planners as `planner="module:factory"`
(referee registry names work too).
