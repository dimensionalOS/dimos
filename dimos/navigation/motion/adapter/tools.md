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
# replay + input ablation, input ages, follower replay. Writes an rrd + svgs
# under recordings/. It opens by SNIFFING what each stream's payload stamp
# means (sensor-time / receipt-echo / foreign-clock), and where it is sensor
# time the latency pass reports the true pipeline age -- receipt minus stamp --
# on top of the inter-arrival cadence it always had.
python -m dimos.navigation.motion.adapter.diagnose ml-trajectory-research/20260805-033007.zenoh.mcap

# how good is the control: precision-vs-speed frontier, recordings overlay on
# one plot (p50/p95 cross-track per speed bin, red line = the precision floor)
python -m dimos.navigation.motion.adapter.precision rec1.mcap rec2.mcap
python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only churn --spawn
python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only replay --model raw_band

# the FOLLOWER re-run: the deployed law on the deployed config against the twist
# that actually went out, one verdict per recorded nav_cmd_vel tick
python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only follower \
    --host-config motion-host.json --from 6.9 --to 8.6

# every pass takes the same two window flags: seconds into the recording, or a
# UTC time of day matching the message stamps
python -m dimos.navigation.motion.adapter.diagnose rec.mcap --from 06:34:35.4 --to 06:34:37

# view the rrd it wrote (or use --spawn above to stream live)
rerun recordings/<rec>-diagnose.rrd
# in the 3D view: world/map grey cloud (1:4 subsampled), world/appeared green /
# world/disappeared red voxels per frame (full resolution). Three pose sources:
# world/requested blue = the plan the robot published (+ wireframe bodies with
# real yaw), world/robot + world/track green = what the body actually drove
# (box at 30 Hz, track line), world/replay white = the same tick re-planned
# offline. world/obstacles orange = the winning obstacle model's hard set, i.e.
# the very points the replayed search planned around, drawn back on the map
# (the search itself is planar and gets them as xy). world/global violet =
# the MLS route at each republish; world/carrot = the goal dot along it.
# A frozen box + carrot with no replay line = a hold.
#
# The replay reads obstacles through motion/obstacles.py and by default SNIFFS
# which model the deployed stack ran -- it replays a tick subsample under each
# and keeps the one whose holds agree with the recorded plans, so a recording
# made before body_band landed still replays as it ran. --model names one.
#
# The follower pass rebuilds each tick's inputs the way the module does: the
# tf-resolved base pose, the latest path, and -- on the hinted track -- the room
# hint RECOMPUTED from the latest local map through that same obstacle model
# (the blind track decodes the path's stamps instead). It runs the deployed rust
# law when the extension is built and says so; on the python fallback, remember
# the venv wheel goes stale silently after any control/rust change:
#   uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/Cargo.toml
#
# Each tick is match (under --threshold, default 0.15 per component), boundary
# (a plan landed within one control period -- unpairable, left out of the
# stats), hold (the module never reached its law: deadman, goal latch, or a
# single-pose refusal, so the recorded twist is held against zero), or MISMATCH.
# The law rate-limits its own command, so ticks run in order through ONE law
# instance and --from only decides which are REPORTED: a window's first tick
# inherits the state the robot's did, and a window reproduces the full run
# restricted to it, tick for tick.
#
# --host-config reads modules.trajectory_follower.config off a motion-host stdin
# blob (track, controller_config, control_frequency, max_path_age_s,
# obstacle_model); without it the go2-zenoh-motion blueprint's own values stand
# in. A recorded twist faster than the config's max_speed is called out: it is
# proof the blob is not what the robot ran.

# tests and types
python -m pytest dimos/navigation/motion/adapter -q
python -m mypy dimos/navigation/motion/adapter
```

MLS stays the global planner but its path is remapped to `planner_path` and only
feeds the carrot: a point `goal_lookahead_m` (5 m) of arc along it. The local
planner replans to the carrot at `replan_hz` over the raycaster's `local_map`;
a refusal is a single-pose stub the follower holds on while MLS reroutes. The
follower annotates the path with clearance from the local map (the sim judge's
room hint, computed on-robot, off the planner's own obstacle model) and stops
through a goal latch that ignores sub-tolerance carrot jitter. Both modules take odometry as LIO stamps it — at
the sensor — and resolve it into `base_link` off tf (`navigation/tf_pose.py`),
dropping messages until the mount leg arrives.

The controller stays a pure pose+path -> twist law behind the
`TrajectoryController` protocol (`control/controller.py`) — that object, not the
module shell, is what gets ported to rust for on-robot deployment. Candidates
plug in as `controller="module:factory"`, planners as `planner="module:factory"`
(referee registry names work too).
