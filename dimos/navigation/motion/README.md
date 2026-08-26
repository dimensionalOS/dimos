# Local motion

Plan a path around obstacles over a 5-10 m horizon, walk it, and prove both
work. Three packages, one body:

```
embodiment/     base.py Embodiment · go2.py the measured Go2 · synthetic.py test bodies
obstacles.py    which returns are obstacles: a z-rule the body decides

planner/                                control/
  planners/   base.py protocol, se2.py    controller.py tracks.py profile.py
              the SE(2) search (the       laws/   seed, blind, hinted
              spec), target.py the        rust/   the same laws, bit-exact
              shipped candidate
  rust/       the production crate

adapter/      MotionPlanner + TrajectoryFollower as dimos modules (python and
              native twins), viz.py, diagnostics.py; rust/ is the on-robot host
```

| dir           | what                                                                                        | docs                                 |
|---------------|---------------------------------------------------------------------------------------------|--------------------------------------|
| `embodiment/` | one robot's measured and fitted numbers; planner and follower are configured with the same one |                                   |
| `planner/`    | SE(2) local planner: the rust crate that ships and the python search it is checked against  | [tools.md](tools.md)                 |
| `control/`    | the follower: per-track laws in python + bit-exact rust, and the precision dialect          | [control/tools.md](control/tools.md) |
| `adapter/`    | planner + follower as dimos modules for the go2-zenoh blueprints                            | [adapter/tools.md](adapter/tools.md) |

The planner's world is the raycaster cloud sliced by the body's own z-band; the
search is planar and its route is priced on the follower's governor, so the two
modules optimise one clock. The rust planner carries its own behavioural
invariants (`planner/rust/tests/invariants.rs`): routes an open world, refuses
a sealed box, never hops a thin wall, answers the same way every call,
memoizes nothing across calls.

[tools.md](tools.md). Setting up a Go2:
[docs/platforms/quadruped/go2/motion.md](../../../docs/platforms/quadruped/go2/motion.md);

why the time-critical half runs on the robot:
[motion-deployment.md](../../../docs/platforms/quadruped/go2/motion-deployment.md).

## Module I/O

```python
from dimos.navigation.motion.adapter.planner import MotionPlanner

print(MotionPlanner.io(color=False))
```

```results
 ├─ local_map: PointCloud2
 ├─ planner_path: Path
 ├─ tf: TFMessage
┌┴──────────────┐
│ MotionPlanner │
└┬──────────────┘
 ├─ path: Path
 ├─ plan_body: Path
```

```python
from dimos.navigation.motion.adapter.follower import TrajectoryFollower

print(TrajectoryFollower.io(color=False))
```

```results
 ├─ path: Path
 ├─ local_map: PointCloud2
 ├─ stop_movement: Bool
 ├─ tf: TFMessage
┌┴───────────────────┐
│ TrajectoryFollower │
└┬───────────────────┘
 ├─ nav_cmd_vel: Twist
 ├─ goal_reached: Bool
```

Rules the ports carry:

- `path` timestamps are not a schedule: only their deltas carry information
  (`dt = segment / governor_speed(clearance)`, `control/profile.py`), so slow
  segment = tight segment. Running slower than the encoding is always legal;
  a plain-`ts` path just loses the hint, and third-party producers interoperate.
- a single-pose `path` means "hold, no safe route".
- there is no odometry port: the pose is read off `tf` each tick — the planner
  looks up `world_frame -> base_frame`, the follower `path.frame_id ->
  base_frame`, so it controls in the frame the plan is expressed in. An edge
  whose stamp stops advancing for the module's deadman (`max_map_age_s`,
  `max_path_age_s`) is a missing pose: the planner plans nothing, the follower
  zeroes the twist.
- `local_map` into the follower is optional: with it the room is measured;
  without it the follower decodes the path stamps (the `blind` track).
- one law per *track* (`control/tracks.py`), never named by config or blueprint.
