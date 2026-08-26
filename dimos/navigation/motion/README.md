# Local motion

Plan a path around obstacles over a 5-10 m horizon, walk it, and prove both
work. Three packages, one body:

```
embodiment/     base.py Embodiment · go2.py the measured Go2 · synthetic.py test bodies
obstacles.py    which returns are obstacles: a z-rule the body decides
loader.py       the one "module:factory" loader

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

## I/O

Stock dimos msgs; the port declarations are the spec:

```python
from typing import get_args, get_origin, get_type_hints
from dimos.core.stream import In, Out
from dimos.navigation.motion.adapter.follower import TrajectoryFollower
from dimos.navigation.motion.adapter.planner import MotionPlanner

for module in (MotionPlanner, TrajectoryFollower):
    print(module.__name__)
    for name, hint in get_type_hints(module).items():
        if get_origin(hint) in (In, Out):
            print(f"  {get_origin(hint).__name__:<4} {name:<14} {get_args(hint)[0].__name__}")
```

```results
MotionPlanner
  In   local_map      PointCloud2
  In   odometry       Odometry
  In   planner_path   Path
  In   tf             TFMessage
  Out  path           Path
  Out  plan_body      Path
TrajectoryFollower
  In   path           Path
  In   odometry       Odometry
  In   local_map      PointCloud2
  In   stop_movement  Bool
  In   tf             TFMessage
  Out  nav_cmd_vel    Twist
  Out  goal_reached   Bool
```

Rules the ports carry:

- `path` timestamps are not a schedule: only their deltas carry information
  (`dt = segment / governor_speed(clearance)`, `control/profile.py`), so slow
  segment = tight segment. Running slower than the encoding is always legal;
  a plain-`ts` path just loses the hint, and third-party producers interoperate.
- a single-pose `path` means "hold, no safe route".
- `local_map` into the follower is optional: with it the room is measured;
  without it the follower decodes the path stamps (the `blind` track).
- one law per *track* (`control/tracks.py`), never named by config or blueprint.
