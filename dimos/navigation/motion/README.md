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

Oneliners for everything: [tools.md](tools.md). Setting up a Go2:
[docs/platforms/quadruped/go2/motion.md](../../../docs/platforms/quadruped/go2/motion.md);
why the time-critical half runs on the robot:
[motion-deployment.md](../../../docs/platforms/quadruped/go2/motion-deployment.md).

## I/O — everything is stock dimos msgs

**MotionPlanner** (adapter)

- in: `local_map: sensor_msgs.PointCloud2` — the raycaster's accumulated cloud
  around the robot
- in: `odometry: nav_msgs.Odometry` — own pose (pointlio, `odom` frame),
  resolved into `base_link` off tf
- in: `planner_path: nav_msgs.Path` — global path; the goal is a carrot along it
- out: `path: nav_msgs.Path` — the local plan. Per-waypoint `ts` deltas encode
  required precision (`dt = segment / governor_speed(clearance)`, see
  `control/profile.py`); a single-pose path means "hold, no safe route"

**TrajectoryFollower** (adapter)

- in: `path: nav_msgs.Path` — stamped or plain (flat `ts` just disables the hint)
- in: `odometry: nav_msgs.Odometry`
- in: `local_map: sensor_msgs.PointCloud2` — optional, richer clearance hint;
  without it the follower decodes the path stamps instead
- in: `stop_movement: std_msgs.Bool`
- out: `nav_cmd_vel: geometry_msgs.Twist` — body-frame (vx, vy, wz) into the
  walking policy
- out: `goal_reached: std_msgs.Bool` — latched arrival

**Inner follower law** (in-process, python and rust produce identical bits):

```
update(pose: PoseStamped, path: Path, t, clearance: ndarray | None) -> Twist
```

One law per *track* — the input regime the follower runs under. `hinted` gets
the clearance array, `blind` does not and recovers the same profile from the
path's own stamps; `control/tracks.py` is the only place that maps a track to
its law, so the follower config and the blueprints never name a law directly.

The path timestamps are NOT a schedule — only their deltas carry information
(slow segment = tight segment = track carefully), a follower must never chase
the clock, and running slower than the encoding is always legal. Third-party
Path producers/consumers interoperate; they just don't get the precision hint.
Because the encoding rides on the path itself it survives the blind track,
which is what makes blind a real deployment case rather than a handicap.

## Not on this branch

Everything that trains, searches or simulates — the matched Go2 MuJoCo env,
the planner's SE(2) gold oracle and judge, the closed-loop follower referee,
and the autoresearch labs — lives on `ivan/feat/trajectory_ctrl`. Every speed,
envelope and slip constant quoted here was measured there; re-measure there.
