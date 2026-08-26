# The motion stack on the robot: the cut

Runbook: [motion.md](/docs/platforms/quadruped/go2/motion.md).

```
laptop                                   robot
------                                   -----
RayTracingVoxelMap  --- local_map ----->  MotionPlanner        (5 Hz)
MLSPlannerNative    --- planner_path -->  TrajectoryFollower   (10 Hz)
GoalRelay                                 CmdVelMux
MovementManager (click half)              go2_tf
vis_module                                go2web bridge (zenoh router :7447)
                    <-- lidar, odom ----
                    --- tele_cmd_vel -->
```

The right column is one baked binary, a zenoh client of the bridge over
loopback. The raycaster stays on the laptop until the robot is shown to carry it.

What the cut has to handle:

- **Staleness**: the link is no longer the deadman. Planner holds on stale
  `local_map` (`max_map_age_s`), follower zeroes on stale path
  (`max_path_age_s`), both treat a tf pose whose stamp stopped advancing for
  that same age as missing, mux zeroes on stale `nav_cmd_vel` (`nav_stale_s`);
  all measured from arrival, not `msg.ts`.
- **Stop lands on both sides**: `CmdVelMux` on the robot preempts `cmd_vel`;
  `MovementManager` on the laptop cancels the goal. Both read `tele_cmd_vel`.
- **tf on the robot**: `go2_tf` is baked in so `mid360_link → base_link` does
  not depend on the laptop; `go2-zenoh-motion-local` remaps `GO2Zenoh`'s tf
  away so there is one publisher of the static tree.

Config travels in the binary: `dimos bake --deployment` embeds the blob built
from the python classes, so binary and config cannot drift. A JSON line on
stdin deep-merges over it for one run; unknown keys are refused.
