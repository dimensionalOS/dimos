# The motion stack on the robot: why, and what it costs

The time-critical half of the motion stack — local planner, trajectory
follower, cmd_vel mux, plus the go2 tf tree — runs on the Go2 itself as one
baked binary. Mapping stays on the laptop. The runbook is
[motion.md](/docs/platforms/quadruped/go2/motion.md); this page is the reasoning, the measurements, and
the sharp edges.

## The cut

```
laptop                                   robot
------                                   -----
RayTracingVoxelMap  --- local_map ----->  MotionPlanner        (5 Hz)
MLSPlannerNative    --- planner_path -->  TrajectoryFollower   (10 Hz)
GoalRelay                                 CmdVelMux
MovementManager (click half)              go2web bridge
vis_module
                    <-- lidar, odom ----
                    --- tele_cmd_vel -->
```

Everything on the right is one baked host binary, plus the bridge the robot
already runs.

### Why not the whole stack

The raycaster is the expensive module and we have not yet proven the robot can
carry it. Until then it stays on the laptop and we accept that `local_map`
crosses the link.

### What this cut does and does not buy

It does **not** remove a wire crossing from the perception→action loop. Today
the loop crosses twice (lidar out, `cmd_vel` in); after the move it still
crosses twice (lidar out, `local_map` in), and the message got much bigger.

What it buys is **jitter immunity on the last stage**: the follower ticks at a
steady 10 Hz off a locally-held path instead of the robot receiving `cmd_vel`
in bursts whenever the link hiccups.

### The end state

Move the raycaster to the robot as well and the loop is fully local: the link
then carries lidar for visualization, goals, teleop and telemetry, none of it
in the control path. MLS can stay on the laptop — its path is small and slow,
and a stale global route is benign, because the local planner only takes a
carrot along it.

Notably this costs no extra bandwidth over the current cut: MLS consumes
`local_map` too (its `global_map` is remapped off), so the cloud crosses the
link either way — inbound now, outbound then. The blocker is purely whether
the robot's SBC can afford the raycaster. **It can, for short missions — see
below.**

## How much slower is the robot? (measured 2026-08-03)

The Go2 board is a 7-core Cortex-A55 at 1.42 GHz (1.8 GHz max), Ubuntu 20.04,
glibc 2.31, no rust toolchain. The laptop is a 20-thread Ryzen AI 9 365.

**A wall-time ratio between the two machines is not a usable measure.** The
raycaster's `update_map` is rayon-parallel, so wall time is a function of how
many cores the pool happened to get — 8.3 on the laptop, 2.9 on the robot —
and the ratio moves with the machine's width, not its speed. Two numbers are
portable and worth remembering:

|                                              | laptop             | robot                | factor             |
|----------------------------------------------|--------------------|----------------------|--------------------|
| single-thread CPU-s per 310 s of lidar       | 100.8              | 310.8                | **3.08x**          |
| single-thread per-frame p50 / p95 / p99 (ms) | 28.9 / 64.7 / 87.3 | 99.7 / 187.7 / 237.7 | 3.4x / 2.9x / 2.7x |

**Rule of thumb: one Go2 core ≈ one third of a laptop core on this workload.**
Use the single-thread CPU-time ratio when sizing anything else for the robot;
use per-frame p99 when the question is whether a loop closes on time.

Do not use the multi-threaded ratio (fps 112.7 vs 21.6, ≈5.2x). Rayon's spin
overhead more than doubles the CPU-seconds the job costs when the pool is
wide — the laptop burned 227.8 CPU-s to do the same work it does in 100.8
CPU-s on one thread — so that 5.2x is a statement about core counts, not speed.

### What the raycaster actually costs on the robot

Replaying `data/mid360_athens_stairs.db` (3093 clouds at 9.99 Hz, ~4.6k
points/cloud, 310 s) through the real `RayTracingVoxelMap` at wall-clock speed,
with the `go2_zenoh_raycaster` config (`voxel_size=0.08`, `emit_every=10`,
`global_emit_every=100`, `support_min=4`):

|                  | laptop  | robot (rayon=7) | robot (rayon=2) |
|------------------|---------|-----------------|-----------------|
| clouds processed | 99.9%   | 97.0%           | 91.8%           |
| sustained rate   | 9.98 Hz | 9.69 Hz         | 9.17 Hz         |
| cores consumed   | 0.83    | 1.46            | 1.02            |
| peak RSS         | 305 MB  | 304 MB          | 304 MB          |

Robot idle baseline at the time: **2.80 of 7 cores busy, 4.20 free** (30 s
`/proc/stat` sample), with the go2web bridge, `basic_service`, `mcf_main`,
`dimos-pointlio`, `videohub` and friends running. So the raycaster fits — it
adds ~1.5 cores to a machine with ~4.2 free — but three things bound the claim:

- **The tail exceeds the frame budget.** p99 is 140 ms against a 100 ms
  budget at 10 Hz. Nothing overflows the 128-deep input queue. The missing 3%
  (and 8% at rayon=2) was the cloud being dropped for want of a pose — but
  load was never the whole story, and blaming it hid how little margin there
  was. `select!` polls its ready arms in random order, so a cloud outruns the
  odometry queued beside it whatever the load; and the go2 sweep measures
  100.8 ms, not the nominal 100, which puts the previous sweep's pose 0.8 ms
  outside `POSE_MATCH_TOLERANCE_S`. There was no second chance to lose.
  Measured on 20260807-190044.mcap at 6.1% of clouds, on an idle laptop.
  The module now holds an unmatched cloud for its own sweep's pose instead of
  discarding it, so re-measure this row before quoting it.
- **Cost grows with the map, without bound.** Single-threaded per-frame mean
  went 73 ms → 161 ms over the 5-minute recording as the map reached 366k
  voxels; `emit_points` walks every voxel on each emit. A 5-minute run fits.
  A 20-minute one probably does not, at this `voxel_size`.
- **Throttling rayon does not pay.** rayon=2 saves 0.44 cores but drops 8% of
  clouds. If the raycaster moves to the robot, give it the full pool.

Verdict: **option C is affordable for short missions and needs a bound on map
growth before it is affordable for long ones.** Alongside a 5 Hz planner and a
10 Hz controller the projected total is ~5.3 of 7 cores — real but thin
headroom, and the growth curve is what will break first.

## Consequences the cut has to handle

### Staleness — the link was the deadman

With the loop on the laptop, a dropped link stopped `cmd_vel` and the bridge
watchdog halted the robot. Once the loop runs on the robot that accidental
safety is gone: the planner would keep replanning on a frozen map while the
follower tracks the result at cruise speed, and the speed governor reads its
clearance off that same stale cloud, so it stays confident. Three guards:

| where                | rule                                                                             |
|----------------------|----------------------------------------------------------------------------------|
| `MotionPlanner`      | `local_map` older than `max_map_age_s` (5 s) → publish the single-pose hold stub |
| `TrajectoryFollower` | `path` older than `max_path_age_s` (2.5 s) → zero the twist                      |
| `CmdVelMux`          | `nav_cmd_vel` older than `nav_stale_s` (0.5 s) → zero `cmd_vel`                  |

Staleness is measured from **arrival**, not `msg.ts`: the mapper's clock is not
the robot's, and what these guard is how long since the producer was last heard
from.

### One keystroke lands on both sides

`MovementManager` was a click-to-goal relay *and* a velocity mux, and the two
halves land on opposite sides of the link. So teleop is consumed twice: the
rust `CmdVelMux` on the robot preempts `cmd_vel` and publishes `stop_movement`
to the follower beside it; the python `MovementManager` on the laptop turns
clicks into goals and cancels them (the NaN goal to GoalRelay/MLS). Both
subscribe `tele_cmd_vel`; nothing routes the cancel back across the link.

### tf on the robot

The planner and the follower both need tf. LIO stamps its odometry at the
SENSOR (`mid360_link`), so the pose it carries is the lidar's, not the robot's
— 0.30 m ahead of the body and 0.16 m above it on this rig. Both modules
resolve it through `tf_pose::OdomBasePose`, the twin of
`dimos/navigation/tf_pose.py`: look the static `mid360_link → base_link` leg up
once, cache it, compose it onto every message, and DROP messages until the leg
arrives.

The leg comes from `go2_tf`, baked into the host, so it does not depend on the
laptop being up. `go2-zenoh-motion-local` remaps `GO2Zenoh`'s tf away for the
same reason: two publishers of one static tree is not redundancy, it is
`base_link` jumping between two mounts at their combined rate.

The failure mode is quiet but not dangerous: a host that never sees tf holds
its pose at `None` and plans nothing, rather than planning off-heading. One
"dropping odometry" line per outage says so.

## Config is the sharp edge

The rust modules have **no defaults**: `#[native_config]` forbids serde
defaults and rejects unknown fields, so the host needs every field of its
config and exits naming the first one it lacks. Python owns the defaults.
That is a feature — one owner — right up until the deployed config is a file
someone hand-maintains, because then a renamed field, a re-shaped one (the
embodiment went from a registry name to the body itself) or a blueprint-tuned
value the class does not carry all present as the host refusing to boot, or
worse, as a controller bug. Three debugging sessions were lost to exactly
that skew before the config was made generated, and one more to a generated
file that was not regenerated with its binary.

So the config travels inside the binary. `dimos bake --deployment` takes a
`Deployment` (`dimos.robot.unitree.go2.zenoh.motion_host:GO2_MOTION_HOST`:
the module list, the overrides applied as pydantic constructor arguments —
`body_dilate_m`, the `max_speed` ceiling — and the zenoh `session` block),
builds the blob from the python classes and embeds it, `graph` stamp and all.
Binary and config cannot drift apart because there is one artifact. A test
re-validates every module block into its config class, so a field that drifts
fails in CI, not on the robot.

Stdin is for overrides only. A JSON line there deep-merges over the embedded
blob — objects descend, scalars and arrays replace — so one nested number can
be changed for a run without a rebake. Nothing on stdin (the unit, a tty)
means the embedded config as is. The merged result is checked exactly as
before: an unknown key, or a `graph` stamp from another bake, is refused.

The `session` block is load-bearing. go2web runs as the zenoh ROUTER on 7447;
the host is its CLIENT over loopback and listens on nothing. The rust side
reads only `DIMOS_TRANSPORT` from the environment, so without the block it
opens zenoh's own defaults — a peer with multicast scouting, which a router
does not forward to.

### Blueprint-as-arg

The eventual fix: the blueprint is the only place the tuned wiring and config
actually live, and the remappings (`MLS.path→planner_path`,
`global_map→global_map_unused`) are exactly the `--remap` flags one would
otherwise retype, where drift is a silent misconnect. Bake a named subset of a
blueprint, leave the rest external, and emit the replacement blueprint so the
binary and the python driving it cannot drift. Known limit it will surface:
`select_modules` refuses duplicate ids, while blueprints can carry two
instances of a class under different namespaces. Not blocking for this stack.
