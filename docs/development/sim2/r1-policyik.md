# R1 Pro paired-hand PolicyIK preview

This example runs a frozen coordinated upper-body PolicyIK actor through Sim2
physics and independent encoders. It controls four torso joints and both
seven-joint arms. The chassis and grippers remain fixed.

## Where it belongs

```text
Paired Cartesian tool goals (teleop or Cartesian planner)
                  |
             PolicyIKTask <--- joint encoders + robot-model FK
                  |
          18 joint position references
                  |
       PolicyIKSimModule / expiring ActionCommand
                  |
       SimModule / MuJoCo / independent encoder sensor
```

`PolicyIKTask` occupies the Cartesian-to-joint role of `CartesianIKTask` or
`TeleopIKTask`. Its outputs already are joint references: do not send them
through Pink IK again. A planner producing joint trajectories instead uses the
existing trajectory task. The coordinated task claims all 18 joints together;
separate arm and torso tasks must not simultaneously command them.

The portable task implements `BaseControlTask`. This preview hosts it directly
from encoder callbacks, following the Sim2 G1 pattern. It does not yet add a
task-registry factory, an R1 hardware connection, a Quest teleop binding, or a
planner frontend. Callers can supply paired goals through
`hand_goals: In[PoseArray]` or `set_goals(goals, frame)` RPC. Goals are left then
right in the bundle's fixed `base_link` frame. RPC rows contain
`[x, y, z, qw, qx, qy, qz]`, in metres and unit quaternions.

The actor receives joint positions/velocities, FK tool poses, both goals, error
history, accumulated-reference error, and previous action. It receives no camera
images, object detections, or simulator body poses. The robot model supplies
forward kinematics, not inverse kinematics. This is a local pose controller,
not an obstacle planner or an object-manipulation policy.

## Run

This branch composes main with the Sim2 stack through PR #5041. That stack pins
unreleased `dimos-lcm` definitions from PR #31. Install the lockfile.

```bash
uv sync --frozen --extra sim --extra planning --extra cpu --no-dev
```

Supply an exported bundle with `manifest.json`, `actor.npz`, `robot.urdf`, its
meshes, and `development_cases.npz`. Weights and robot assets are not included
in the source checkout. The manifest fixes actor/URDF hashes, joint/tool order,
frame, gains, effort limits, and original checkpoint hash.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 uv run --no-sync dimos-r1-policyik-prepare \
  --bundle /path/to/bundle --output /path/to/new-scene.usda

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 uv run --no-sync dimos-r1-policyik-demo \
  --bundle /path/to/bundle --scene /path/to/new-scene.usda \
  --host 127.0.0.1 --port 8875
```

Open the printed URL. Drag a target frame and press **Apply paired targets**, or
select a development pair and press **Load selected target pair**. The solid
robot follows measured encoders. **Pause policy / hold references** stops
inference while refreshing the last commanded references with active servos.
Resume continues the current goals. This is not an emergency stop. Command
expiry retains Sim2's fallback, which can sag under gravity when it replaces
loaded references with measured joint positions.

`--no-viewer --duration 25 --output /path/to/new-run` runs a bounded integration
check. Output includes configuration, telemetry, and final status. The demo
creates an isolated transport domain and no hardware connection. For remote
access, bind `--host` to a browser-reachable private interface.

## Fidelity and measured limits

The NumPy CPU actor implements the 124-input, 256/256 ELU, 18-output network and
native normalization. It preserves two-hand history, the gain schedule, and the
accumulated reference update. It adds neither `tanh` nor an action clip to +/-1.
Inference uses fresh encoders at 30 Hz, while physics advances independently at
approximately 480 Hz. Commands expire after 100 ms. Duplicate samples do not
advance the policy state.

The actor was trained in Isaac Gym/PhysX; this Sim2 backend uses MuJoCo. Gains,
effort limits, geometry and joint order were checked, but MuJoCo does not enforce
the native hard joint-speed caps. Its contact model and collision scoring also
differ. Native benchmark success is not a MuJoCo or hardware result.

A preselected 32-pair training-pool transfer check reached both targets in 23
cases. Sixteen also passed the recorded force, joint-position and penetration
checks. All 32 exceeded a native joint-speed cap at some point, so those sixteen
are not fully constraint-qualified. This development check is separate from the
held-out native 826/1024 result and does not establish moving-target teleoperation
performance. Arbitrary dragged or switched targets may fail.

The inference equations follow the Apache-2.0
[PolicyIK reference](https://tinaliu0123.github.io/PolicyIK/). The coordinated R1
extension and checkpoint are separate from the published OpenArm benchmark.
