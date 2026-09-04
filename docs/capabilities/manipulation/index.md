# Manipulation

Motion planning and teleoperation for robotic manipulators. RoboPlan provides
the default world and native path planner.

For typed client RPCs, see [Manipulation from Python](/docs/capabilities/manipulation/python_api.md).

## Quick Start

Recent addition: the A-750 keyboard teleop blueprint is now available via:

```bash
dimos run keyboard-teleop-a750
```

### Keyboard Teleop (single command)

Each blueprint launches the full stack: keyboard UI, mock controller, IK solver, and Drake visualization:

```bash
dimos run keyboard-teleop-a750    # A-750 6-DOF
dimos run openarm-planner-coordinator # OpenArm bimanual 2x(7-DOF + gripper)
dimos run keyboard-teleop-a1z     # Galaxea A1Z 6-DOF
dimos run keyboard-teleop-piper   # Piper 6-DOF
dimos run keyboard-teleop-openyam # OpenYAM 6-DOF + gripper
dimos run keyboard-teleop-xarm6   # XArm6 6-DOF
dimos run keyboard-teleop-xarm7   # XArm7 7-DOF
```

OpenYAM is exposed as one whole-body device with six angular arm joints and a
normalized gripper joint. `arm/gripper` uses `0.0` for fully closed and `1.0`
for fully open; it does not use meters. Hardware activation calibrates both
mechanical endpoints, so clear the gripper jaws and workspace before startup.
The gripper has no default startup target and moves only after joint control has
an explicit target.

OpenArm follows the same whole-body model with both arms and both grippers in
one device: fourteen angular joints (`left_arm/joint1..7`,
`right_arm/joint1..7`) plus two normalized gripper joints (`left_arm/gripper`,
`right_arm/gripper`). The keyboard jogs the left arm while the right arm holds
its pose; keyboard gripper bindings are a follow-up.

Open the Meshcat URL printed in the terminal (default `http://localhost:7000`) to see the robot.

Keyboard controls:

| Key | Action |
|-----|--------|
| W/S | +X/-X (forward/back) |
| A/D | +Y/-Y (left/right) |
| Q/E | +Z/-Z (up/down) |
| R/F | +Roll/-Roll |
| T/G | +Pitch/-Pitch |
| Y/H | +Yaw/-Yaw |
| ESC | Quit |

### Motion Planning (two terminals)

```bash
# Terminal 1: Mock coordinator
dimos run coordinator-mock

# Terminal 2: Planner with Drake visualization
dimos run xarm7-planner-coordinator
```

Pink IK is the default solver. Tune it with nested module config overrides:

```bash
dimos run xarm7-planner-coordinator \
  --kinematics.backend=pink \
  --kinematics.max-iterations=100 \
  --kinematics.dt=0.02
```

The same nested shorthand applies to the `ManipulationModule` composed by
pick-and-place blueprints:

```bash
dimos run xarm-perception-sim \
  --kinematics.backend=pink
```

Then run the typed Python motion example:

```bash
python -m dimos.manipulation.planning.examples.manipulation_client
```

The example uses `Dimos.connect()` and `app.get_module(ManipulationSpec)` to
discover the group, plan, preview, and execute. The
[Python guide](/docs/capabilities/manipulation/python_api.md) also covers scan, pick, and place.

### Planning backend selection

Manipulation planning separates the world backend from the planner algorithm:

- `world_backend` selects the robot/world/collision representation.
- `planner.backend` selects the path-planning algorithm.
- `kinematics.backend` selects the IK backend. The legacy `kinematics_name`
  field remains available as a compatibility shim.


```bash
dimos run xarm7-planner-coordinator
```

Select the legacy Drake world and generic RRT planner explicitly when needed:

```bash
dimos run xarm7-planner-coordinator \
  --world-backend=drake \
  --planner.backend=rrt_connect
```

Valid combinations:

| `world_backend` | `planner.backend` | `kinematics.backend` | Status |
|-----------------|----------------|-------------------|--------|
| `roboplan` | `roboplan` | `pink` or `jacobian` | Default path; RoboPlan-native planner |
| `drake` | `rrt_connect` | `pink` | Legacy Drake world |
| `drake` | `rrt_connect` | `jacobian` | Legacy Jacobian IK |
| `drake` | `rrt_connect` | `drake_optimization` | Drake-only IK |
| `roboplan` | `rrt_connect` | `pink` or `jacobian` | Generic RRT over RoboPlan collision checks |

Invalid combinations fail during startup instead of waiting for the first plan
request. For example, `planner.backend=roboplan` requires
`world_backend=roboplan`, and `kinematics.backend=drake_optimization` requires
`world_backend=drake`.

Trajectory parametrization is a separate startup choice. Joint-space planners
normally return an untimed geometric path; dimOS accepts the plan only after
the selected backend converts that path to a validated timed trajectory:

```bash
# Stock xArm compatibility test: independent trapezoids on RoboPlanWorld
dimos run xarm7-planner-coordinator \
  --trajectory-parametrization.backend=simple_trapezoid

# Omitting trajectory_parametrization selects TOPP-RA for RoboPlanWorld
dimos run xarm7-planner-coordinator

# Equivalent explicit TOPP-RA selection
dimos run xarm7-planner-coordinator \
  --world-backend=roboplan \
  --trajectory-parametrization.backend=roboplan_toppra

# DrakeWorld selects simple_trapezoid when no parametrizer is specified
dimos run xarm7-planner-coordinator \
  --world-backend=drake \
  --planner.backend=rrt_connect
```

Exactly one backend is constructed for the stack lifetime. There is no
cross-backend fallback. `roboplan_toppra` may parametrize paths from either
RoboPlan's planner or the generic RRT planner, but it requires
`world_backend=roboplan` because it reuses that world's model, groups, and URDF
motion limits. A planner-native result that already has timestamps and
velocities bypasses path parametrization and retains its existing timing after
canonical validation. TOPP-RA follows the collision-checked geometric path
without corner blending; collision checking remains the planner's concern.
Explicit configuration overrides the world-based default.
RoboPlan model preparation preserves authored acceleration limits and inserts a
temporary default `2.0 rad/s²` limit where they are absent. Formal per-joint
acceleration overrides will replace this fallback.

The Viser panel's **Next plan speed** slider provides runtime speed tuning from
`0.05` to `1.0`. Changing it leaves the accepted plan and any active execution
unchanged; press **Plan** again to generate motion at the new scale. For
joint-space planning the value reduces the selected parametrizer's configured
velocity and acceleration scales. For Cartesian planning Viser puts the same
scale into a native time-optimal planning request before its timestamps are
generated. Cartesian output is sampled every 50 ms; the control coordinator
interpolates it at execution rate.

RoboPlan shortens native joint-space RRT paths by default. Configure or disable
the backend's best-effort shortcutting pass with nested planner options:

```bash
dimos run xarm7-planner-coordinator \
  --planner.path-shortcutting.enabled true \
  --planner.path-shortcutting.max-iters 100 \
  --planner.path-shortcutting.max-step-size 0.05
```

Existing RoboPlan deployments may therefore receive paths with fewer waypoints.
Shortcutting configuration is copied when the RoboPlan planner is constructed.

The remaining options mirror RoboPlan's native path shortcutter:
`seed`, `max_convergence_iters`, and `redundant_removal_iters`. If shortcutting
fails, planning returns the valid raw RRT path and logs a warning.

RoboPlan Cartesian options are supplied per planning request:

```python skip
from dimos.manipulation.planning.planners.roboplan_config import (
    RoboPlanCartesianPathConfig,
)

path_config = RoboPlanCartesianPathConfig()
```

The default `time_optimal` mode returns the TOPP-RA trajectory constrained by
the robot's joint velocity and acceleration limits. To enforce Cartesian speed
and acceleration maxima instead, opt into bounded mode:

```python skip
bounded_config = RoboPlanCartesianPathConfig(
    speed_mode="bounded",
    max_linear_speed=0.1,
    max_angular_speed=0.5,
    max_linear_acceleration=0.5,
    max_angular_acceleration=2.5,
    max_position_error=0.005,
    max_orientation_error=0.01,
)
```

RoboPlan first resolves the Cartesian reference as a geometric joint path, then
uses TOPP-RA to produce the timed trajectory. Both speed modes follow this
pipeline. Time-optimal mode returns the joint-limit-constrained trajectory;
bounded mode slows it further when needed to respect the configured Cartesian
speed and acceleration maxima. `toppra_blend_deviation` controls TOPP-RA corner
rounding in both modes and influences how aggressively the resolved path is
decimated before timing.

The remaining settings mirror RoboPlan's Cartesian planner options, including
sample time, solver weights, linear/angular acceleration limits, joint
velocity/acceleration scaling, TOPP-RA corner blending, and joint-limit
handling. RoboPlan 0.6 removed the former `limit_ratio_tolerance` and
`max_attempts_per_step` settings.

Cartesian path planning remains a low-level internal capability in this
release. The internal generator accepts an ordered waypoint sequence for each
target planning group. A sequence contains only
`PoseStamped` absolute waypoints or only `Transform` displacements relative to
the planning start, and begins at the current TCP pose or identity transform.
RoboPlan plans all target groups simultaneously. The Viser panel constructs a
two-waypoint absolute path for interactive planning. There is no skill, MCP
tool, or CLI motion command yet.

### Cartesian control IK

Cartesian, keyboard EEF-twist, and engagement-relative teleop IK tasks use the
portable `RobotModel` from `RobotModelConfig`. The model owns source loading,
package paths, and Xacro arguments; the configuration supplies the named end-effector frame and
coordinator-to-model joint mapping. Invalid models, frames, or mappings fail at
startup; teleop configuration does not use a separate model path or numeric
end-effector joint ID.

Each control tick starts from measured joints, applies model position and
velocity limits, and holds the measured position when a solve cannot produce a
safe command. This local control path is separate from manipulation planning and
does not use `WorldSpec` or provide world-obstacle avoidance.

For a custom robot, pass the typed model configuration to the helper:

```python skip
from dimos.robot.manipulators.common.blueprints import cartesian_ik_task, teleop_ik_task

task = cartesian_ik_task(
    hardware,
    robot_model=robot_model,
)
teleop_task = teleop_ik_task(
    hardware,
    name="teleop_arm",
    hand="right",
    robot_model=robot_model,
)
```

Teleop pose commands are deltas from an end-effector pose captured from measured
joints at engagement. Disengage, timeout, stop, clear, or E-STOP discards that
baseline; commands received during E-STOP are rejected rather than replayed
after clear.

Validate Cartesian, twist, and teleop behavior in simulation or replay before
hardware use.

Install the manipulation dependencies:

```bash
uv sync --extra manipulation --inexact
```

The `manipulation` extra includes RoboPlan via `roboplan` from PyPI.
The `--inexact` flag preserves other extras already installed in your current
environment.

Safety behavior for unsupported RoboPlan features:

- Planning-critical unsupported inputs fail loudly before planning. Examples
  include unsupported obstacle geometry, unavailable robot loading APIs, or
  unavailable collision query APIs. RoboPlan worlds generate a minimal SRDF from
  the dimOS robot config, including configured collision-exclusion pairs.
- Unverified non-critical query methods raise explicit `NotImplementedError`.
  In particular, signed minimum-distance semantics are not implemented for
  RoboPlan until a safe equivalent is verified.
- Embedded Meshcat visualization requires a world implementing `VisualizationSpec`;
  use Viser or `none` with the RoboPlan backend.

### Planning Visualization

Manipulation visualization is configured on `ManipulationModuleConfig.visualization`.
It is independent from the global Rerun stream viewer in `docs/usage/visualization.md`.

Backend choices:

- `meshcat`: embedded Drake/Meshcat visualizer. The planning world must be created with
  embedded visualization enabled, so this is selected through the visualization config.
- `viser`: in-process Viser visualizer. It renders pushed current robot state,
  target controls, transient preview ghosts, synchronized trajectory previews,
  and optional panel controls.
- `none`: no manipulation planning visualization.

CLI example:

```bash
uv run dimos run xarm7-planner-coordinator \
  --visualization.backend=viser
```

Viser binds to `127.0.0.1` by default. To expose it on the network, opt in
explicitly with the nested host override:

```bash
uv run dimos run xarm7-planner-coordinator \
  -o manipulationmodule.visualization.backend=viser \
  -o manipulationmodule.visualization.host=0.0.0.0
```

Blueprint example:

```python skip
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig

manipulation = ManipulationModule.blueprint(
    config=ManipulationModuleConfig(
        model=robot_model,
        visualization={
            "backend": "viser",
            "host": "127.0.0.1",
            "port": 8095,
            "open_browser": True,
            "panel_enabled": True,  # default; set False for scene-only Viser
        },
    )
)
```

Viser support is included in the `manipulation` extra:

```bash
uv sync --extra manipulation --inexact
```

The Viser panel talks to the concrete `ManipulationOperator` bound into its
`VisualizationSession`. GUI callbacks enqueue operations through that operator
for target evaluation, planning, preview, execution, cancellation, reset, and
clear-plan actions. The panel owns only target drafts, selection state, and
callback generations; it does not touch `WorldSpec`, IK, planner objects,
`ManipulationModule`, `WorldMonitor`, or live Drake contexts directly.

The panel's **Planning mode** selector chooses how the current target is
reached:

- **Joint space** is the default. It resolves the target to joints and invokes
  the configured collision-free joint-path planner.
- **Cartesian space** sends the existing transform-control poses as absolute
  world-frame TCP goals to RoboPlan's Cartesian path planner. Selected groups
  without a TCP participate as auxiliary groups.

Cartesian planning failure never falls back to joint-space planning. A backend
without Cartesian path support reports `UNSUPPORTED`; collision or tracking
failure leaves the plan unavailable. Preview and execution use RoboPlan's
original synchronized timestamps and velocities.

External manipulation visualizers are initialized from a backend-neutral
`VisualizationSession` after the planning world has loaded its model. The
session contains static `PlanningSceneInfo` metadata: the `RobotModelConfig`
and resolved planning groups. Runtime joint state is
then pushed through `VisualizationStateFrame` updates so renderers do not poll
world/module state or own freshness policy. Embedded Meshcat visualization does
not need extra setup because it observes the Drake world directly.

Previews use the stored synchronized `JointTrajectory` from the generated plan.
Viser plays the stored canonical trajectory directly; optional preview duration
only scales the stored delays. Execution forwards that same accepted trajectory
with unchanged joint names, ordering, timestamps, and velocities; it does not
regenerate or retime it. Execute freshness is enforced by the
manipulation module/operator immediately before dispatch, not by Viser-side
telemetry snapshots.

### Perception + Agent

```bash
# Coordinator + perception + manipulation + LLM agent (single command)
XARM7_IP=<ip> dimos run coordinator-xarm7 xarm-perception-agent
```

For a simulation walkthrough, see [Agentic xArm simulation](/docs/capabilities/manipulation/agentic.md).

## Architecture

```
KeyboardTeleopModule ──→ ControlCoordinator ──→ ManipulationModule
  (pygame UI)              (100Hz tick loop)      (WorldSpec backend)
       │                        │                       │
  TwistStamped           EEFTwistTask             RRT planner
  spatial EEF twist      (control IK)             JacobianIK
                               │                   DrakeWorld
                          JointState ────────────→ (visualization)
```

- **KeyboardTeleopModule**: Pygame UI publishing routed spatial EEF twist intent
- **ControlCoordinator**: 100Hz control loop with mock or real hardware adapters
- **ManipulationModule**: world backend, optional visualization, RRT motion planning, obstacle management

### Streaming pose-target control

`CartesianIKTask` and `TeleopIKTask` are sibling leaves over the shared
`PoseTargetIKTask` control core. Their configuration uses a `RobotModelConfig`,
explicit controlled `joint_names`, and named target frames. The common core
warm-starts one bounded Pink update from live coordinator joint state on each
tick; it does not require a planning world or expose planning groups to the
coordinator.

Cartesian IK accepts one absolute robot-frame target. Quest IK accepts one or
two controller-to-frame bindings and owns engagement, reference capture,
relative target mapping, and optional per-hand gripper commands. The
coordinator only routes the distinct left/right pose streams by task name and
arbitrates the resulting joint command.

### Robot-specific Pink task stacks

For robot-specific control feel, subclass `PinkPoseTargetSolver`, override its
task-construction hooks, and pass the class through `solver_type`. The
coordinator constructs a fresh stateful solver for every control task. See
[Pink IK Configuration and Tuning](/docs/capabilities/manipulation/pink_ik_tuning.md)
for the supported hooks, objective tuning, command bounds, and hardware test
order.

Internally, planning code depends on `WorldSpec` for world, collision, and
kinematics behavior. Meshcat preview and publishing are exposed separately
through `VisualizationSpec`, so non-visual planning paths do not require a
visualization backend.

All `WorldSpec` obstacle operations are runtime operations and require the
world to be finalized first. `update_obstacle(obstacle)` replaces the complete
obstacle identified by `obstacle.name`; callers must provide every geometry and
appearance field. `update_obstacle_pose(name, pose)` is the pose-only fast path
and preserves the other fields. Each update is serialized with native scene
queries, so collision checking sees either the old obstacle or the new one,
never the remove/add intermediate state. This boundary applies to each native
operation, not to an entire generic planning run; RoboPlan's opaque native
planner is locked for its whole native call.

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `keyboard-teleop-a750` | A750 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-a1z` | Galaxea A1Z keyboard teleop, planning, and hardware control |
| `keyboard-teleop-piper` | Piper 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm6` | XArm6 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm7` | XArm7 7-DOF keyboard teleop with Drake viz |
| `xarm7-planner-coordinator` | XArm7 planner with coordinator integration |
| `dual-xarm6-planner-coordinator` | Dual XArm6 planning with mock coordinator hardware |
| `xarm-perception` | XArm7 + RealSense camera for perception |
| `xarm-perception-agent` | XArm7 perception + LLM agent |
| `xarm-perception-sim` | XArm7 simulation perception stack |
| [`xarm-perception-sim-agent`](/docs/capabilities/manipulation/agentic.md) | XArm7 simulation perception stack + LLM agent |

## Supported Robots

| Robot | DOF | Teleop | Planning | Perception |
|-------|-----|--------|----------|------------|
| [A-750](/docs/capabilities/manipulation/a750.md) | 6 | Y | Y | N |
| [Galaxea A1Z](/docs/capabilities/manipulation/a1z.md) | 6 | Y | Y | N |
| Piper | 6 | Y | Y | N |
| XArm6 | 6 | Y | Y | N |
| XArm7 | 7 | Y | Y | Y |

## Adding a Custom Arm

[guide is here](/docs/capabilities/manipulation/adding_a_custom_arm.md)

## Key Files

| File | Description |
|------|-------------|
| [`manipulation_module.py`](/dimos/manipulation/manipulation_module.py) | Main module (RPC interface, state machine) |
| [`robot/manipulators/common/blueprints.py`](/dimos/robot/manipulators/common/blueprints.py) | Shared coordinator, planner, and task helpers |
| [`robot/manipulators/a750/config.py`](/dimos/robot/manipulators/a750/config.py) | A-750 model and hardware config |
| [`robot/manipulators/a750/blueprints/teleop.py`](/dimos/robot/manipulators/a750/blueprints/teleop.py) | A-750 keyboard teleop blueprint |
| [`robot/manipulators/piper/blueprints/basic.py`](/dimos/robot/manipulators/piper/blueprints/basic.py) | Piper coordinator blueprint |
| [`robot/manipulators/piper/blueprints/teleop.py`](/dimos/robot/manipulators/piper/blueprints/teleop.py) | Piper teleop blueprints |
| [`robot/manipulators/xarm/blueprints/basic.py`](/dimos/robot/manipulators/xarm/blueprints/basic.py) | XArm coordinator and planner blueprints |
| [`robot/manipulators/xarm/blueprints/perception.py`](/dimos/robot/manipulators/xarm/blueprints/perception.py) | XArm perception blueprint |
| [`teleop/keyboard/keyboard_teleop_module.py`](/dimos/teleop/keyboard/keyboard_teleop_module.py) | Keyboard teleop module |
| [`planning/world/drake_world.py`](/dimos/manipulation/planning/world/drake_world.py) | Drake physics backend |
| [`planning/world/roboplan_world.py`](/dimos/manipulation/planning/world/roboplan_world.py) | RoboPlan scene, state, and collision backend |
| [`planning/planners/roboplan_planner.py`](/dimos/manipulation/planning/planners/roboplan_planner.py) | RoboPlan-native joint and Cartesian planner |
| [`planning/planners/rrt_planner.py`](/dimos/manipulation/planning/planners/rrt_planner.py) | RRT-Connect motion planner |
