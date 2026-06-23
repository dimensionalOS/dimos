# Manipulation

Motion planning and teleoperation for robotic manipulators. Uses Drake for physics simulation and optional Meshcat or Viser planning visualization.

## Quick Start

Recent addition: the A-750 keyboard teleop blueprint is now available via:

```bash
dimos run keyboard-teleop-a750
```

### Keyboard Teleop (single command)

Each blueprint launches the full stack — keyboard UI, mock controller, IK solver, and Drake visualization:

```bash
dimos run keyboard-teleop-a750    # A-750 6-DOF
dimos run keyboard-teleop-piper   # Piper 6-DOF
dimos run keyboard-teleop-xarm6   # XArm6 6-DOF
dimos run keyboard-teleop-xarm7   # XArm7 7-DOF
```

Open the Meshcat URL printed in the terminal (default `http://localhost:7000`) to see the robot.

Keyboard controls:

| Key | Action |
|-----|--------|
| W/S | +X/-X (forward/back) |
| A/D | -Y/+Y (left/right) |
| Q/E | +Z/-Z (up/down) |
| R/F | +Roll/-Roll |
| T/G | +Pitch/-Pitch |
| Y/H | +Yaw/-Yaw |
| SPACE | Reset to home pose |
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
  -o manipulationmodule.kinematics.backend=pink \
  -o manipulationmodule.kinematics.max_iterations=100 \
  -o manipulationmodule.kinematics.dt=0.02
```

For blueprints that instantiate `PickAndPlaceModule`, use the corresponding
module prefix:

```bash
dimos run xarm-perception-sim \
  -o pickandplacemodule.kinematics.backend=pink
```

Then use the IPython client:

```bash
python -m dimos.manipulation.planning.examples.manipulation_client
```

```python skip
joints()                # Get current joints
plan([0.1] * 7)         # Plan to target
preview()               # Preview in Meshcat
execute()               # Execute via coordinator
```

### Rigid vs compliant joint trajectories

Manipulator coordinator stacks can execute either rigid joint trajectories or
compliant joint trajectories:

- `trajectory` keeps the existing rigid behavior: the trajectory sampler emits
  the final `SERVO_POSITION` command directly.
- `compliant_trajectory` keeps the same nominal trajectory interface, then runs
  an internal joint-compliance stage before the final coordinator command.

The compliant task is still one coordinator-facing task. It does not create a
coordinator-level task graph, and the trajectory and compliance stages do not
compete for the same joints during arbitration.

Version 1 is position-servo only. It reads joint position feedback and uses
effort feedback only when explicitly enabled in task params; joint velocity
feedback is not part of the v1 compliance law. This is deliberate: some arm
adapters expose placeholder zero effort values, so zero torque readings are not
treated as reliable contact data by default. The generic manipulator route also
does not require torque output.

Compliance behavior is bounded by per-joint virtual mass, damping, stiffness,
maximum offset, and maximum offset velocity. If required joint position state is
missing, the transform suppresses output instead of inventing offsets. If `dt`
is invalid, it safely passes the nominal reference through for that tick. Offset
and saturation diagnostics are available from the compliant task for tests and
debugging.

Blueprint helpers can select the behavior explicitly:

```python skip
from dimos.robot.manipulators.common.blueprints import (
    compliant_trajectory_task,
    trajectory_task,
)

rigid = trajectory_task(arm_hw)

compliant = compliant_trajectory_task(
    arm_hw,
    params={
        "stiffness": 20.0,
        "damping": 6.0,
        "max_offset": 0.1,
        "max_offset_velocity": 0.4,
        "use_effort_feedback": False,
    },
)
```

#### MuJoCo verification

Use the dedicated xArm7 obstacle scene as the canonical manipulator simulation
entry point for compliant trajectory checks:

```bash
uv run dimos --simulation run xarm7-compliant-obstacle-sim
```

The scene places a fixed wall near the robot's forward workspace and registers a
`compliant_trajectory` task named `traj_arm`. For a free-space smoke check, run a
trajectory that stays in front of the wall; for contact tuning, run a trajectory
whose nominal target reaches behind the translucent marker. Expected pass/fail
observations:

- Free space: compliant offsets stay near zero and the final joint error remains
  comparable to the rigid trajectory.
- Rigid contact or obstruction: compliant offset grows only within configured
  limits, saturation is observable in diagnostics, and the command does not keep
  integrating through the bound.
- Placeholder effort feedback: leave `use_effort_feedback=False` unless the
  adapter is known to expose reliable effort readings for the scenario.

Rigid table/obstacle and soft-contact demos should compare joint-space metrics
such as peak effort, tracking error, offset, and saturation time. These demos are
useful for tuning, but deterministic unit tests remain the source of truth for
safe invalid-`dt`, missing-state, and saturation behavior.

### Planning Visualization

Manipulation visualization is configured on `ManipulationModuleConfig.visualization`.
It is independent from the global Rerun stream viewer in `docs/usage/visualization.md`.

Backend choices:

- `meshcat`: embedded Drake/Meshcat visualizer. The planning world must be created with
  embedded visualization enabled, so this is selected through the visualization config.
- `viser`: in-process Viser visualizer. It renders current robot state, target controls,
  transient preview ghosts, planned path previews, and optional panel controls.
- `none`: no manipulation planning visualization.

CLI example:

```bash
uv run dimos run xarm7-planner-coordinator \
  -o manipulationmodule.visualization.backend=viser \
  -o manipulationmodule.visualization.allow_plan_execute=true
```

Blueprint example:

```python skip
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig

manipulation = ManipulationModule.blueprint(
    config=ManipulationModuleConfig(
        robots=[...],
        visualization={
            "backend": "viser",
            "host": "127.0.0.1",
            "port": 8095,
            "open_browser": True,
            "panel_enabled": True,  # default; set False for scene-only Viser
            "allow_plan_execute": False,  # keep panel execution blocked by default
        },
    )
)
```

Viser support is included in the `manipulation` extra:

```bash
uv sync --extra manipulation --inexact
```

The Viser panel uses existing manipulation planning, preview, execute, cancel, and clear-plan
RPC methods through a small in-process adapter. GUI callbacks enqueue operations instead of
touching `WorldSpec`, IK, planner objects, or live Drake contexts directly. Rendering copies
mutable joint state/path containers at the read boundary, then updates the Viser scene after
manipulation/world accessors have returned.

External manipulation visualizers are initialized from a backend-neutral planning-scene snapshot
after the planning world has added its robots. This snapshot maps world robot IDs to
`RobotModelConfig` metadata so Viser can prepare current, target, and transient preview robot
visuals without `WorldMonitor` depending on Viser-specific hooks. Embedded Meshcat visualization
does not need extra setup because it observes the Drake world directly.

Panel execution is opt-in. Leave `allow_plan_execute=False` unless the operator intentionally
wants the browser panel to call the existing manipulation execution path.

### Perception + Agent

```bash
# Coordinator + perception + manipulation + LLM agent (single command)
XARM7_IP=<ip> dimos run coordinator-xarm7 xarm-perception-agent
```

## Architecture

```
KeyboardTeleopModule ──→ ControlCoordinator ──→ ManipulationModule
  (pygame UI)              (100Hz tick loop)      (Drake + Meshcat)
       │                        │                       │
  PoseStamped            CartesianIK task         RRT planner
  commands               (Pinocchio IK)           JacobianIK
                              │                   DrakeWorld
                         JointState ────────────→ (visualization)
```

- **KeyboardTeleopModule** — Pygame UI publishing cartesian pose commands
- **ControlCoordinator** — 100Hz control loop with mock or real hardware adapters
- **ManipulationModule** — Drake physics, Meshcat viz, RRT motion planning, obstacle management

Internally, planning code depends on `WorldSpec` for world, collision, and
kinematics behavior. Meshcat preview and publishing are exposed separately
through `VisualizationSpec`, so non-visual planning paths do not require a
visualization backend.

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `keyboard-teleop-a750` | A750 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-piper` | Piper 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm6` | XArm6 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm7` | XArm7 7-DOF keyboard teleop with Drake viz |
| `xarm6-planner-only` | XArm6 standalone planner (no coordinator) |
| `xarm7-planner-coordinator` | XArm7 planner with coordinator integration |
| `dual-xarm6-planner` | Dual XArm6 planning |
| `xarm-perception` | XArm7 + RealSense camera for perception |
| `xarm-perception-agent` | XArm7 perception + LLM agent |
| `xarm-perception-sim` | XArm7 simulation perception stack |

## Supported Robots

| Robot | DOF | Teleop | Planning | Perception |
|-------|-----|--------|----------|------------|
| [A-750](/docs/capabilities/manipulation/a750.md) | 6 | Y | Y | — |
| Piper | 6 | Y | Y | — |
| XArm6 | 6 | Y | Y | — |
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
| [`planning/planners/rrt_planner.py`](/dimos/manipulation/planning/planners/rrt_planner.py) | RRT-Connect motion planner |
