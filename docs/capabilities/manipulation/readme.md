# Manipulation

Motion planning and teleoperation for robotic manipulators. Drake is the default planning backend for physics simulation and Meshcat visualization; RoboPlan can be selected as an optional manipulation planning backend for RoboPlan-native scene loading, RRT planning, IK, and optional TOPPRA timing.

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

### Viser Operator Panel (optional)

The Viser manipulation panel is a companion operator UI for a running
`ManipulationModule`. It is optional and installed separately from the base
manipulation extra:

```bash
uv sync --extra manipulation-viser
```

This extra installs Viser with URDF support, including `yourdfpy`, which is
required to render robot models with `ViserUrdf`.

Start a manipulation stack first, then launch the panel in another terminal:

```bash
python -m dimos.manipulation.viser_panel --host 127.0.0.1 --port 8095
```

For a single-command mock test stack with no hardware, run:

```bash
uv run --extra manipulation-viser dimos run xarm7-viser-panel-mock
```

Open `http://127.0.0.1:8095` unless `--open-browser` is passed. The panel shows
a disconnected state and keeps planning/execution controls disabled until it can
reach a compatible `ManipulationModule` over RPC.

The panel keeps the live robot and target visible at the same time:

- the current robot reflects live joint state;
- the target ghost follows the current target and turns red when infeasible;
- the end-effector transform gizmo is separate from both robot meshes;
- Cartesian target movement and joint sliders are both visible and stay
  synchronized through IK/FK preview RPCs;
- Current, Init, and Home presets apply once, then the selector returns to its
  neutral state.

Planning, local Viser preview, execution, cancel, and clear plan use public
`ManipulationModule` RPCs. Hardware execution is opt-in: pass `--allow-execute`
and verify the target and plan are still fresh before Execute is enabled. Do not
execute on hardware until the same operation has been checked in mock or
simulation and Cancel remains visible.

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

## Optional RoboPlan Backend

Manipulation stacks can opt into RoboPlan without changing the public RPC, skill, preview, or execute method names:

```python skip
ManipulationModule(
    robots=[robot_config],
    planning_backend="roboplan",
    planning_backend_options={
        "roboplan": {
            "srdf_path": "/path/to/robot.srdf",
            "planning_group": "arm",
            "active_joint_names": robot_config.joint_names,
            "base_frame": robot_config.base_link,
            "end_effector_frame": robot_config.end_effector_link,
            "retiming": "dimos",
        }
    },
)
```

```error
Traceback (most recent call last):
  File "/tmp/tmpya5ckocy.py", line 1, in <module>
    ManipulationModule(
    ^^^^^^^^^^^^^^^^^^
NameError: name 'ManipulationModule' is not defined

Exit code: 1
```

RoboPlan is optional and lazy-imported. Default Drake-backed manipulation stacks still start if RoboPlan is not installed. When RoboPlan is selected, the robot config must provide the URDF path, SRDF path, package paths for meshes when needed, joint order, base and end-effector frames, and complete joint limits.

RoboPlan changes planning only. Real robot execution still goes through the existing `ControlCoordinator` trajectory task path, including coordinator joint-name translation and execution state handling. Validate RoboPlan configurations in mock or simulation before supervised hardware use.

Verify required RoboPlan modules with:

```bash
python -c "import roboplan.core, roboplan.rrt, roboplan.simple_ik"
```

For `retiming="toppra"`, also verify:

```bash
python -c "import roboplan.toppra"
```

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `keyboard-teleop-a750` | A750 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-piper` | Piper 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm6` | XArm6 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm7` | XArm7 7-DOF keyboard teleop with Drake viz |
| `xarm6-planner-only` | XArm6 standalone planner (no coordinator) |
| `xarm7-planner-coordinator` | XArm7 planner with coordinator integration |
| `xarm7-viser-panel-mock` | Mock XArm7 planner + coordinator + Viser operator panel |
| `dual-xarm6-planner` | Dual XArm6 planning |
| `xarm-perception` | XArm7 + RealSense camera for perception |
| `xarm-perception-agent` | XArm7 perception + LLM agent |

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
| [`viser_panel/module.py`](/dimos/manipulation/viser_panel/module.py) | Optional Viser operator panel module over the manipulation RPC interface |
| [`manipulation/blueprints.py`](/dimos/manipulation/blueprints.py) | Planner and perception blueprints |
| [`robot/manipulators/a750/blueprints.py`](/dimos/robot/manipulators/a750/blueprints.py) | A-750 keyboard teleop blueprint |
| [`robot/manipulators/piper/blueprints.py`](/dimos/robot/manipulators/piper/blueprints.py) | Piper keyboard teleop blueprint |
| [`robot/manipulators/xarm/blueprints.py`](/dimos/robot/manipulators/xarm/blueprints.py) | XArm keyboard teleop blueprints |
| [`teleop/keyboard/keyboard_teleop_module.py`](/dimos/teleop/keyboard/keyboard_teleop_module.py) | Keyboard teleop module |
| [`planning/world/drake_world.py`](/dimos/manipulation/planning/world/drake_world.py) | Drake physics backend |
| [`planning/planners/rrt_planner.py`](/dimos/manipulation/planning/planners/rrt_planner.py) | RRT-Connect motion planner |
| [`planning/backends/roboplan/backend.py`](/dimos/manipulation/planning/backends/roboplan/backend.py) | Optional RoboPlan active backend |
