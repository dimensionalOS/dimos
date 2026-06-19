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
| [`manipulation/blueprints.py`](/dimos/manipulation/blueprints.py) | Planner and perception blueprints |
| [`robot/manipulators/a750/blueprints.py`](/dimos/robot/manipulators/a750/blueprints.py) | A-750 keyboard teleop blueprint |
| [`robot/manipulators/piper/blueprints.py`](/dimos/robot/manipulators/piper/blueprints.py) | Piper keyboard teleop blueprint |
| [`robot/manipulators/xarm/blueprints.py`](/dimos/robot/manipulators/xarm/blueprints.py) | XArm keyboard teleop blueprints |
| [`teleop/keyboard/keyboard_teleop_module.py`](/dimos/teleop/keyboard/keyboard_teleop_module.py) | Keyboard teleop module |
| [`planning/world/drake_world.py`](/dimos/manipulation/planning/world/drake_world.py) | Drake physics backend |
| [`planning/planners/rrt_planner.py`](/dimos/manipulation/planning/planners/rrt_planner.py) | RRT-Connect motion planner |
