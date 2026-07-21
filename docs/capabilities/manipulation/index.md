---
title: "Manipulation"
---

Motion planning and teleoperation for robotic manipulators. Drake remains the default
world backend, RoboPlan is available as an optional planning backend, and
manipulation visualization supports Meshcat or Viser.

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

### Planning backend selection

Manipulation planning separates the world backend from the planner algorithm:

- `world_backend` selects the robot/world/collision representation.
- `planner_name` selects the path-planning algorithm.
- `kinematics.backend` selects the IK backend. The legacy `kinematics_name`
  field remains available as a compatibility shim.

Drake remains the default:

```bash
dimos run xarm7-planner-coordinator
```

RoboPlan is available as an optional backend for evaluating a non-Drake world
implementation. Select it explicitly with module options:

```bash
dimos run xarm7-planner-coordinator \
  -o manipulationmodule.world_backend=roboplan \
  -o manipulationmodule.planner_name=rrt_connect
```

Valid combinations:

| `world_backend` | `planner_name` | `kinematics.backend` | Status |
|-----------------|----------------|-------------------|--------|
| `drake` | `rrt_connect` | `pink` | Default path |
| `drake` | `rrt_connect` | `jacobian` | Legacy Jacobian IK |
| `drake` | `rrt_connect` | `drake_optimization` | Drake-only IK |
| `roboplan` | `rrt_connect` | `pink` or `jacobian` | Generic RRT over RoboPlan collision checks |
| `roboplan` | `roboplan` | `pink` or `jacobian` | RoboPlan-native planner, using the RoboPlan world object |

Invalid combinations fail during startup instead of waiting for the first plan
request. For example, `planner_name=roboplan` requires
`world_backend=roboplan`, and `kinematics.backend=drake_optimization` requires
`world_backend=drake`.

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
  the DimOS robot config, including configured collision-exclusion pairs.
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
- `viser`: in-process Viser visualizer. It renders current robot state, target controls,
  transient preview ghosts, planned path previews, and optional panel controls.
- `none`: no manipulation planning visualization.

CLI example:

```bash
uv run dimos run xarm7-planner-coordinator \
  -o manipulationmodule.visualization.backend=viser
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

When the Viser panel is enabled, it can call the existing manipulation execution path after a
fresh feasible plan is available and the current robot joints still match the plan start.

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
  spatial EEF twist      (Pinocchio FK/IK)        JacobianIK
                               │                   DrakeWorld
                          JointState ────────────→ (visualization)
```

- **KeyboardTeleopModule** — Pygame UI publishing routed spatial EEF twist intent
- **ControlCoordinator** — 100Hz control loop with mock or real hardware adapters
- **ManipulationModule** — world backend, optional visualization, RRT motion planning, obstacle management

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
| [`xarm-perception-sim-agent`](/docs/capabilities/manipulation/agentic.md) | XArm7 simulation perception stack + LLM agent |

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
