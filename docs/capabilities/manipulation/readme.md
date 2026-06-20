# Manipulation

Motion planning and teleoperation for robotic manipulators. Drake remains the default
world backend and Meshcat is the default manipulation visualization path.

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

Select a non-default IK backend with nested module config overrides:

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
- `kinematics_name` selects the IK backend.

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

| `world_backend` | `planner_name` | `kinematics_name` | Status |
|-----------------|----------------|-------------------|--------|
| `drake` | `rrt_connect` | `jacobian` | Default path |
| `drake` | `rrt_connect` | `drake_optimization` | Drake-only IK |
| `roboplan` | `rrt_connect` | `jacobian` | Generic RRT over RoboPlan collision checks |
| `roboplan` | `roboplan` | `jacobian` | RoboPlan-native planner, using the RoboPlan world object |

Invalid combinations fail during startup instead of waiting for the first plan
request. For example, `planner_name=roboplan` requires
`world_backend=roboplan`, and `kinematics_name=drake_optimization` requires
`world_backend=drake`.

Install RoboPlan together with the default manipulation dependencies:

```bash
uv sync --extra manipulation-roboplan --inexact
```

The RoboPlan manipulation extra includes the default `manipulation` dependencies
and installs `roboplan-dimos` from PyPI.
The `--inexact` flag preserves other extras already installed in your current
environment.

RoboPlan builds C++ bindings locally. Install system build prerequisites before
syncing the extra; for example, on Ubuntu:

```bash
sudo apt-get install libeigen3-dev
```

If CMake reports another missing package, install the matching development
package or expose its CMake package directory through `CMAKE_PREFIX_PATH`.

Safety behavior for unsupported RoboPlan features:

- Planning-critical unsupported inputs fail loudly before planning. Examples
  include unsupported obstacle geometry, unavailable robot loading APIs, or
  unavailable collision query APIs. RoboPlan worlds generate a minimal SRDF from
  the DimOS robot config, including configured collision-exclusion pairs.
- Unverified non-critical query methods raise explicit `NotImplementedError`.
  In particular, signed minimum-distance semantics are not implemented for
  RoboPlan until a safe equivalent is verified.
- RoboPlan manipulation visualization is not implemented; Drake/Meshcat remains
  the visualization path.

### Perception + Agent

```bash
# Coordinator + perception + manipulation + LLM agent (single command)
XARM7_IP=<ip> dimos run coordinator-xarm7 xarm-perception-agent
```

## Architecture

```
KeyboardTeleopModule ──→ ControlCoordinator ──→ ManipulationModule
  (pygame UI)              (100Hz tick loop)      (WorldSpec backend)
       │                        │                       │
  PoseStamped            CartesianIK task         RRT planner
  commands               (Pinocchio IK)           JacobianIK
                              │                   DrakeWorld
                         JointState ────────────→ (visualization)
```

- **KeyboardTeleopModule** — Pygame UI publishing cartesian pose commands
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
