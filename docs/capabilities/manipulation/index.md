(doc-capabilities-manipulation-index-manipulation)=

# Manipulation

Motion planning and teleoperation for robotic manipulators. Drake remains the default world backend, RoboPlan is available as an optional planning backend, and manipulation visualization supports Meshcat or Viser.

(doc-capabilities-manipulation-index-quick-start)=

## Quick Start

Recent addition: the A-750 keyboard teleop blueprint is now available via:

```bash
dimos run keyboard-teleop-a750
```

(doc-capabilities-manipulation-index-keyboard-teleop-single-command)=

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

| Key | Action               |
| --- | -------------------- |
| W/S | +X/-X (forward/back) |
| A/D | +Y/-Y (left/right)   |
| Q/E | +Z/-Z (up/down)      |
| R/F | +Roll/-Roll          |
| T/G | +Pitch/-Pitch        |
| Y/H | +Yaw/-Yaw            |
| ESC | Quit                 |

(doc-capabilities-manipulation-index-motion-planning-two-terminals)=

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

For blueprints that instantiate [`PickAndPlaceModule`][PickAndPlaceModule], use the corresponding module prefix:

```bash
dimos run xarm-perception-sim \
  -o pickandplacemodule.kinematics.backend=pink
```

Then use the IPython client:

```bash
python -m dimos.manipulation.planning.examples.manipulation_client
```

```python
joints()                # Get current joints
plan([0.1] * 7)         # Plan to target
preview()               # Preview in Meshcat
execute()               # Execute via coordinator
```

(doc-capabilities-manipulation-index-planning-backend-selection)=

### Planning backend selection

Manipulation planning separates the world backend from the planner algorithm:

- `world_backend` selects the robot/world/collision representation.
- `planner_name` selects the path-planning algorithm.
- `kinematics.backend` selects the IK backend. The legacy `kinematics_name` field remains available as a compatibility shim.

Drake remains the default:

```bash
dimos run xarm7-planner-coordinator
```

RoboPlan is available as an optional backend for evaluating a non-Drake world implementation. Select it explicitly with module options:

```bash
dimos run xarm7-planner-coordinator \
  -o manipulationmodule.world_backend=roboplan \
  -o manipulationmodule.planner_name=rrt_connect
```

Valid combinations:

| `world_backend` | `planner_name` | `kinematics.backend` | Status                                                   |
| --------------- | -------------- | -------------------- | -------------------------------------------------------- |
| `drake`         | `rrt_connect`  | `pink`               | Default path                                             |
| `drake`         | `rrt_connect`  | `jacobian`           | Legacy Jacobian IK                                       |
| `drake`         | `rrt_connect`  | `drake_optimization` | Drake-only IK                                            |
| `roboplan`      | `rrt_connect`  | `pink` or `jacobian` | Generic RRT over RoboPlan collision checks               |
| `roboplan`      | `roboplan`     | `pink` or `jacobian` | RoboPlan-native planner, using the RoboPlan world object |

Invalid combinations fail during startup instead of waiting for the first plan request. For example, `planner_name=roboplan` requires `world_backend=roboplan`, and `kinematics.backend=drake_optimization` requires `world_backend=drake`.

Install the manipulation dependencies:

```bash
uv sync --extra manipulation --inexact
```

The `manipulation` extra includes RoboPlan via `roboplan` from PyPI. The `--inexact` flag preserves other extras already installed in your current environment.

Safety behavior for unsupported RoboPlan features:

- Planning-critical unsupported inputs fail loudly before planning. Examples include unsupported obstacle geometry, unavailable robot loading APIs, or unavailable collision query APIs. RoboPlan worlds generate a minimal SRDF from the DimOS robot config, including configured collision-exclusion pairs.
- Unverified non-critical query methods raise explicit `NotImplementedError`. In particular, signed minimum-distance semantics are not implemented for RoboPlan until a safe equivalent is verified.
- Embedded Meshcat visualization requires a world implementing [`VisualizationSpec`][VisualizationSpec]; use Viser or `none` with the RoboPlan backend.

(doc-capabilities-manipulation-index-planning-visualization)=

### Planning Visualization

Manipulation visualization is configured on `ManipulationModuleConfig.visualization`.
It is independent from the global Rerun stream viewer described in
[Viewer Backends](../../usage/visualization.md).

Backend choices:

- `meshcat`: embedded Drake/Meshcat visualizer. The planning world must be created with embedded visualization enabled, so this is selected through the visualization config.
- `viser`: in-process Viser visualizer. It renders pushed current robot state, target controls, transient preview ghosts, synchronized trajectory previews, and optional panel controls.
- `none`: no manipulation planning visualization.

CLI example:

```bash
uv run dimos run xarm7-planner-coordinator \
  -o manipulationmodule.visualization.backend=viser
```

Blueprint example:

```python
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

The Viser panel talks to the concrete [`ManipulationOperator`][ManipulationOperator] bound into its [`VisualizationSession`][VisualizationSession]. GUI callbacks enqueue operations through that operator for target evaluation, planning, preview, execution, cancellation, reset, and clear-plan actions. The panel owns only target drafts, selection state, and callback generations; it does not touch [`WorldSpec`][WorldSpec], IK, planner objects, [`ManipulationModule`][ManipulationModule], [`WorldMonitor`][WorldMonitor], or live Drake contexts directly.

External manipulation visualizers are initialized from a backend-neutral [`VisualizationSession`][VisualizationSession] after the planning world has added its robots. The session contains static [`PlanningSceneInfo`][PlanningSceneInfo] metadata: world robot IDs, [`RobotModelConfig`][RobotModelConfig] values, and resolved planning groups. Runtime joint state is then pushed through [`VisualizationStateFrame`][VisualizationStateFrame] updates so renderers do not poll world/module state or own freshness policy. Embedded Meshcat visualization does not need extra setup because it observes the Drake world directly.

Previews use the stored synchronized [`JointTrajectory`][JointTrajectory] from the generated plan. Viser projects the globally named trajectory into robot-local preview ghosts and plays the stored timestamped points directly; optional preview duration only scales the stored delays. Execute freshness is enforced by the manipulation module/operator immediately before dispatch, not by Viser-side telemetry snapshots.

(perception-agent)=

(doc-capabilities-manipulation-index-perception-agent)=

### Perception + Agent

```bash
# Coordinator + perception + manipulation + LLM agent (single command)
XARM7_IP=<ip> dimos run coordinator-xarm7 xarm-perception-agent
```

For a simulation walkthrough, see [Agentic xArm simulation](agentic.md).

(doc-capabilities-manipulation-index-architecture)=

## Architecture

```text
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

Internally, planning code depends on [`WorldSpec`][WorldSpec] for world, collision, and kinematics behavior. Meshcat preview and publishing are exposed separately through [`VisualizationSpec`][VisualizationSpec], so non-visual planning paths do not require a visualization backend.

(doc-capabilities-manipulation-index-blueprints)=

## Blueprints

:::{list-table}
   :header-rows: 1

   * - Blueprint
     - Description
   * - ``keyboard-teleop-a750``
     - A750 6-DOF keyboard teleop with Drake visualization
   * - ``keyboard-teleop-piper``
     - Piper 6-DOF keyboard teleop with Drake visualization
   * - ``keyboard-teleop-xarm6``
     - XArm6 6-DOF keyboard teleop with Drake visualization
   * - ``keyboard-teleop-xarm7``
     - XArm7 7-DOF keyboard teleop with Drake visualization
   * - ``xarm6-planner-only``
     - XArm6 standalone planner without a coordinator
   * - ``xarm7-planner-coordinator``
     - XArm7 planner with coordinator integration
   * - ``dual-xarm6-planner``
     - Dual XArm6 planning
   * - ``xarm-perception``
     - XArm7 and RealSense perception
   * - ``xarm-perception-agent``
     - XArm7 perception with an LLM agent
   * - ``xarm-perception-sim``
     - XArm7 simulated perception stack
   * - [xarm-perception-sim-agent](agentic.md)
     - XArm7 simulated perception stack with an LLM agent
:::

(doc-capabilities-manipulation-index-supported-robots)=

## Supported Robots

:::{list-table}
   :header-rows: 1

   * - Robot
     - DOF
     - Teleoperation
     - Planning
     - Perception
   * - [A-750](a750.md)
     - 6
     - Yes
     - Yes
     - —
   * - Piper
     - 6
     - Yes
     - Yes
     - —
   * - XArm6
     - 6
     - Yes
     - Yes
     - —
   * - XArm7
     - 7
     - Yes
     - Yes
     - Yes
:::

(doc-capabilities-manipulation-index-adding-a-custom-arm)=

## Adding a Custom Arm

[guide is here](adding_a_custom_arm.md)

(doc-capabilities-manipulation-index-key-files)=

## Key Files

:::{list-table}
   :header-rows: 1
   :widths: 55 45

   * - File
     - Description
   * - `manipulation_module.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/manipulation/manipulation_module.py>`__
     - Main module (RPC interface and state machine)
   * - `robot/manipulators/common/blueprints.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/manipulators/common/blueprints.py>`__
     - Shared coordinator, planner, and task helpers
   * - `robot/manipulators/a750/config.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/manipulators/a750/config.py>`__
     - A-750 model and hardware config
   * - `robot/manipulators/a750/blueprints/teleop.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/manipulators/a750/blueprints/teleop.py>`__
     - A-750 keyboard teleop blueprint
   * - `robot/manipulators/piper/blueprints/basic.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/manipulators/piper/blueprints/basic.py>`__
     - Piper coordinator blueprint
   * - `robot/manipulators/piper/blueprints/teleop.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/manipulators/piper/blueprints/teleop.py>`__
     - Piper teleop blueprints
   * - `robot/manipulators/xarm/blueprints/basic.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/manipulators/xarm/blueprints/basic.py>`__
     - XArm coordinator and planner blueprints
   * - `robot/manipulators/xarm/blueprints/perception.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/manipulators/xarm/blueprints/perception.py>`__
     - XArm perception blueprint
   * - `teleop/keyboard/keyboard_teleop_module.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/teleop/keyboard/keyboard_teleop_module.py>`__
     - Keyboard teleop module
   * - `planning/world/drake_world.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/manipulation/planning/world/drake_world.py>`__
     - Drake physics backend
   * - `planning/planners/rrt_planner.py <https://github.com/dimensionalOS/dimos/blob/main/dimos/manipulation/planning/planners/rrt_planner.py>`__
     - RRT-Connect motion planner
:::

```{toctree}
:hidden: true
:maxdepth: 1

agentic
adding_a_custom_arm
planning_groups
a750
piper_integration
openarm_integration
```

[PickAndPlaceModule]: #dimos.manipulation.pick_and_place_module.PickAndPlaceModule
[VisualizationSpec]: #dimos.manipulation.planning.spec.protocols.VisualizationSpec
[ManipulationOperator]: #dimos.manipulation.visualization.operator.ManipulationOperator
[VisualizationSession]: #dimos.manipulation.planning.spec.models.VisualizationSession
[WorldSpec]: #dimos.manipulation.planning.spec.protocols.WorldSpec
[ManipulationModule]: #dimos.manipulation.manipulation_module.ManipulationModule
[WorldMonitor]: #dimos.manipulation.planning.monitor.world_monitor.WorldMonitor
[PlanningSceneInfo]: #dimos.manipulation.planning.spec.models.PlanningSceneInfo
[RobotModelConfig]: #dimos.manipulation.planning.spec.config.RobotModelConfig
[VisualizationStateFrame]: #dimos.manipulation.planning.spec.models.VisualizationStateFrame
[JointTrajectory]: #dimos.msgs.trajectory_msgs.JointTrajectory.JointTrajectory
