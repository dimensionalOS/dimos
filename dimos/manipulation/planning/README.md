# Manipulation Planning Stack

Motion planning for robotic manipulators. The stack separates geometric path
planning from conversion to an executable timed trajectory.

## Quick Start

```bash
# 1. Verify manipulation dependencies with mock hardware:
dimos run xarm7-planner-coordinator

# 2. Keyboard teleop with mock arm (single command):
dimos run keyboard-teleop-xarm7

# 3. Interactive RPC client (plan, preview, execute from Python):
dimos run xarm7-planner-coordinator                                    # terminal 1
python -i -m dimos.manipulation.planning.examples.manipulation_client  # terminal 2
```

In the interactive client:
```python skip
commands()              # List available commands
joints()                # Get current joint positions
plan([0.1] * 7)         # Plan to target
preview()               # Preview in Meshcat (url() for link)
execute()               # Execute via coordinator
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ManipulationModule                       │
│       (RPC interface, state machine, one robot model)       │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│              Backend-Agnostic Components                    │
│  ┌──────────────────┐  ┌─────────────────────────────┐     │
│  │ RRTConnectPlanner│  │ JacobianIK                  │     │
│  │ (rrt_planner.py) │  │ (iterative & differential) │     │
│  └──────────────────┘  └─────────────────────────────┘     │
│              Uses only WorldSpec interface                  │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                    WorldSpec Protocol                       │
│  Context management, collision checking, FK, Jacobian       │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│               Backend-Specific Implementations              │
│  ┌──────────────────┐  ┌─────────────────────────────┐     │
│  │ DrakeWorld       │  │ DrakeOptimizationIK         │     │
│  │ (physics/viz)    │  │ (nonlinear IK)              │     │
│  └──────────────────┘  └─────────────────────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

## Using ManipulationModule

```python skip
from dimos.manipulation import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel

config = RobotModelConfig(
    model=RobotModel.from_file("/path/to/xarm7.urdf"),
    base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
    base_link="link_base",
    planning_groups=[
        PlanningGroupDefinition(
            name="arm",
            joint_names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"),
            base_link="link_base",
            tip_link="link7",
        )
    ],
)

module = ManipulationModule(
    model=config,
    planning_timeout=10.0,
    enable_viz=True,
    world_backend="drake",                # RoboPlan is the default
    planner={"backend": "rrt_connect"},    # RoboPlan is the default
    trajectory_parametrization={"backend": "simple_trapezoid"},
    kinematics={"backend": "drake_optimization"}, # Or "jacobian" / "pink"
)
module.start()
module.plan_to_joints(
    {"arm": JointState(position=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])}
)
module.execute()  # Sends to coordinator
```

## Path-to-Trajectory Lifecycle

A joint-space planner normally returns an untimed geometric path. Before DimOS
accepts a `GeneratedPlan`, the one trajectory-parametrization backend selected
at startup converts that path into a timed `JointTrajectory`. DimOS then
validates joint ordering, dimensions, finite values, strictly increasing time,
and start and goal preservation. Each backend is responsible for generating
motion within the velocity and acceleration limits it receives. A failure
leaves no executable plan cached.

When `trajectory_parametrization` is omitted, the world selects the matching
default: `RoboPlanWorld` uses `roboplan_toppra`, while `DrakeWorld` uses
`simple_trapezoid`. An explicit backend always overrides that default.

This boundary is exposed internally as `TrajectoryParametrizerSpec`, alongside
`PlannerSpec` and `WorldSpec`. Its implementations own conversion, validation,
and `GeneratedPlan` construction; `ManipulationModule` only supplies the world,
selected planning groups, planning result, and next-plan speed.

A planner may instead return a trajectory that already contains timestamps and
velocities. That result is already on the trajectory side of the boundary, so
DimOS skips parametrization, preserves its timing, and applies the same
canonical structural validation. This is not fallback: a failure of the
selected parametrizer never invokes another backend.

Preview and execution both consume the accepted stored trajectory. The same
canonical joint names and ordering flow through planning, visualization, and
coordinator execution without renaming, splitting, or retiming.

When Viser is enabled, its **Next plan speed** slider selects a runtime
reduction from `0.05` to `1.0`. The value multiplies the configured velocity
and acceleration scales for the next plan. It does not modify the currently
accepted plan: move the slider, then press **Plan** again. Joint-space paths
apply the value during trajectory parametrization; Viser Cartesian requests
pass it to the native planner before that planner produces timestamps.

## Trajectory Parametrization

The compatibility backend retains the existing segmented trapezoidal behavior:

```python skip
ManipulationModuleConfig(
    trajectory_parametrization={
        "backend": "simple_trapezoid",
        "velocity_scale": 1.0,
        "acceleration_scale": 1.0,
        "points_per_segment": 50,
    },
)
```

RoboPlan TOPP-RA produces continuous timing across a geometric path:

```python skip
ManipulationModuleConfig(
    world_backend="roboplan",
    trajectory_parametrization={
        "backend": "roboplan_toppra",
        "output_period": 0.01,
        "velocity_scale": 0.8,
        "acceleration_scale": 0.8,
    },
)
```

DimOS uses zero-deviation linear fitting so parametrization changes timing,
not the collision-checked geometric path.

`roboplan_toppra` can parametrize a geometric path from any planner, but only
when `world_backend="roboplan"`: it reuses the finalized `RoboPlanWorld` model
and planning groups. Selecting it with another world fails during startup.
DimOS supports RoboPlan `0.6.x` for this integration.

For every selected movable joint, the RoboPlan URDF must provide a finite,
positive velocity limit. DimOS uses an authored extended acceleration limit
when present; otherwise it temporarily inserts a default `2.0 rad/s²` limit
while preparing the RoboPlan model. Formal per-joint acceleration overrides
will replace this fallback.

```xml
<limit
  lower="-3.14"
  upper="3.14"
  effort="100"
  velocity="2.0"
  acceleration="4.0"
/>
```

RoboPlan scene limits are authoritative for this backend. The current
`RobotModelConfig.max_velocity`, `velocity_limits`, and `max_acceleration`
fields are not substituted when a URDF limit is missing. Missing or invalid
limits fail plan materialization with the affected joint named. Formal
canonical per-joint overrides are future work.

## RobotModelConfig Fields

| Field | Description |
|-------|-------------|
| `model` | Lazy portable robot model |
| `base_pose` | PoseStamped for robot base in world frame |
| `joint_names` | Canonical joint names in the model |
| `base_link` | Base link name |
| `planning_groups` | Named planning subsets with canonical joints and frames |
| `max_velocity` | Max joint velocity (rad/s) |
| `max_acceleration` | Max acceleration (rad/s²) |

## Components

### Planners (Backend-Agnostic)

| Planner | Description |
|---------|-------------|
| `RRTConnectPlanner` | Bi-directional RRT-Connect (fast, reliable) |

### IK Solvers

| Solver | Type | Description |
|--------|------|-------------|
| `JacobianIK` | Backend-agnostic | Iterative damped least-squares |
| `DrakeOptimizationIK` | Drake-specific | Full nonlinear optimization |
| `PinkIK` | Pinocchio/Pink | Local differential IK with task QP composition |

`PinkIK` is selectable with `kinematics={"backend": "pink"}` or the CLI override
`--kinematics.backend=pink`. Pink tuning fields are nested
under the same config, for example
`--kinematics.max-iterations=100`. It is installed with the
`manipulation` optional dependencies, which include the PyPI package `pin-pink`
(import name `pink`) and a `qpsolvers` backend (`proxqp`). Pink is
local/differential rather than global IK, so it can converge to local minima;
collision checks remain enforced by the planning world before a candidate is
accepted.

### World Backends

| Backend | Description |
|---------|-------------|
| `DrakeWorld` | Drake physics with Meshcat visualization |
| `RoboPlanWorld` | RoboPlan model, collision scene, native planner, and TOPP-RA support |

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `xarm7-planner-coordinator` | XArm 7-DOF with coordinator |
| `dual-xarm6-planner-coordinator` | Dual XArm 6-DOF with mock coordinator hardware |
| `xarm-perception-sim` | XArm 7-DOF simulation perception stack |

## Directory Structure

```
planning/
├── spec.py                  # Protocols (WorldSpec, KinematicsSpec, PlannerSpec)
├── factory.py               # create_world, create_kinematics, create_planner
├── world/
│   └── drake_world.py       # DrakeWorld implementation
├── kinematics/
│   ├── jacobian_ik.py       # Backend-agnostic Jacobian IK
│   ├── drake_optimization_ik.py  # Drake nonlinear IK
│   └── pink_ik.py           # Optional Pink differential IK
├── planners/
│   └── rrt_planner.py       # RRTConnectPlanner
├── monitor/                 # WorldMonitor (live state sync)
├── trajectory_generator/    # Time-parameterized trajectories
└── examples/
    └── manipulation_client.py    # Interactive RPC client (python -i)
```

## Obstacle Types

| Type | Dimensions |
|------|------------|
| `BOX` | (width, height, depth) |
| `SPHERE` | (radius,) |
| `CYLINDER` | (radius, height) |
| `MESH` | mesh_path |

## Supported Robots

| Robot | DOF |
|-------|-----|
| `piper` | 6 |
| `xarm6` | 6 |
| `xarm7` | 7 |

## Testing

```bash
# Unit tests (fast, no Drake)
pytest dimos/manipulation/test_manipulation_unit.py -v

# Integration tests (requires Drake)
pytest dimos/e2e_tests/test_manipulation_module.py -v
```
