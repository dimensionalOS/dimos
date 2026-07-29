# Manipulation Planning Stack

Motion planning and inverse kinematics for robotic manipulators, with generic
and world-native backends for RoboPlan and Drake.

## Quick Start

```bash
# 1. Verify manipulation dependencies load correctly (standalone, no hardware):
dimos run xarm6-planner-only

# 2. Keyboard teleop with mock arm (single command):
dimos run keyboard-teleop-xarm7

# 3. Interactive RPC client (plan, preview, execute from Python):
dimos run xarm7-planner-coordinator                                    # terminal 1
python -i -m dimos.manipulation.planning.examples.manipulation_client  # terminal 2
```

In the interactive client:
```python
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
│         (RPC interface, state machine, multi-robot)         │
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

```python
from pathlib import Path
from dimos.manipulation import ManipulationModule
from dimos.manipulation.planning.spec import RobotModelConfig

config = RobotModelConfig(
    name="xarm7",
    model_path=Path("/path/to/xarm7.urdf"),
    base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
    end_effector_link="link7",
    base_link="link_base",
    joint_name_mapping={"arm_joint1": "joint1", ...},  # coordinator <-> URDF
    coordinator_task_name="traj_arm",
)

module = ManipulationModule(
    robots=[config],
    planning_timeout=10.0,
    enable_viz=True,
    world_backend="drake",                # RoboPlan is the default
    planner_name="rrt_connect",           # RoboPlan is the default
    # Omit kinematics for the world default, or select an explicit override:
    kinematics={"backend": "drake_optimization"}, # "roboplan" / "jacobian" / "pink"
)
module.start()
module.plan_to_joints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
module.execute()  # Sends to coordinator
```

## RobotModelConfig Fields

| Field | Description |
|-------|-------------|
| `name` | Robot identifier |
| `model_path` | Path to URDF/XACRO file |
| `base_pose` | PoseStamped for robot base in world frame |
| `joint_names` | Joint names in URDF |
| `end_effector_link` | EE link name |
| `base_link` | Base link name |
| `max_velocity` | Max joint velocity (rad/s) |
| `max_acceleration` | Max acceleration (rad/s²) |
| `joint_name_mapping` | Coordinator → URDF name mapping |
| `coordinator_task_name` | Task name for execution RPC |
| `package_paths` | ROS package paths for meshes |
| `xacro_args` | Xacro arguments (e.g., `{"dof": "7"}`) |

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
| RoboPlan OInK | RoboPlan-specific | Native task/QP IK using the RoboPlan scene |

When kinematics configuration is omitted, a RoboPlan world uses native OInK and
other worlds use Pink. Explicit configuration always overrides that policy, so
Pink remains selectable with `kinematics={"backend": "pink"}` or the CLI override
`-o manipulationmodule.kinematics.backend=pink`. Pink tuning fields are nested
under the same config, for example
`-o manipulationmodule.kinematics.max_iterations=100`. It is installed with the
`manipulation` optional dependencies, which include the PyPI package `pin-pink`
(import name `pink`) and a `qpsolvers` backend (`proxqp`). Pink is
local/differential rather than global IK, so it can converge to local minima;
collision checks remain enforced by the planning world before a candidate is
accepted.

RoboPlan OInK is selected explicitly with
`kinematics={"backend": "roboplan"}` and requires a RoboPlan world. It is part
of the normal `roboplan` package in the manipulation extra. The initial backend
supports `world`-frame targets and optional converged-endpoint collision
validation. It does not use a self-collision barrier, and its numerical search
iterations are not an executable path.

### World Backends

| Backend | Description |
|---------|-------------|
| `DrakeWorld` | Drake physics with Meshcat visualization |
| `RoboPlanWorld` | RoboPlan scene, collision queries, native RRT, and native OInK |

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `xarm6_planner_only` | XArm 6-DOF standalone (no coordinator) |
| `xarm7-planner-coordinator` | XArm 7-DOF with coordinator |
| `dual-xarm6-planner` | Dual XArm 6-DOF |
| `xarm-perception-sim` | XArm 7-DOF simulation perception stack |

## Directory Structure

```
planning/
├── spec/protocols.py        # Protocols (WorldSpec, KinematicsSpec, PlannerSpec)
├── factory.py               # create_world, create_kinematics, create_planner
├── world/
│   ├── drake_world.py       # DrakeWorld implementation
│   ├── roboplan_model.py    # RoboPlan model and planning-group mappings
│   ├── roboplan_world.py    # RoboPlan world, planner, and locked IK delegation
│   └── roboplan_oink.py     # Private request-local OInK solver
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
