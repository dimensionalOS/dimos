# Manipulation Planning Stack

Motion planning for robotic manipulators. Backend-agnostic design with Drake implementation.

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
    planner_name="rrt_connect",           # Only option
    kinematics_name="drake_optimization", # Or "jacobian" / "pink"
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

`PinkIK` is selectable with `kinematics_name="pink"`. It is an optional backend:
install it with `uv sync --extra all --extra pink`. The `pink` extra installs the
PyPI package `pin-pink` (import name `pink`) and a `qpsolvers` backend
(`proxqp`). Pink is local/differential rather than global IK, so it can converge
to local minima; collision checks remain enforced by the planning world before a
candidate is accepted.

### World Backends

| Backend | Description |
|---------|-------------|
| `DrakeWorld` | Drake physics with Meshcat visualization |

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `xarm6_planner_only` | XArm 6-DOF standalone (no coordinator) |
| `xarm7-planner-coordinator` | XArm 7-DOF with coordinator |
| `dual-xarm6-planner` | Dual XArm 6-DOF |
| `xarm-perception-sim` | XArm 7-DOF simulation perception stack |

### Pink IK Manual QA

Run the existing simulation stack with Pink selected by CLI override in one
terminal:

```bash
uv sync --extra all --extra pink
uv run dimos --simulation run xarm-perception-sim \
  -o pickandplacemodule.kinematics_name=pink
```

For blueprints that instantiate `ManipulationModule` directly, use the matching
module prefix instead, for example
`-o manipulationmodule.kinematics_name=pink`.

In another terminal, open the manipulation client:

```bash
uv run python -i -m dimos.manipulation.planning.examples.manipulation_client
```

Then run:

```python
robots()
joints()
ee()
ik_pose(0.45, 0.0, 0.25)  # IK only, no path planning
ik_pose(0.45, 0.0, 0.25, seed_joints=[0.0] * 7)  # optional local-IK seed
plan_pose(0.45, 0.0, 0.25)
preview()
```

`ik_pose(...)` is a client convenience wrapper: it builds a `Pose` from xyz/rpy
arguments and calls the `solve_ik(Pose, ...)` RPC. The optional `seed_joints`
argument initializes local IK backends such as Pink from a specific joint
configuration; omit it to use the robot's current joint state.

For a baseline comparison, stop the Pink stack, start the Jacobian-backed
simulation stack, and repeat the same client calls:

```bash
uv run dimos --simulation run xarm-perception-sim
```

This QA path is simulation-only. Do not run `execute()` on physical hardware as
part of the Pink backend smoke test.

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
