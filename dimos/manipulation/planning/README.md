# Manipulation Planning Stack

Motion planning for robotic manipulators. Backend-agnostic design with Drake implementation.

## Quick Start

```bash
# Terminal 1: Mock coordinator
dimos run coordinator-mock

# Terminal 2: Manipulation planner
dimos run xarm7-planner-coordinator

# Terminal 3: IPython client
python -m dimos.manipulation.planning.examples.manipulation_client
```

In IPython:
```python
joints()                # Get current joints
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
    urdf_path=Path("/path/to/xarm7.urdf"),
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
    kinematics_name="drake_optimization", # Or "jacobian"
)
module.start()
module.plan_to_joints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
module.execute()  # Sends to coordinator
```

## RobotModelConfig Fields

| Field | Description |
|-------|-------------|
| `name` | Robot identifier |
| `urdf_path` | Path to URDF/XACRO file |
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

## Directory Structure

```
planning/
├── spec.py                  # Protocols (WorldSpec, KinematicsSpec, PlannerSpec)
├── factory.py               # create_world, create_kinematics, create_planner
├── world/
│   └── drake_world.py       # DrakeWorld implementation
├── kinematics/
│   ├── jacobian_ik.py       # Backend-agnostic Jacobian IK
│   └── drake_optimization_ik.py  # Drake nonlinear IK
├── planners/
│   └── rrt_planner.py       # RRTConnectPlanner
├── monitor/                 # WorldMonitor (live state sync)
├── trajectory_generator/    # Time-parameterized trajectories
└── examples/
    ├── planning_tester.py        # Standalone CLI tester
    └── manipulation_client.py    # IPython RPC client
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
