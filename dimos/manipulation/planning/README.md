# Manipulation Planning Stack

Motion planning for robotic manipulators. Backend-agnostic design with Drake as the default backend and RoboPlan as an optional backend.

## Quick Start

```bash
# 1. Verify manipulation dependencies load correctly (standalone, no hardware):
dimos run xarm6-planner-only

# 2. Keyboard teleop with mock arm (single command):
dimos run keyboard-teleop-xarm7

# 3. Interactive RPC client (plan, inspect, execute from Python):
dimos run xarm7-planner-coordinator                                    # terminal 1
python -i -m dimos.manipulation.planning.examples.manipulation_client  # terminal 2
```

In the interactive client:
```python skip
commands()              # List available commands
joints()                # Get current joint positions
plan([0.1] * 7)         # Plan to target
get_planned_path()      # Inspect stored path data; use Viser for visual preview
execute()               # Execute via coordinator
```

```error
Traceback (most recent call last):
  File "/tmp/tmpsotl_ecl.py", line 1, in <module>
    commands()              # List available commands
    ^^^^^^^^
NameError: name 'commands' is not defined

Exit code: 1
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

```python skip
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
    planning_backend="drake",
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

## Planning Backends

| Backend | Selection | Notes |
|---------|-----------|-------|
| `drake` | `planning_backend="drake"` or omitted | Default backend. Uses Drake physics and collision checking. |
| `roboplan` | `planning_backend="roboplan"` | Optional backend. Uses RoboPlan scene, RRT, SimpleIK, and optional RoboPlan TOPPRA retiming. |

RoboPlan is selected through the existing `ManipulationModule` configuration surface:

```python skip
module = ManipulationModule(
    robots=[config],
    planning_backend="roboplan",
    planning_backend_options={
        "roboplan": {
            "srdf_path": "/path/to/robot.srdf",
            "planning_group": "arm",
            "active_joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            "base_frame": "base_link",
            "end_effector_frame": "tool0",
            "retiming": "dimos",
            "rrt": {
                "max_nodes": 1000,
                "max_connection_distance": 3.0,
                "collision_check_step_size": 0.05,
                "collision_check_use_bisection": False,
                "goal_biasing_probability": 0.15,
                "rrt_connect": True,
            },
            "ik": {
                "max_iters": 100,
                "max_time": 0.05,
                "check_collisions": True,
            },
            "toppra": {
                "dt": 0.02,
                "mode": "Hermite",
                "velocity_scale": 1.0,
                "acceleration_scale": 1.0,
            },
        }
    },
)
```

RoboPlan requires a URDF from `RobotModelConfig.model_path`, an SRDF path in `planning_backend_options`, package paths when meshes use package URIs, a planning group or active joint names, base and end-effector frames, and complete position limits in `joint_limits_lower` and `joint_limits_upper`. Public DimOS inputs and outputs continue to use `RobotModelConfig.joint_names`; the backend maps into RoboPlan's active joint order internally.

### RoboPlan Feature Support

| Feature | Status |
|---------|--------|
| Joint planning | Supported through `roboplan.rrt.RRT` when `roboplan.core` and `roboplan.rrt` are installed. |
| Pose planning / IK | Supported as RoboPlan SimpleIK followed by joint planning when `roboplan.simple_ik` is installed. |
| Collision validation | Supported through RoboPlan scene collision checks and path collision checks. |
| Primitive obstacles | Boxes, spheres, and cylinders are projected when primitive obstacles are enabled. |
| Mesh obstacles | Reported through capability diagnostics unless mesh projection is explicitly enabled and implemented for the selected config. |
| Pointcloud layers / attached objects | Reported as unsupported unless runtime capabilities say otherwise. |
| FK / Jacobian | Exposed when the installed RoboPlan scene binding provides `forwardKinematics` and `computeFrameJacobian`. |
| Distance query | Reported as unsupported. |
| Path rendering | Owned by the Viser manipulation panel; the backend returns normalized path data. |
| Retiming | `retiming="dimos"` uses the existing DimOS trajectory generator. `retiming="toppra"` requires `roboplan.toppra`. |

### RoboPlan Dependency Troubleshooting

RoboPlan is lazy-imported. Missing RoboPlan bindings do not affect default Drake-backed stacks. If `planning_backend="roboplan"` is selected, verify the required modules with:

```bash
python -c "import roboplan.core, roboplan.rrt, roboplan.simple_ik"
```

If TOPPRA retiming is selected, also verify:

```bash
python -c "import roboplan.toppra"
```

Some distributions split RoboPlan into separate Python binding packages for core, RRT, SimpleIK, and TOPPRA. Install the bindings that match the modules your configuration uses.

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
| `DrakeWorld` | Drake physics and collision checking |
| `RoboPlan` | Optional native RoboPlan scene, RRT, SimpleIK, and optional TOPPRA timing |

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

# RoboPlan backend unit tests (mocked optional dependency behavior)
pytest dimos/manipulation/planning/backends/roboplan/test_roboplan_backend.py -v

# Integration tests (requires Drake)
pytest dimos/e2e_tests/test_manipulation_module.py -v
```
