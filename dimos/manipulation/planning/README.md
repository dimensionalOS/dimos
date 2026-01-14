# Manipulation Planning Stack

Motion planning for robotic manipulators using Drake. Integrates with Control Orchestrator for execution.

## Quick Start (3 Terminals)

```bash
# Terminal 1: Start mock orchestrator
dimos run orchestrator-mock

# Terminal 2: Start manipulation planner
dimos run xarm7-planner-orchestrator

# Terminal 3: Run IPython client
python -m dimos.manipulation.planning.examples.manipulation_client
```

Then in IPython:
```python
c.joints()                # Get current joints
c.plan([0.1] * 7)         # Plan to target
c.preview()               # Preview in Meshcat (check c.url())
c.execute()               # Execute via orchestrator
```

## Architecture

```
ManipulationModule (RPC interface, state machine, multi-robot)
        │
Factory Functions (create_world, create_kinematics, create_planner)
        │
Drake Implementations (DrakeWorld, DrakeKinematics, DrakePlanner)
        │
Control Orchestrator (trajectory execution via RPC)
```

## Using ManipulationModule

```python
from dimos.manipulation import ManipulationModule
from dimos.manipulation.planning.spec import RobotModelConfig

config = RobotModelConfig(
    name="xarm7",
    urdf_path="/path/to/xarm7.urdf",
    base_pose=np.eye(4),
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
    end_effector_link="link7",
    base_link="link_base",
    joint_name_mapping={"arm_joint1": "joint1", ...},  # URDF <-> orchestrator
    orchestrator_task_name="traj_arm",
)

module = ManipulationModule(robots=[config], planning_timeout=10.0, enable_viz=True)
module.start()
module.plan_to_joints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
module.execute()  # Sends to orchestrator
```

## RobotModelConfig Fields

| Field | Description |
|-------|-------------|
| `name` | Robot identifier |
| `urdf_path` | Path to URDF/XACRO file |
| `base_pose` | 4x4 transform matrix for robot base |
| `joint_names` | List of joint names in URDF |
| `end_effector_link` | Name of EE link in URDF |
| `base_link` | Name of base link in URDF |
| `max_velocity` | Max joint velocity (rad/s) |
| `max_acceleration` | Max joint acceleration (rad/s²) |
| `joint_name_mapping` | Dict mapping orchestrator names to URDF names |
| `orchestrator_task_name` | Task name for trajectory execution RPC |
| `package_paths` | Dict of ROS package paths for mesh resolution |
| `xacro_args` | Dict of xacro arguments (e.g., `{"dof": "7"}`) |

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `xarm6-planner` | XArm 6-DOF planner (standalone) |
| `xarm7-planner-orchestrator` | XArm 7-DOF with orchestrator |
| `dual-xarm6-planner` | Dual XArm 6-DOF planner |

## Directory Structure

```
planning/
├── spec.py              # Protocol definitions
├── factory.py           # Factory functions
├── world/               # DrakeWorld
├── kinematics/          # DrakeKinematics
├── planners/            # RRT-Connect, RRT*
├── monitor/             # WorldMonitor, state sync
├── trajectory_generator/
└── examples/
    ├── planning_tester.py     # Standalone CLI
    └── manipulation_client.py # IPython RPC client
```

## Protocols

- **WorldSpec**: Physics/collision backend (DrakeWorld, MuJoCoWorld, PyBulletWorld)
- **KinematicsSpec**: IK solving (DrakeKinematics, TracIK, KDL)
- **PlannerSpec**: Path planning (DrakeRRTConnect, OMPL, cuRobo)

## Supported Robots

| Robot | DOF |
|-------|-----|
| `piper` | 6 |
| `xarm6` | 6 |
| `xarm7` | 7 |

## Planners

| Planner | Description |
|---------|-------------|
| `rrt_connect` | Bidirectional RRT (fast) |
| `rrt_star` | RRT* with rewiring (optimal) |

## Obstacle Types

| Type | Dimensions |
|------|------------|
| `BOX` | (w, h, d) |
| `SPHERE` | (radius,) |
| `CYLINDER` | (radius, height) |
| `MESH` | mesh_path |
