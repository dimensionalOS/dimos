# Manipulation Planning Stack

A Protocol-based motion planning system for robotic manipulators using Drake as the physics backend.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        User Code                                │
│   Uses Protocol types: WorldSpec, KinematicsSpec, PlannerSpec   │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────────┐
│                     Factory Functions                           │
│   create_world(), create_kinematics(), create_planner()         │
│   Only place that knows about concrete implementations          │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────────┐
│                  Concrete Implementations                       │
│   DrakeWorld, DrakeKinematics, DrakePlanner, DrakeRRTStar       │
└─────────────────────────────────────────────────────────────────┘
```

## Core Components

### WorldSpec (Protocol)
The core backend owning physics/collision. Responsibilities:
- Robot and obstacle management
- Collision checking
- Forward kinematics
- Context management for thread safety

### KinematicsSpec (Protocol)
Stateless IK operations. Methods:
- `solve()`: Full optimization-based IK with collision checking
- `solve_iterative()`: Jacobian-based iterative IK
- `solve_differential()`: Single Jacobian step for velocity control

### PlannerSpec (Protocol)
Joint-space path planning. Methods:
- `plan_joint_path()`: Plan collision-free path from start to goal

## Quickstart

### Basic Usage

```python
import numpy as np
from dimos.manipulation.planning import (
    create_world,
    create_kinematics,
    create_planner,
    RobotModelConfig,
    Obstacle,
    ObstacleType,
)

# 1. Create components
world = create_world(backend="drake", enable_viz=True)
kinematics = create_kinematics(backend="drake")
planner = create_planner(name="rrt_connect")

# 2. Configure robot
config = RobotModelConfig(
    name="my_robot",
    urdf_path="/path/to/robot.urdf",
    base_pose=np.eye(4),
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    end_effector_link="ee_link",
)

# 3. Add robot and finalize
robot_id = world.add_robot(config)
world.finalize()

# 4. Add obstacles (can be done after finalize)
obstacle = Obstacle(
    name="box1",
    obstacle_type=ObstacleType.BOX,
    pose=np.eye(4),
    dimensions=(0.1, 0.1, 0.1),
)
world.add_obstacle(obstacle)

# 5. Plan motion
q_start = np.zeros(6)
q_goal = np.array([0.5, 0.3, -0.2, 0.1, 0.4, 0.0])

result = planner.plan_joint_path(world, robot_id, q_start, q_goal, timeout=10.0)
if result.is_success():
    print(f"Found path with {len(result.path)} waypoints")
```

### Using Contexts for Collision Checking

```python
# Use scratch_context for thread-safe operations
with world.scratch_context() as ctx:
    world.set_positions(ctx, robot_id, q_test)

    if world.is_collision_free(ctx, robot_id):
        ee_pose = world.get_ee_pose(ctx, robot_id)
        print(f"Valid config, EE at: {ee_pose[:3, 3]}")
```

### IK Solving

```python
target_pose = np.eye(4)
target_pose[:3, 3] = [0.3, 0.2, 0.4]  # Position
# Set rotation if needed

result = kinematics.solve(
    world=world,
    robot_id=robot_id,
    target_pose=target_pose,
    seed=current_joints,  # Optional initial guess
    check_collision=True,
)

if result.is_success():
    goal_joints = result.joint_positions
```

## Directory Structure

```
planning/
├── spec.py              # Protocol definitions and data classes
├── factory.py           # Factory functions (only file with concrete imports)
├── world/
│   └── drake_world.py   # DrakeWorld implementation
├── kinematics/
│   └── drake_kinematics.py
├── planners/
│   └── drake_planner.py # RRT-Connect and RRT* implementations
├── utils/
│   └── path_utils.py    # Path interpolation utilities
└── examples/
    ├── planning_tester.py  # Interactive CLI for testing
    └── example_worldspec_integration.py
```

## Running the Interactive Tester

```bash
# Full planning tester with robot, obstacles, IK, and motion planning
python -m dimos.manipulation.planning.examples.planning_tester
```

Commands:
- `joints` - Show current joint positions
- `home` - Move to home position (all zeros)
- `random` - Move to random valid configuration
- `ee` - Show end-effector pose
- `collision` - Check current collision status
- `ik <x> <y> <z>` - Solve IK for position
- `plan <j1> <j2> ... <j6>` - Plan to joint configuration
- `add` - Add obstacle interactively
- `list` - List all obstacles
- `clear` - Remove all obstacles
- `quit` - Exit

## Key Design Decisions

1. **Protocol-based architecture**: All code uses Protocol types, enabling future backend swaps (MuJoCo, VAMP, etc.)

2. **Factory pattern**: Only factory functions know about concrete implementations

3. **Context management**: Thread-safe collision checking via scratch contexts

4. **Dynamic obstacles**: Obstacles can be added/removed after `finalize()` and are immediately visible to collision queries

5. **Stateless IK/Planning**: Kinematics and planners are stateless, using WorldSpec for all state operations

## Available Planners

| Planner | Description |
|---------|-------------|
| `rrt_connect` | Bidirectional RRT with greedy connect (fast) |
| `rrt_star` | RRT* with rewiring (asymptotically optimal) |

## Obstacle Types

| Type | Dimensions |
|------|------------|
| `BOX` | (width, height, depth) |
| `SPHERE` | (radius,) |
| `CYLINDER` | (radius, height) |
| `MESH` | mesh_path parameter |
