# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Manipulation Planning Module

Motion planning stack for robotic manipulators using Protocol-based architecture.

## Architecture

- WorldSpec: Core backend owning physics/collision (DrakeWorld, future: MuJoCoWorld)
- KinematicsSpec: IK solvers
  - JacobianIK: Backend-agnostic iterative/differential IK
  - DrakeOptimizationIK: Drake-specific nonlinear optimization IK
- PlannerSpec: Backend-agnostic joint-space path planning
  - RRTConnectPlanner: Bi-directional RRT-Connect
  - RRTStarPlanner: RRT* (asymptotically optimal)

## Factory Functions

Use factory functions to create components:

```python
from dimos.manipulation.planning.factory import (
    create_world,
    create_kinematics,
    create_planner,
)

world = create_world(backend="drake", enable_viz=True)
kinematics = create_kinematics(name="jacobian")  # or "drake_optimization"
planner = create_planner(name="rrt_connect")  # backend-agnostic
```

## Monitors

Use WorldMonitor for reactive state synchronization:

```python
from dimos.manipulation.planning.monitor import WorldMonitor

monitor = WorldMonitor(enable_viz=True)
robot_id = monitor.add_robot(config)
monitor.finalize()
monitor.start_state_monitor(robot_id)
```
"""

# Factory functions
from dimos.manipulation.planning.factory import (
    create_kinematics,
    create_planner,
    create_planning_stack,
    create_world,
)

# Data classes and Protocols
from dimos.manipulation.planning.spec import (
    CollisionObjectMessage,
    IKResult,
    IKStatus,
    JointPath,
    KinematicsSpec,
    Obstacle,
    ObstacleType,
    PlannerSpec,
    PlanningResult,
    PlanningStatus,
    RobotModelConfig,
    RobotName,
    WorldRobotID,
    WorldSpec,
)

# Trajectory generation
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)

__all__ = [
    "CollisionObjectMessage",
    "IKResult",
    "IKStatus",
    "JointPath",
    "JointTrajectoryGenerator",
    "KinematicsSpec",
    "Obstacle",
    "ObstacleType",
    "PlannerSpec",
    "PlanningResult",
    "PlanningStatus",
    "RobotModelConfig",
    "RobotName",
    "WorldRobotID",
    "WorldSpec",
    "create_kinematics",
    "create_planner",
    "create_planning_stack",
    "create_world",
]
