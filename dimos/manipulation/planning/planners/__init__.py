# Copyright 2025 Dimensional Inc.
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
Motion Planners Module

Contains motion planning implementations that use WorldSpec.

## Implementations

- DrakePlanner: RRT-Connect planner
- DrakeRRTStarPlanner: RRT* planner (asymptotically optimal)

## Usage

Use factory functions to create planners:

```python
from dimos.manipulation.planning.factory import create_planner

planner = create_planner(name="rrt_connect")  # Returns PlannerSpec
result = planner.plan_joint_path(world, robot_id, q_start, q_goal)
```
"""

from dimos.manipulation.planning.planners.drake_planner import (
    DrakePlanner,
    DrakeRRTStarPlanner,
)

__all__ = ["DrakePlanner", "DrakeRRTStarPlanner"]
