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
Kinematics Module

Contains IK solver implementations that use WorldSpec.

## Implementations

- JacobianIK: Backend-agnostic iterative/differential IK (works with any WorldSpec)
- DrakeOptimizationIK: Drake-specific nonlinear optimization IK (requires DrakeWorld)

## Usage

Use factory functions to create IK solvers:

```python
from dimos.manipulation.planning.factory import create_kinematics

# Backend-agnostic (works with any WorldSpec)
kinematics = create_kinematics(name="jacobian")

# Drake-specific (requires DrakeWorld, more accurate)
kinematics = create_kinematics(name="drake_optimization")

result = kinematics.solve(world, robot_id, target_pose)
```
"""

from dimos.manipulation.planning.kinematics.drake_optimization_ik import (
    DrakeOptimizationIK,
)
from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK

__all__ = ["DrakeOptimizationIK", "JacobianIK"]
