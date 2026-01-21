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
World Monitor Module

Provides reactive monitoring for keeping WorldSpec synchronized with the real world.

## Components

- WorldMonitor: Top-level monitor using WorldSpec Protocol
- WorldStateMonitor: Syncs joint state to WorldSpec
- WorldObstacleMonitor: Syncs obstacles to WorldSpec

All monitors use the factory pattern and Protocol types.

## Example

```python
from dimos.manipulation.planning.monitor import WorldMonitor

monitor = WorldMonitor(enable_viz=True)
robot_id = monitor.add_robot(config)
monitor.finalize()

# Start monitoring
monitor.start_state_monitor(robot_id)
monitor.start_obstacle_monitor()

# Handle joint state messages
monitor.on_joint_state(msg, robot_id)

# Thread-safe collision checking
is_valid = monitor.is_state_valid(robot_id, q_test)
```
"""

from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.monitor.world_obstacle_monitor import (
    WorldObstacleMonitor,
)
from dimos.manipulation.planning.monitor.world_state_monitor import WorldStateMonitor

# Re-export message types from spec for convenience
from dimos.manipulation.planning.spec import CollisionObjectMessage

__all__ = [
    "CollisionObjectMessage",
    "WorldMonitor",
    "WorldObstacleMonitor",
    "WorldStateMonitor",
]
