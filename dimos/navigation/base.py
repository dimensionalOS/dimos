#!/usr/bin/env python3
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

"""State-machine-based navigation interface for autonomous mobile robots.

This module defines the contract that all navigation backends must implement. The
interface centers on three design principles:

1. **State machine control**: Navigation has explicit states (IDLE, FOLLOWING_PATH,
   RECOVERY) that callers poll to determine progress.

2. **Non-blocking goal submission**: `set_goal()` returns immediately after accepting
   the goal, allowing agent skills to interleave navigation with perception or
   communication during long traversals.

3. **Success indicator**: When returning to IDLE, `is_goal_reached()` reports whether
   the goal was actually reached (vs. cancelled/failed), so you can check the outcome
   without polling state during the transition or maintaining external tracking.

Typical usage pattern:

```python
nav.set_goal(target_pose)
while nav.get_state() == NavigationState.FOLLOWING_PATH:
    # Optionally: check sensors, update beliefs, handle interruptions
    time.sleep(0.25)
return nav.is_goal_reached()  # True = success, False = cancelled/failed
```
"""

from abc import ABC, abstractmethod
from enum import Enum

from dimos.msgs.geometry_msgs import PoseStamped


class NavigationState(Enum):
    """State machine states for navigation control.

    Used by skills and agents to monitor navigation progress and distinguish
    between idle, active navigation, and recovery behaviors.

    Attributes:
        IDLE: No active navigation goal. The navigator is ready to accept
            a new goal via `set_goal()`.
        FOLLOWING_PATH: Actively navigating toward a goal. Path planning
            and motion control are engaged.
        RECOVERY: Reserved for stuck detection and recovery behaviors.
            Currently only partially implemented - most implementations
            transition directly from FOLLOWING_PATH to IDLE on failure.
    """

    IDLE = "idle"
    FOLLOWING_PATH = "following_path"
    RECOVERY = "recovery"


class NavigationInterface(ABC):
    """Abstract interface for state-machine-based robot navigation.

    Defines a uniform API for autonomous navigation that works across different
    backends (ROS Nav2, custom planners, behavior trees). The interface uses
    non-blocking goal submission with polling-based monitoring, allowing callers
    to interleave navigation with other tasks.

    See also:
        `NavigationState`: Enum defining the state machine states.
    """

    @abstractmethod
    def set_goal(self, goal: PoseStamped) -> bool:
        """Submit a navigation goal (non-blocking).

        Initiates navigation toward the target pose and returns immediately. If a
        previous goal is active, it is implicitly cancelled. The navigator transitions
        to the `FOLLOWING_PATH` state upon acceptance.

        Args:
            goal: Target pose to navigate to.

        Returns:
            True if goal was accepted, False otherwise. Acceptance does not
            guarantee reachability.

        Note:
            Use `get_state()` and `is_goal_reached()` to poll navigation progress.
            The goal's frame_id determines the coordinate frame (e.g., "map", "odom").
        """
        pass

    @abstractmethod
    def get_state(self) -> NavigationState:
        """
        Get the current state of the navigator.

        Returns:
            Current navigation state
        """
        pass

    @abstractmethod
    def is_goal_reached(self) -> bool:
        """
        Check if the current goal has been reached.

        Returns:
            True if goal was reached, False otherwise
        """
        pass

    @abstractmethod
    def cancel_goal(self) -> bool:
        """
        Cancel the current navigation goal.

        Returns:
            True if goal was cancelled, False if no goal was active
        """
        pass


__all__ = ["NavigationInterface", "NavigationState"]
