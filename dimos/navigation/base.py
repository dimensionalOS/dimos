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

from abc import ABC, abstractmethod
from enum import Enum

from dimos.msgs.geometry_msgs import PoseStamped


class NavigationState(Enum):
    IDLE = "idle"
    FOLLOWING_PATH = "following_path"
    RECOVERY = "recovery"


class NavigationInterface(ABC):
    @abstractmethod
    def set_goal(self, goal: PoseStamped) -> bool:
        """
        Set a new navigation goal (non-blocking).

        Args:
            goal: Target pose to navigate to

        Returns:
            True if goal was accepted, False otherwise
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
