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

"""Shared lightweight test harnesses for manipulation module tests."""

import threading
from unittest.mock import MagicMock

from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationState,
)


class ManipulationModuleHarness(ManipulationModule):
    """Manipulation module initialized only with state needed by unit tests."""

    def __init__(self) -> None:
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""
        self._planning_epoch = 0
        self._robots = {}
        self._last_plan = None
        self._world_monitor = None
        self._planner = None
        self._kinematics = None
        self._coordinator_client = None
        self.config = MagicMock(planning_timeout=10.0)


def make_module() -> ManipulationModule:
    """Create a lightweight ManipulationModule harness for behavior tests."""
    return ManipulationModuleHarness()
