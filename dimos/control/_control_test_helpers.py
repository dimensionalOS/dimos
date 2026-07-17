# Copyright 2026 Dimensional Inc.
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

"""Shared control-task stubs for tests.

Not named ``test_*``/``*_test`` so pytest does not collect it; both
``test_control.py`` and ``test_coordinator_routing.py`` import ``RecordingTask``
from here rather than reaching into each other's test modules.
"""

from __future__ import annotations

from typing import Any

from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)


class RecordingTask(BaseControlTask):
    """Stub task that records every stream handler invocation."""

    def __init__(self, name: str, joints: frozenset[str] = frozenset()) -> None:
        self._name = name
        self._joints = frozenset(joints)
        self.cartesian_calls: list[tuple[Any, float]] = []
        self.ee_twist_calls: list[tuple[Any, float]] = []
        self.buttons_calls: list[Any] = []

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=self._joints)

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    def on_cartesian_command(self, pose: Any, t_now: float) -> bool:
        self.cartesian_calls.append((pose, t_now))
        return True

    def on_ee_twist_command(self, twist: Any, t_now: float) -> bool:
        self.ee_twist_calls.append((twist, t_now))
        return True

    def on_buttons(self, msg: Any) -> bool:
        self.buttons_calls.append(msg)
        return True

    def on_teleop_buttons(self, msg: Any, t_now: float) -> bool:
        # Mirrors TeleopIKTask: the uniform handler delegates to on_buttons.
        return self.on_buttons(msg)
