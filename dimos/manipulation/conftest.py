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

"""Shared manipulation test fixtures."""

from collections.abc import Iterator
from typing import Any, Protocol, cast
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus


class ModuleFactory(Protocol):
    """Callable type returned by the module factory fixture."""

    def __call__(self, coordinator: ControlCoordinator | None = None) -> ManipulationModule: ...


def _mock_control_coordinator() -> MagicMock:
    """Create a coordinator reference with safe default execution results."""
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    coordinator.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.ALREADY_STOPPED
    )
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.IDLE)
    return coordinator


@pytest.fixture
def module_factory() -> Iterator[ModuleFactory]:
    """Create started modules and stop every instance during fixture teardown."""
    modules: list[ManipulationModule] = []

    def create(coordinator: ControlCoordinator | None = None) -> ManipulationModule:
        module = ManipulationModule()
        modules.append(module)
        module._control_coordinator = (
            coordinator if coordinator is not None else _mock_control_coordinator()
        )
        cast("Any", module).coordinator_joint_state = None
        module.start()
        return module

    yield create

    for module in reversed(modules):
        module.stop()
