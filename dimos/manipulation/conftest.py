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
from pathlib import Path
from typing import Any, Protocol, cast
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.assets.model import RobotModel


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


def _test_model() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/test/model.urdf")),
        joint_names=["arm/j0"],
        base_link="base",
        planning_groups=[PlanningGroupDefinition("manipulator", ("arm/j0",), "base", "tool")],
    )


@pytest.fixture
def module_factory(mocker: MockerFixture) -> Iterator[ModuleFactory]:
    """Create started modules and stop every instance during fixture teardown."""
    modules: list[ManipulationModule] = []

    def create(coordinator: ControlCoordinator | None = None) -> ManipulationModule:
        module = ManipulationModule(model=_test_model())
        modules.append(module)
        module._control_coordinator = (
            coordinator if coordinator is not None else _mock_control_coordinator()
        )
        cast("Any", module).coordinator_joint_state = None
        # Nulling a port drops it from Module.inputs, so start() does not bind
        # handle_voxel_map and no transport is needed. Same reason as above.
        cast("Any", module).voxel_map = None
        mocker.patch.object(module, "_initialize_planning")
        module.start()
        return module

    yield create

    for module in reversed(modules):
        module.stop()
