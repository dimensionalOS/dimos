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

from collections.abc import Iterator
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator_client import ControlCoordinatorClient
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


@pytest.fixture
def rpc_client() -> MagicMock:
    return MagicMock()


@pytest.fixture
def coordinator_client(rpc_client: MagicMock) -> Iterator[ControlCoordinatorClient]:
    client = ControlCoordinatorClient(rpc_client)
    yield client
    client.close()


def test_trajectory_commands_preserve_semantic_results(
    coordinator_client: ControlCoordinatorClient,
    rpc_client: MagicMock,
) -> None:
    trajectory = JointTrajectory()
    execute_result = TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)
    cancel_result = TrajectoryCancellationResult(TrajectoryCancellationStatus.ALREADY_STOPPED)
    rpc_client.execute_trajectory.return_value = execute_result
    rpc_client.cancel_trajectory.return_value = cancel_result

    actual_execute = coordinator_client.execute_trajectory(trajectory)
    actual_cancel = coordinator_client.cancel_trajectory()

    assert actual_execute is execute_result
    assert actual_cancel is cancel_result
    rpc_client.execute_trajectory.assert_called_once_with(trajectory)
    rpc_client.cancel_trajectory.assert_called_once_with()


def test_gripper_commands_delegate_with_typed_results(
    coordinator_client: ControlCoordinatorClient,
    rpc_client: MagicMock,
) -> None:
    rpc_client.set_gripper_position.return_value = True
    rpc_client.get_gripper_position.return_value = 0.25

    set_result = coordinator_client.set_gripper_position("gripper", 0.25)
    position = coordinator_client.get_gripper_position("gripper")

    assert set_result is True
    assert position == 0.25
    rpc_client.set_gripper_position.assert_called_once_with("gripper", 0.25)
    rpc_client.get_gripper_position.assert_called_once_with("gripper")


def test_close_stops_transferred_rpc_client_once(rpc_client: MagicMock) -> None:
    client = ControlCoordinatorClient(rpc_client)

    client.close()
    client.close()

    rpc_client.stop_rpc_client.assert_called_once_with()
