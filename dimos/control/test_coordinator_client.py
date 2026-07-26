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

from unittest.mock import MagicMock, call

import pytest

from dimos.control.coordinator_client import ControlCoordinatorClient
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


@pytest.fixture
def rpc_client() -> MagicMock:
    return MagicMock()


@pytest.fixture
def coordinator_client(rpc_client: MagicMock) -> ControlCoordinatorClient:
    client = ControlCoordinatorClient(rpc_client)
    yield client
    client.close()


def test_task_commands_preserve_raw_results(
    coordinator_client: ControlCoordinatorClient,
    rpc_client: MagicMock,
) -> None:
    trajectory = JointTrajectory()
    rpc_client.task_invoke.side_effect = [True, None]

    execute_result = coordinator_client.execute_task("trajectory", trajectory)
    cancel_result = coordinator_client.cancel_task("trajectory")

    assert execute_result is True
    assert cancel_result is None
    assert rpc_client.task_invoke.call_args_list == [
        call("trajectory", "execute", {"trajectory": trajectory}),
        call("trajectory", "cancel", {}),
    ]


def test_status_and_gripper_commands_delegate_with_typed_results(
    coordinator_client: ControlCoordinatorClient,
    rpc_client: MagicMock,
) -> None:
    rpc_client.task_invoke.return_value = 2
    rpc_client.set_gripper_position.return_value = True
    rpc_client.get_gripper_position.return_value = 0.25

    state = coordinator_client.get_task_state("trajectory")
    set_result = coordinator_client.set_gripper_position("gripper", 0.25)
    position = coordinator_client.get_gripper_position("gripper")

    assert state == 2
    assert set_result is True
    assert position == 0.25
    rpc_client.task_invoke.assert_called_once_with("trajectory", "get_state", {})
    rpc_client.set_gripper_position.assert_called_once_with("gripper", 0.25)
    rpc_client.get_gripper_position.assert_called_once_with("gripper")


def test_close_stops_transferred_rpc_client_once(rpc_client: MagicMock) -> None:
    client = ControlCoordinatorClient(rpc_client)

    client.close()
    client.close()

    rpc_client.stop_rpc_client.assert_called_once_with()
