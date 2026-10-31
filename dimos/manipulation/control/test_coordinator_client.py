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

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.control.coordinator_client import CoordinatorClient
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


@pytest.fixture
def client(mocker):
    rpc = mocker.Mock(spec=["task_invoke", "stop_rpc_client"])
    mocker.patch("dimos.manipulation.control.coordinator_client.RPCClient", return_value=rpc)
    instance = CoordinatorClient()
    try:
        yield instance, rpc
    finally:
        instance.stop()


def test_execute_uses_generic_task_rpc_and_preserves_rejection(client):
    instance, rpc = client
    trajectory = JointTrajectory()
    rejection = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.START_STATE_MISMATCH, "latest tick no longer matches the anchor"
    )
    rpc.task_invoke.return_value = rejection

    assert instance.execute_trajectory(trajectory) is rejection
    rpc.task_invoke.assert_called_once_with(
        "joint_trajectory", "execute", {"trajectory": trajectory}
    )


def test_cancel_uses_generic_task_rpc_and_preserves_uncertainty(client):
    instance, rpc = client
    uncertainty = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.UNCERTAIN, "driver did not acknowledge"
    )
    rpc.task_invoke.return_value = uncertainty

    assert instance.cancel_trajectory() is uncertainty
    rpc.task_invoke.assert_called_once_with("joint_trajectory", "cancel", {})
