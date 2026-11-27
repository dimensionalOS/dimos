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

"""Shutdown must finish even after the coordinator's RPC service has stopped."""

from collections.abc import Iterator
import time
from typing import cast

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.core.rpc_client import RPCClient
from dimos.manipulation.manipulation_spec import ExecutionStatus, OperationStatus
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


@pytest.fixture
def remote_coordinator() -> Iterator[tuple[ControlCoordinator, RPCClient]]:
    coordinator = ControlCoordinator()
    client = RPCClient.remote(ControlCoordinator)
    try:
        assert client.get_tick_count() == 0
        yield coordinator, client
    finally:
        client.stop_rpc_client()
        coordinator.stop()


@pytest.mark.parametrize(
    "state", [OperationStatus.IDLE, OperationStatus.PLANNING, OperationStatus.HAS_PLAN]
)
def test_idle_stop_needs_no_coordinator(module_factory, mocker, state):
    module = module_factory()
    module._state = state
    invoke = mocker.patch.object(
        module._control_coordinator, "task_invoke", side_effect=TimeoutError("coordinator stopped")
    )

    module.stop()

    assert module._state is OperationStatus.IDLE
    invoke.assert_not_called()


@pytest.mark.timeout(5)
@pytest.mark.parametrize("active", [False, True])
def test_stop_after_coordinator_rpc_shutdown(remote_coordinator, module_factory, mocker, active):
    coordinator, client = remote_coordinator
    module = module_factory(cast("ControlCoordinator", client))
    monitor = mocker.Mock(spec=WorldMonitor)
    module._world_monitor = monitor
    # Also covers motion whose earlier cancellation was not confirmed.
    module._state = OperationStatus.EXECUTING if active else OperationStatus.IDLE
    coordinator.stop()

    started = time.monotonic()
    module.stop()

    assert time.monotonic() - started < 2.0
    monitor.stop_all_monitors.assert_called_once_with()
    assert module.rpc is None
    assert module._state is (OperationStatus.EXECUTING if active else OperationStatus.IDLE)
    assert module._execution_manager.status is (
        ExecutionStatus.UNCERTAIN if active else ExecutionStatus.IDLE
    )


@pytest.mark.timeout(5)
def test_stop_cancels_active_remote_trajectory(remote_coordinator, module_factory, mocker):
    coordinator, client = remote_coordinator
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=("arm/j0",)))
    coordinator.add_task(task, task_type="trajectory")
    mocker.patch.object(coordinator, "get_joint_positions", return_value={"arm/j0": 0.0})
    module = module_factory(cast("ControlCoordinator", client))
    module._pending_plan = GeneratedPlan(
        group_ids=("manipulator",),
        status=PlanningStatus.SUCCESS,
        trajectory=JointTrajectory(
            joint_names=["arm/j0"],
            points=[
                TrajectoryPoint(time_from_start=0.0, positions=[0.0], velocities=[0.0]),
                TrajectoryPoint(time_from_start=1.0, positions=[0.1], velocities=[0.0]),
            ],
        ),
    )
    module._state = OperationStatus.HAS_PLAN
    assert module.execute(blocking=False).status is ExecutionStatus.ACCEPTED

    module.stop()

    assert task.get_status(time.perf_counter()).state is TrajectoryState.ABORTED
    assert module._state is OperationStatus.IDLE
    assert module._execution_manager.status is ExecutionStatus.ABORTED
