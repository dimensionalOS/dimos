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

"""Large E2E tests for manipulation planning groups with a coordinator.

These tests launch a real ManipulationModule + ControlCoordinator blueprint and
exercise the public planning RPCs over LCM, matching the self-hosted large-test
style used by the navigation stack.
"""

from __future__ import annotations

from collections.abc import Callable
import time

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.rpc_client import RPCClient
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import (
    ExecutionStatus,
    ManipulationSnapshot,
    PlanningGroupInfo,
)
from dimos.msgs.sensor_msgs.JointState import JointState

pytestmark = [pytest.mark.self_hosted_large]

JOINT_STATE_TOPIC = "/coordinator_joint_state#sensor_msgs.JointState"
BLUEPRINT = "openarm-planner-coordinator"
LEFT_GROUP_ID = "left_arm"
RIGHT_GROUP_ID = "right_arm"
BOTH_GROUP_ID = "both_arms"
ALL_GROUP_IDS = {LEFT_GROUP_ID, RIGHT_GROUP_ID, BOTH_GROUP_ID}


def _wait_for_groups(
    client: RPCClient,
    expected_ids: set[str],
    *,
    timeout: float = 120.0,
) -> dict[str, PlanningGroupInfo]:
    deadline = time.time() + timeout
    last_error: BaseException | None = None
    groups_by_id: dict[str, PlanningGroupInfo] = {}
    while time.time() < deadline:
        try:
            groups_by_id = {group.id: group for group in client.list_planning_groups()}
            if groups_by_id.keys() == expected_ids:
                return groups_by_id
        except Exception as exc:
            last_error = exc
        time.sleep(0.5)
    raise TimeoutError(
        f"Timed out waiting for planning groups {sorted(expected_ids)}; "
        f"last groups={sorted(groups_by_id)}"
    ) from last_error


def _wait_for_trajectory_completion(
    coordinator_client: RPCClient,
    *,
    timeout: float = 10.0,
) -> None:
    deadline = time.time() + timeout
    last_active: list[str] = []
    while time.time() < deadline:
        last_active = coordinator_client.get_active_tasks()
        if not last_active:
            return
        time.sleep(0.1)
    raise TimeoutError(f"Trajectory did not complete; active={last_active}")


def _wait_for_manipulation_ready(
    client: RPCClient,
    *,
    timeout: float = 10.0,
) -> None:
    deadline = time.time() + timeout
    last_state: str | None = None
    while time.time() < deadline:
        snapshot = client.get_state()
        last_state = snapshot.operation_status.name
        if last_state in {"IDLE", "COMPLETED"}:
            return
        time.sleep(0.1)
    raise TimeoutError(f"ManipulationModule did not become ready; last={last_state}")


def _wait_for_current_joints(
    client: RPCClient,
    group_ids: tuple[str, ...],
    *,
    timeout: float = 10.0,
) -> None:
    deadline = time.time() + timeout
    missing = group_ids
    while time.time() < deadline:
        try:
            snapshot = client.get_state()
            missing = tuple(
                group_id for group_id in group_ids if snapshot.groups[group_id].joints is None
            )
        except Exception:
            # Robot metadata becomes visible while the planning world is still
            # finalizing. Treat that readiness race like a missing joint state.
            missing = group_ids
        if not missing:
            return
        time.sleep(0.1)
    raise TimeoutError(f"Timed out waiting for current joints from {missing}")


def _prepare_for_planning(client: RPCClient, group_ids: tuple[str, ...]) -> None:
    _wait_for_current_joints(client, group_ids)
    # Robot info and joint-state topics can become available just before the
    # manipulation module finishes finalizing world monitors. Require a stable
    # ready state after joint state is flowing to avoid command-readiness flakes.
    time.sleep(0.25)
    _wait_for_manipulation_ready(client)


def _offset_target(snapshot: ManipulationSnapshot, group_id: str, delta: float) -> JointState:
    current = snapshot.groups[group_id].joints
    assert current is not None
    return JointState(
        name=current.name,
        position=[position + delta for position in current.position],
    )


def _start_openarm_mock_planner(
    start_blueprint: Callable[..., DimosCliCall], lcm_spy: LcmSpy
) -> None:
    lcm_spy.save_topic(JOINT_STATE_TOPIC)
    start_blueprint(BLUEPRINT)
    lcm_spy.wait_for_saved_topic(JOINT_STATE_TOPIC, timeout=120.0)


def test_single_arm_plans_and_executes_through_control_coordinator(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
) -> None:
    """Plan with one arm and execute through its trajectory task."""
    _start_openarm_mock_planner(start_blueprint, lcm_spy)

    client = RPCClient(None, ManipulationModule)
    coordinator_client = RPCClient(None, ControlCoordinator)
    try:
        groups = _wait_for_groups(client, ALL_GROUP_IDS)
        left_id = groups[LEFT_GROUP_ID].id

        tasks = coordinator_client.list_tasks()
        assert tasks == [JOINT_TRAJECTORY_TASK_NAME]

        _prepare_for_planning(client, (left_id,))

        planned = client.plan_to_joints(
            {left_id: _offset_target(client.get_state(), left_id, 0.02)}
        )
        assert planned.succeeded, planned
        result = client.execute(blocking=True)
        assert result.status is ExecutionStatus.COMPLETED

        _wait_for_trajectory_completion(coordinator_client)
    finally:
        coordinator_client.stop_rpc_client()
        client.stop_rpc_client()


def test_dual_arm_plans_and_dispatches_both_arms_through_control_coordinator(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
) -> None:
    """Plan one generated plan over both arms and dispatch through one trajectory task."""
    _start_openarm_mock_planner(start_blueprint, lcm_spy)

    client = RPCClient(None, ManipulationModule)
    coordinator_client = RPCClient(None, ControlCoordinator)
    try:
        groups = _wait_for_groups(client, ALL_GROUP_IDS)
        left_id = groups[LEFT_GROUP_ID].id
        right_id = groups[RIGHT_GROUP_ID].id

        tasks = coordinator_client.list_tasks()
        assert tasks == [JOINT_TRAJECTORY_TASK_NAME]

        _prepare_for_planning(client, (left_id, right_id))

        snapshot = client.get_state()
        planned = client.plan_to_joints(
            {
                left_id: _offset_target(snapshot, left_id, 0.02),
                right_id: _offset_target(snapshot, right_id, 0.02),
            }
        )
        assert planned.succeeded, planned
        result = client.execute(blocking=True)
        assert result.status is ExecutionStatus.COMPLETED

        _wait_for_trajectory_completion(coordinator_client)
    finally:
        coordinator_client.stop_rpc_client()
        client.stop_rpc_client()
