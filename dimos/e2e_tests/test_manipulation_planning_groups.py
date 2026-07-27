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
from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.rpc_client import RPCClient
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState

pytestmark = [pytest.mark.self_hosted_large]

JOINT_STATE_TOPIC = "/coordinator_joint_state#sensor_msgs.JointState"
BLUEPRINT = "openarm-mock-planner-coordinator"


def _wait_for_robot_info(
    client: RPCClient,
    robot_name: str,
    *,
    timeout: float = 120.0,
) -> dict[str, Any]:
    deadline = time.time() + timeout
    last_error: BaseException | None = None
    while time.time() < deadline:
        try:
            info = client.get_robot_info(robot_name)
            if info and info.get("planning_groups"):
                return info
        except BaseException as exc:
            last_error = exc
        time.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for {robot_name!r} robot info") from last_error


def _wait_for_trajectory_completion(
    client: RPCClient,
    robot_name: str,
    *,
    timeout: float = 10.0,
) -> None:
    deadline = time.time() + timeout
    last_status: dict[str, Any] | None = None
    while time.time() < deadline:
        last_status = client.get_trajectory_status(robot_name)
        if last_status is not None and last_status.get("state") == TrajectoryState.COMPLETED:
            return
        time.sleep(0.1)
    raise TimeoutError(f"{robot_name!r} trajectory did not complete; last={last_status}")


def _wait_for_manipulation_state(
    client: RPCClient,
    state_name: str,
    *,
    timeout: float = 10.0,
) -> None:
    deadline = time.time() + timeout
    last_state: str | None = None
    while time.time() < deadline:
        last_state = client.get_state()
        if last_state == state_name:
            return
        time.sleep(0.1)
    raise TimeoutError(f"ManipulationModule did not reach {state_name}; last={last_state}")


def _wait_for_current_joints(
    client: RPCClient,
    robot_names: tuple[str, ...],
    *,
    timeout: float = 10.0,
) -> None:
    deadline = time.time() + timeout
    missing = robot_names
    while time.time() < deadline:
        missing = tuple(
            robot_name
            for robot_name in robot_names
            if client.get_current_joints(robot_name) is None
        )
        if not missing:
            return
        time.sleep(0.1)
    raise TimeoutError(f"Timed out waiting for current joints from {missing}")


def _prepare_for_planning(client: RPCClient, robot_names: tuple[str, ...]) -> None:
    client.reset()
    _wait_for_manipulation_state(client, "IDLE")
    _wait_for_current_joints(client, robot_names)
    # Robot info and joint-state topics can become available just before the
    # manipulation module finishes finalizing world monitors. Require a stable
    # ready state after joint state is flowing to avoid command-readiness flakes.
    time.sleep(0.25)
    _wait_for_manipulation_state(client, "IDLE")


def _planning_group_id(info: dict[str, Any]) -> str:
    groups = info["planning_groups"]
    assert len(groups) == 1
    group = groups[0]
    if isinstance(group, PlanningGroup):
        return group.id
    group_id = group["id"]
    assert isinstance(group_id, str)
    return group_id


def _offset_target(client: RPCClient, robot_name: str, delta: float) -> JointState:
    current = client.get_current_joints(robot_name)
    assert current is not None
    return JointState(position=[position + delta for position in current])


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
        left_info = _wait_for_robot_info(client, "left_arm")
        left_id = _planning_group_id(left_info)

        tasks = coordinator_client.list_tasks()
        assert left_info["coordinator_task_name"] in tasks

        _prepare_for_planning(client, ("left_arm",))

        planned = client.plan_to_joint_targets({left_id: _offset_target(client, "left_arm", 0.02)})
        assert planned, client.get_error()
        assert client.has_planned_path()
        assert client.execute_plan()

        _wait_for_trajectory_completion(client, "left_arm")
    finally:
        coordinator_client.stop_rpc_client()
        client.stop_rpc_client()


def test_dual_arm_plans_and_dispatches_both_arms_through_control_coordinator(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
) -> None:
    """Plan one generated plan over both arms and dispatch both JTC tasks."""
    _start_openarm_mock_planner(start_blueprint, lcm_spy)

    client = RPCClient(None, ManipulationModule)
    coordinator_client = RPCClient(None, ControlCoordinator)
    try:
        left_info = _wait_for_robot_info(client, "left_arm")
        right_info = _wait_for_robot_info(client, "right_arm")
        left_id = _planning_group_id(left_info)
        right_id = _planning_group_id(right_info)

        tasks = coordinator_client.list_tasks()
        assert left_info["coordinator_task_name"] in tasks
        assert right_info["coordinator_task_name"] in tasks

        _prepare_for_planning(client, ("left_arm", "right_arm"))

        planned = client.plan_to_joint_targets(
            {
                left_id: _offset_target(client, "left_arm", 0.02),
                right_id: _offset_target(client, "right_arm", -0.02),
            }
        )
        assert planned, client.get_error()
        assert client.has_planned_path()
        assert client.execute_plan()

        _wait_for_trajectory_completion(client, "left_arm")
        _wait_for_trajectory_completion(client, "right_arm")
    finally:
        coordinator_client.stop_rpc_client()
        client.stop_rpc_client()
