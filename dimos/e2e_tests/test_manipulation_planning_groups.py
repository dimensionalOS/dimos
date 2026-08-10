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
from typing import Any, cast

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.rpc_client import RPCClient
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.topics import DEFAULT_TRAJECTORY_TASK_NAME

pytestmark = [pytest.mark.self_hosted_large]

JOINT_STATE_TOPIC = "/coordinator_joint_state#sensor_msgs.JointState"
BLUEPRINT = "openarm-mock-planner-coordinator"


def _wait_for_model_info(
    client: RPCClient,
    *,
    timeout: float = 120.0,
) -> dict[str, Any]:
    deadline = time.time() + timeout
    last_error: BaseException | None = None
    while time.time() < deadline:
        try:
            info = client.get_model_info()
            if info and info.get("planning_groups"):
                return cast("dict[str, Any]", info)
        except Exception as exc:
            last_error = exc
        time.sleep(0.5)
    raise TimeoutError("Timed out waiting for model info") from last_error


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
    *,
    timeout: float = 10.0,
) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if client.get_current_joints() is not None:
                return
        except Exception:
            # Model metadata becomes visible while the planning world is still
            # finalizing. Treat that readiness race like a missing joint state.
            pass
        time.sleep(0.1)
    raise TimeoutError("Timed out waiting for current model joints")


def _prepare_for_planning(client: RPCClient) -> None:
    client.reset()
    _wait_for_manipulation_state(client, "IDLE")
    _wait_for_current_joints(client)
    # Model info and joint-state topics can become available just before the
    # manipulation module finishes finalizing world monitors. Require a stable
    # ready state after joint state is flowing to avoid command-readiness flakes.
    time.sleep(0.25)
    _wait_for_manipulation_state(client, "IDLE")


def _planning_group(info: dict[str, Any], group_id: str) -> PlanningGroup:
    group = next(
        group
        for group in info["planning_groups"]
        if (group.id if isinstance(group, PlanningGroup) else group["id"]) == group_id
    )
    if isinstance(group, PlanningGroup):
        return group
    return PlanningGroup(**group)


def _offset_target(
    client: RPCClient, info: dict[str, Any], group: PlanningGroup, delta: float
) -> JointState:
    current = client.get_current_joints()
    assert current is not None
    positions = dict(zip(info["joint_names"], current, strict=True))
    return JointState(
        name=list(group.joint_names),
        position=[positions[name] + delta for name in group.joint_names],
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
        info = _wait_for_model_info(client)
        left_group = _planning_group(info, "left_arm")

        tasks = coordinator_client.list_tasks()
        assert tasks == [DEFAULT_TRAJECTORY_TASK_NAME]

        _prepare_for_planning(client)

        planned = client.plan_to_joint_targets(
            {left_group.id: _offset_target(client, info, left_group, 0.02)}
        )
        assert planned, client.get_error()
        assert client.has_planned_path()
        assert client.execute_plan()

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
        info = _wait_for_model_info(client)
        left_group = _planning_group(info, "left_arm")
        right_group = _planning_group(info, "right_arm")

        tasks = coordinator_client.list_tasks()
        assert tasks == [DEFAULT_TRAJECTORY_TASK_NAME]

        _prepare_for_planning(client)

        planned = client.plan_to_joint_targets(
            {
                left_group.id: _offset_target(client, info, left_group, 0.02),
                right_group.id: _offset_target(client, info, right_group, -0.02),
            }
        )
        assert planned, client.get_error()
        assert client.has_planned_path()
        assert client.execute_plan()

        _wait_for_trajectory_completion(coordinator_client)
    finally:
        coordinator_client.stop_rpc_client()
        client.stop_rpc_client()
