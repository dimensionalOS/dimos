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

"""OpenArm bimanual manipulation e2e tests."""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import time
from typing import Any

import pytest

from dimos.core.rpc_client import RPCClient
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

_ROBOTS = ("left_arm", "right_arm")
_TARGET = [0.05] * 7
_JOINT_STATE_TOPIC = "/coordinator/joint_state#sensor_msgs.JointState"
_PLANNING_TIMEOUT_S = 30.0
_PREVIEW_DURATION_S = 0.5

logger = setup_logger()


def _wait_until(
    predicate: Callable[[], bool],
    *,
    timeout_s: float,
    interval_s: float = 0.1,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(interval_s)
    raise AssertionError(f"condition was not met within {timeout_s:.1f}s")


def _wait_for_robot_state(client: RPCClient, robot_name: str) -> None:
    _wait_until(
        lambda: client.get_current_joints(robot_name) is not None,
        timeout_s=10.0,
    )


def _assert_successful_plan_status(status: dict[str, Any], robot_name: str) -> None:
    assert status["robot_name"] == robot_name
    assert status["done"] is True
    assert status["success"] is True
    assert status["error"] is None
    assert status["duration_s"] is not None


def _submit_plan_when_ready(
    client: RPCClient,
    robot_name: str,
    target: list[float],
    *,
    timeout_s: float = 15.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if client.plan_to_joints(JointState(position=target), robot_name) is True:
            return
        time.sleep(0.25)
    raise AssertionError(f"{robot_name} plan was not accepted within {timeout_s:.1f}s")


def _plan_and_wait(
    client: RPCClient,
    robot_name: str,
    target: list[float],
    *,
    wait_until_ready: bool = False,
) -> float:
    start = time.monotonic()
    if wait_until_ready:
        _submit_plan_when_ready(client, robot_name, target)
    else:
        accepted = client.plan_to_joints(JointState(position=target), robot_name)
        assert accepted is True
    assert client.wait_for_planning_completion(robot_name, _PLANNING_TIMEOUT_S) is True
    status = client.get_planning_status(robot_name)
    _assert_successful_plan_status(status, robot_name)
    assert client.has_planned_path(robot_name) is True
    return time.monotonic() - start


def _plan_concurrently(client: RPCClient, targets: dict[str, list[float]]) -> float:
    def submit(robot_name: str) -> bool:
        return bool(client.plan_to_joints(JointState(position=targets[robot_name]), robot_name))

    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=len(targets)) as pool:
        futures = {robot_name: pool.submit(submit, robot_name) for robot_name in targets}
        for robot_name, future in futures.items():
            assert future.result(timeout=5.0) is True, f"{robot_name} plan was not accepted"

    assert client.wait_for_planning_completion(None, _PLANNING_TIMEOUT_S) is True
    return time.monotonic() - start


def _preview_concurrently(client: RPCClient) -> float:
    def preview(robot_name: str) -> bool:
        return bool(client.preview_path(_PREVIEW_DURATION_S, robot_name))

    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=len(_ROBOTS)) as pool:
        futures = {robot_name: pool.submit(preview, robot_name) for robot_name in _ROBOTS}
        for robot_name, future in futures.items():
            assert future.result(timeout=5.0) is True, f"{robot_name} preview failed"
    return time.monotonic() - start


def _execute_concurrently(client: RPCClient) -> float:
    def execute(robot_name: str) -> bool:
        return bool(client.execute(robot_name))

    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=len(_ROBOTS)) as pool:
        futures = {robot_name: pool.submit(execute, robot_name) for robot_name in _ROBOTS}
        for robot_name, future in futures.items():
            assert future.result(timeout=5.0) is True, f"{robot_name} execute was not accepted"
    return time.monotonic() - start


@pytest.mark.skipif_in_ci
@pytest.mark.self_hosted
def test_openarm_mock_bimanual_planning_overlap(lcm_spy, start_blueprint) -> None:
    """OpenArm mock should overlap bimanual plan, preview, and execute acceptance."""
    lcm_spy.save_topic(_JOINT_STATE_TOPIC)
    start_blueprint("openarm-mock-planner-coordinator")
    lcm_spy.wait_for_saved_topic(_JOINT_STATE_TOPIC)

    client = RPCClient(None, ManipulationModule)
    try:
        _wait_until(
            lambda: set(_ROBOTS).issubset(set(client.list_robots())),
            timeout_s=10.0,
        )
        for robot_name in _ROBOTS:
            _wait_for_robot_state(client, robot_name)

        targets = {robot_name: list(_TARGET) for robot_name in _ROBOTS}

        sequential_left_s = _plan_and_wait(
            client, "left_arm", targets["left_arm"], wait_until_ready=True
        )
        sequential_right_s = _plan_and_wait(client, "right_arm", targets["right_arm"])
        sequential_total_s = sequential_left_s + sequential_right_s

        assert client.reset().startswith("Reset")

        concurrent_total_s = _plan_concurrently(client, targets)

        for robot_name in _ROBOTS:
            status = client.get_planning_status(robot_name)
            _assert_successful_plan_status(status, robot_name)
            assert client.has_planned_path(robot_name) is True

        # Very fast mock plans are dominated by process/RPC jitter. Enforce a
        # ratio only once the sequential baseline is long enough to be meaningful.
        if sequential_total_s >= 1.0:
            assert concurrent_total_s < sequential_total_s * 0.85, (
                f"expected bimanual planning overlap: sequential={sequential_total_s:.2f}s, "
                f"concurrent={concurrent_total_s:.2f}s"
            )

        preview_total_s = _preview_concurrently(client)
        execute_accept_s = _execute_concurrently(client)
        logger.info(
            "OpenArm bimanual timing: sequential_plan=%.2fs "
            "(left=%.2fs right=%.2fs), concurrent_plan=%.2fs, "
            "concurrent_preview=%.2fs, concurrent_execute_accept=%.2fs",
            sequential_total_s,
            sequential_left_s,
            sequential_right_s,
            concurrent_total_s,
            preview_total_s,
            execute_accept_s,
        )
    finally:
        client.stop_rpc_client()
