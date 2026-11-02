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

import asyncio
from unittest.mock import call

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.experimental.patrolling.module import PatrollingModule
from dimos.navigation.go2.replanning_a_star.module_spec import ReplanningAStarPlannerSpec


@pytest.fixture
async def patrol(mocker):
    module = PatrollingModule()
    try:
        mocker.patch.object(
            module, "_planner_spec", mocker.Mock(spec=ReplanningAStarPlannerSpec), create=True
        )
        mocker.patch.object(module.goal_request, "publish")
        mocker.patch.object(module, "stop_tool")
        module._latest_pose = PoseStamped()
        yield module
    finally:
        module.stop()


async def test_stopping_idle_patrol_does_not_call_the_planner(patrol) -> None:
    assert await patrol.stop_patrol() == "Patrol stopped."

    assert patrol._planner_spec.mock_calls == []
    patrol.goal_request.publish.assert_not_called()
    patrol.stop_tool.assert_called_once_with("start_patrol")


async def test_stopping_active_patrol_cancels_once(patrol, mocker) -> None:
    mocker.patch.object(patrol, "_patrol_loop", asyncio.Event().wait)
    await patrol.start_patrol()
    task = patrol._patrol_task
    try:
        assert patrol.is_patrolling()
        await patrol.stop_patrol()
        await patrol.stop_patrol()

        assert task.cancelled()
        assert not patrol.is_patrolling()
        assert patrol._planner_spec.set_replanning_enabled.call_args_list == [
            call(False),
            call(True),
        ]
        patrol._planner_spec.reset_safe_goal_clearance.assert_called_once_with()
        patrol.goal_request.publish.assert_called_once_with(patrol._latest_pose)
    finally:
        await patrol.stop_patrol()


async def test_stopping_after_failed_start_restores_the_planner(patrol) -> None:
    patrol._planner_spec.set_safe_goal_clearance.side_effect = RuntimeError("planner down")
    with pytest.raises(RuntimeError):
        await patrol.start_patrol()

    await patrol.stop_patrol()

    assert not patrol.is_patrolling()
    assert patrol._planner_spec.set_replanning_enabled.call_args_list == [call(False), call(True)]
    patrol._planner_spec.reset_safe_goal_clearance.assert_called_once_with()
