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

"""Terminal navigation results with a fake planner and clock."""

import pytest

from dimos.navigation import skill_navigation
from dimos.navigation.base import NavigationState


def test_wait_reports_verified_arrival(mocker):
    nav = mocker.Mock()
    nav.is_goal_reached.side_effect = [False, True]
    nav.get_state.return_value = NavigationState.FOLLOWING_PATH
    mocker.patch.object(skill_navigation.time, "sleep")
    result = skill_navigation.wait_for_navigation(nav)
    assert result.success
    assert result.metadata["status"] == "succeeded"
    nav.cancel_goal.assert_not_called()


def test_timeout_cancels_goal_and_returns_failure(mocker):
    nav = mocker.Mock()
    result = skill_navigation.wait_for_navigation(nav, timeout=0)
    assert not result.success
    assert result.error_code == "EXECUTION_TIMEOUT"
    nav.cancel_goal.assert_called_once_with()


def test_observation_error_stops_goal(mocker):
    nav = mocker.Mock()
    nav.is_goal_reached.side_effect = RuntimeError("connection lost")
    with pytest.raises(RuntimeError, match="connection lost"):
        skill_navigation.wait_for_navigation(nav)
    nav.cancel_goal.assert_called_once_with()


def test_idle_after_replanning_reports_failure(mocker):
    nav = mocker.Mock()
    nav.is_goal_reached.return_value = False
    nav.get_state.return_value = NavigationState.IDLE
    mocker.patch.object(skill_navigation.time, "monotonic", side_effect=[0, 0, 0, 3, 3])
    mocker.patch.object(skill_navigation.time, "sleep")
    result = skill_navigation.wait_for_navigation(nav)
    assert result.error_code == "NAVIGATION_STOPPED"
    nav.cancel_goal.assert_called_once_with()
