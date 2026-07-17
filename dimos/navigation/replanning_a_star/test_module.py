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
from pytest_mock import MockerFixture

from dimos.navigation.replanning_a_star import module as planner_module
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner


def test_stop_delegates_goal_cleanup_to_global_planner(mocker: MockerFixture) -> None:
    planner_type = mocker.patch.object(planner_module, "GlobalPlanner", autospec=True)
    module = ReplanningAStarPlanner()

    try:
        module.stop()
    finally:
        module._close_module()

    planner_type.return_value.stop.assert_called_once_with()
    planner_type.return_value.cancel_goal.assert_not_called()


def test_stop_closes_module_when_global_planner_stop_fails(mocker: MockerFixture) -> None:
    planner_type = mocker.patch.object(planner_module, "GlobalPlanner", autospec=True)
    planner_type.return_value.stop.side_effect = RuntimeError("planner stop failed")
    module = ReplanningAStarPlanner()

    try:
        with pytest.raises(RuntimeError, match="planner stop failed"):
            module.stop()
        assert module._module_closed
    finally:
        module._close_module()
