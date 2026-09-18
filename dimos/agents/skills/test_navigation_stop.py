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

from collections.abc import Iterator
from unittest.mock import Mock

import pytest
from pytest_mock import MockerFixture

from dimos.agents.skills.navigation_stop import NavigationStopSkill
from dimos.navigation.navigation_spec import NavigationInterfaceSpec


@pytest.fixture
def skill_module(mocker: MockerFixture) -> Iterator[tuple[NavigationStopSkill, Mock]]:
    module = NavigationStopSkill()
    navigation = mocker.Mock(spec=NavigationInterfaceSpec)
    mocker.patch.object(module, "_navigation", navigation, create=True)
    try:
        yield module, navigation
    finally:
        module.stop()


def test_stop_cancels_navigation(skill_module: tuple[NavigationStopSkill, Mock]) -> None:
    module, navigation = skill_module
    assert module.stop_navigation() == "Stopped"
    navigation.cancel_goal.assert_called_once_with()


def test_stop_failure_is_not_reported_as_success(
    skill_module: tuple[NavigationStopSkill, Mock],
) -> None:
    module, navigation = skill_module
    navigation.cancel_goal.side_effect = RuntimeError("connection lost")
    with pytest.raises(RuntimeError, match="connection lost"):
        module.stop_navigation()
