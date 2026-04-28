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

from __future__ import annotations

from dimos_lcm.std_msgs import Bool

from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal


class _Out:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


def test_stop_movement_does_not_publish_phantom_goal() -> None:
    click_to_goal = ClickToGoal.__new__(ClickToGoal)
    click_to_goal.way_point = _Out()
    click_to_goal.goal = _Out()

    click_to_goal._on_stop_movement(Bool(data=True))

    assert click_to_goal.way_point.messages == []
    assert click_to_goal.goal.messages == []
