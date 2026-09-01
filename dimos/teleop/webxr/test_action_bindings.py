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

from collections.abc import Iterator

import pytest

from dimos.teleop.quest.action_bindings import QuestActionBindingsModule
from dimos.teleop.quest.quest_types import Buttons


@pytest.fixture
def module() -> Iterator[QuestActionBindingsModule]:
    action_bindings = QuestActionBindingsModule()
    yield action_bindings
    action_bindings.stop()


def test_primary_action_publishes_only_on_rising_edges(
    module: QuestActionBindingsModule,
) -> None:
    actions: list[bool] = []
    module.primary_action.subscribe(lambda message: actions.append(message.data))
    pressed = Buttons()
    pressed.right_primary = True

    module._on_buttons(pressed)
    module._on_buttons(pressed)
    module._on_buttons(Buttons())
    module._on_buttons(pressed)

    assert actions == [True, True]


def test_manual_override_publishes_state_transitions(
    module: QuestActionBindingsModule,
) -> None:
    states: list[bool] = []
    module.manual_override.subscribe(lambda message: states.append(message.data))
    held = Buttons()
    held.left_grip = True

    module._on_buttons(held)
    module._on_buttons(held)
    module._on_buttons(Buttons())

    assert states == [True, False]


def test_manual_override_suppresses_primary_action(
    module: QuestActionBindingsModule,
) -> None:
    actions: list[bool] = []
    module.primary_action.subscribe(lambda message: actions.append(message.data))
    buttons = Buttons()
    buttons.right_primary = True
    buttons.right_grip = True

    module._on_buttons(buttons)

    assert actions == []


def test_unknown_button_configuration_fails_at_construction() -> None:
    with pytest.raises(ValueError, match="unknown Quest button"):
        QuestActionBindingsModule(primary_button="NOPE")
