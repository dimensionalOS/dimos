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

"""Turn raw Quest buttons into stack-level operator actions."""

from __future__ import annotations

from pydantic import field_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons


def _button_attribute(name: str) -> str:
    attribute = BUTTON_ALIASES.get(name, name)
    if attribute not in Buttons.BITS:
        raise ValueError(f"unknown Quest button {name!r}")
    return attribute


class QuestActionBindingsConfig(ModuleConfig):
    """Buttons that drive one primary action and a manual override."""

    primary_button: str = "A"
    override_buttons: tuple[str, ...] = ("LG", "RG")

    @field_validator("primary_button")
    @classmethod
    def validate_primary_button(cls, name: str) -> str:
        _button_attribute(name)
        return name

    @field_validator("override_buttons")
    @classmethod
    def validate_override_buttons(cls, names: tuple[str, ...]) -> tuple[str, ...]:
        if not names:
            raise ValueError("override_buttons must not be empty")
        for name in names:
            _button_attribute(name)
        return names


class QuestActionBindingsModule(Module):
    """Publish edge-triggered primary actions and level-triggered overrides."""

    config: QuestActionBindingsConfig

    teleop_buttons: In[Buttons]
    primary_action: Out[Bool]
    manual_override: Out[Bool]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._primary_button = _button_attribute(self.config.primary_button)
        self._override_buttons = tuple(
            _button_attribute(name) for name in self.config.override_buttons
        )
        self._primary_pressed = False
        self._override_active = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))

    def _on_buttons(self, buttons: Buttons) -> None:
        override_active = any(bool(getattr(buttons, name)) for name in self._override_buttons)
        if override_active != self._override_active:
            self.manual_override.publish(Bool(data=override_active))
        self._override_active = override_active

        primary_pressed = bool(getattr(buttons, self._primary_button))
        if primary_pressed and not self._primary_pressed and not override_active:
            self.primary_action.publish(Bool(data=True))
        self._primary_pressed = primary_pressed
