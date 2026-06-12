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

from typing import Any

import pytest

import dimos.teleop.keyboard.keyboard_teleop_module as keyboard_mod
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


class _NoInitialJointState:
    def get_next(self, timeout: float) -> Any:
        raise TimeoutError


class _PublishedCommands:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.messages.append(msg)


def test_keyboard_teleop_exits_without_publishing_when_initial_state_is_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(keyboard_mod, "pygame", object())
    module = KeyboardTeleopModule(initial_state_timeout=0.01)
    published = _PublishedCommands()
    module.joint_state = _NoInitialJointState()  # type: ignore[assignment]
    module.cartesian_command = published  # type: ignore[assignment]

    try:
        module.start()
        assert module._thread is not None
        module._thread.join(timeout=1.0)

        assert not module._thread.is_alive()
        assert published.messages == []
    finally:
        module.stop()
