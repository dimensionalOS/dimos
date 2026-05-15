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

"""Tests for :class:`QuestRolloutToggle` — Quest-button → policy rollout toggle.

Mirrors `test_quest_episode_boundary`. We drive `_on_buttons` directly with
a fake `PolicyModule` so the tests stay hermetic — no LCM transport, no
inference thread, no backend.
"""

from __future__ import annotations

import time
from typing import Any

from dimos.manipulation.policy.quest_rollout_toggle import QuestRolloutToggle
from dimos.teleop.quest.quest_types import Buttons


class _FakePolicyModule:
    """Fake `PolicyModule` exposing the surface `QuestRolloutToggle` touches.

    Stateful: start/stop_rollout flip an internal bit so `is_rollout_active`
    returns the new value on the next call (matches the real node)."""

    def __init__(self) -> None:
        self.start_calls = 0
        self.stop_calls = 0
        self._active = False

    def is_rollout_active(self) -> bool:
        return self._active

    def start_rollout(self) -> None:
        self.start_calls += 1
        self._active = True

    def stop_rollout(self) -> None:
        self.stop_calls += 1
        self._active = False


def _make_buttons(**flags: bool) -> Buttons:
    b = Buttons()
    for name, value in flags.items():
        setattr(b, name, bool(value))
    return b


def _make_toggle(button: str = "left_secondary", debounce: float = 0.5) -> Any:
    rt = QuestRolloutToggle(button=button, debounce_seconds=debounce)
    rt.policy_module = _FakePolicyModule()
    return rt


def test_default_button_is_left_secondary() -> None:
    rt = QuestRolloutToggle()
    try:
        assert rt.config.button == "left_secondary"
    finally:
        rt.stop()


def test_rising_edge_starts_rollout() -> None:
    rt = _make_toggle()
    try:
        rt._on_buttons(_make_buttons(left_secondary=False))
        assert rt.policy_module.start_calls == 0
        rt._on_buttons(_make_buttons(left_secondary=True))
        assert rt.policy_module.start_calls == 1
        assert rt.policy_module.stop_calls == 0
        assert rt.policy_module.is_rollout_active() is True
    finally:
        rt.stop()


def test_rising_edge_when_active_stops_rollout() -> None:
    rt = _make_toggle()
    try:
        # Force active state.
        rt.policy_module._active = True

        rt._on_buttons(_make_buttons(left_secondary=False))
        rt._on_buttons(_make_buttons(left_secondary=True))
        assert rt.policy_module.stop_calls == 1
        assert rt.policy_module.start_calls == 0
        assert rt.policy_module.is_rollout_active() is False
    finally:
        rt.stop()


def test_sustained_press_does_not_retoggle() -> None:
    rt = _make_toggle()
    try:
        rt._on_buttons(_make_buttons(left_secondary=True))
        rt._on_buttons(_make_buttons(left_secondary=True))
        rt._on_buttons(_make_buttons(left_secondary=True))
        assert rt.policy_module.start_calls == 1
        assert rt.policy_module.stop_calls == 0
    finally:
        rt.stop()


def test_falling_edge_ignored() -> None:
    rt = _make_toggle()
    try:
        rt._on_buttons(_make_buttons(left_secondary=True))
        assert rt.policy_module.start_calls == 1
        # Falling edge — must NOT dispatch.
        rt._on_buttons(_make_buttons(left_secondary=False))
        assert rt.policy_module.start_calls == 1
        assert rt.policy_module.stop_calls == 0
    finally:
        rt.stop()


def test_debounce_window_suppresses_repeated_edges() -> None:
    rt = _make_toggle(debounce=0.5)
    try:
        rt._on_buttons(_make_buttons(left_secondary=True))
        rt._on_buttons(_make_buttons(left_secondary=False))
        # Re-press within the debounce window.
        rt._on_buttons(_make_buttons(left_secondary=True))
        # Only the first rising edge dispatches; the second is debounced.
        assert rt.policy_module.start_calls + rt.policy_module.stop_calls == 1
    finally:
        rt.stop()


def test_debounce_allows_second_edge_after_window() -> None:
    rt = _make_toggle(debounce=0.05)
    try:
        rt._on_buttons(_make_buttons(left_secondary=True))
        assert rt.policy_module.start_calls == 1
        rt._on_buttons(_make_buttons(left_secondary=False))
        time.sleep(0.1)
        rt._on_buttons(_make_buttons(left_secondary=True))
        # After the window, the next rising edge dispatches — stop_rollout
        # because is_rollout_active is now True.
        assert rt.policy_module.stop_calls == 1
    finally:
        rt.stop()


def test_unknown_button_name_logs_and_skips() -> None:
    rt = _make_toggle(button="not_a_real_button")
    try:
        rt._on_buttons(_make_buttons(left_secondary=True))
        assert rt.policy_module.start_calls == 0
        assert rt.policy_module.stop_calls == 0
        # Subsequent unknown-button presses still don't dispatch and don't
        # spam logs (the warning is rate-limited via _unknown_button_logged).
        rt._on_buttons(_make_buttons(left_secondary=True))
        rt._on_buttons(_make_buttons(left_secondary=True))
        assert rt.policy_module.start_calls == 0
    finally:
        rt.stop()
