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

"""Tests for :class:`EpisodeBoundary` — Quest-button → recorder rollover."""

from __future__ import annotations

import time
from typing import Any

from dimos.teleop.quest.episode_boundary import EpisodeBoundary
from dimos.teleop.quest.quest_types import Buttons


class _FakeRecorder:
    """Stateful fake — flips an internal RECORDING/IDLE bit on every toggle.

    Default is IDLE (matches the real recorder, which starts idle until the
    operator presses the toggle button)."""

    def __init__(self) -> None:
        self.rotate_calls = 0
        self._recording = False

    def toggle_recording(self) -> object:
        self.rotate_calls += 1
        self._recording = not self._recording
        return "/tmp/episode.rrd" if self._recording else None


def _make_buttons(**flags: bool) -> Buttons:
    b = Buttons()
    for name, value in flags.items():
        setattr(b, name, bool(value))
    return b


def _make_boundary(button: str = "right_secondary", debounce: float = 0.5) -> Any:
    eb = EpisodeBoundary(button=button, debounce_seconds=debounce)
    eb.recorder = _FakeRecorder()
    return eb


def test_rising_edge_calls_rotate_once() -> None:
    eb = _make_boundary()
    try:
        eb._on_buttons(_make_buttons(right_secondary=False))
        assert eb.recorder.rotate_calls == 0
        eb._on_buttons(_make_buttons(right_secondary=True))
        assert eb.recorder.rotate_calls == 1
        # Stays high — no additional rotate.
        eb._on_buttons(_make_buttons(right_secondary=True))
        eb._on_buttons(_make_buttons(right_secondary=True))
        assert eb.recorder.rotate_calls == 1
    finally:
        eb.stop()


def test_debounced_press_counts_once() -> None:
    eb = _make_boundary(debounce=0.5)
    try:
        eb._on_buttons(_make_buttons(right_secondary=True))
        eb._on_buttons(_make_buttons(right_secondary=False))
        # Re-press within the debounce window.
        eb._on_buttons(_make_buttons(right_secondary=True))
        assert eb.recorder.rotate_calls == 1
    finally:
        eb.stop()


def test_presses_outside_debounce_window_count_twice() -> None:
    eb = _make_boundary(debounce=0.01)
    try:
        eb._on_buttons(_make_buttons(right_secondary=True))
        eb._on_buttons(_make_buttons(right_secondary=False))
        time.sleep(0.03)
        eb._on_buttons(_make_buttons(right_secondary=True))
        assert eb.recorder.rotate_calls == 2
    finally:
        eb.stop()


def test_held_button_does_not_call_rotate() -> None:
    eb = _make_boundary()
    try:
        # Multiple high reads in a row — only one rising edge.
        for _ in range(10):
            eb._on_buttons(_make_buttons(right_secondary=True))
        assert eb.recorder.rotate_calls == 1
    finally:
        eb.stop()


def test_high_low_high_low_high_counts_two_presses() -> None:
    """A press = a True following a False. Two presses means two rotates,
    provided they're outside the debounce window."""
    eb = _make_boundary(debounce=0.01)
    try:
        eb._on_buttons(_make_buttons(right_secondary=False))
        eb._on_buttons(_make_buttons(right_secondary=True))
        eb._on_buttons(_make_buttons(right_secondary=False))
        time.sleep(0.03)
        eb._on_buttons(_make_buttons(right_secondary=True))
        eb._on_buttons(_make_buttons(right_secondary=False))
        assert eb.recorder.rotate_calls == 2
    finally:
        eb.stop()


def test_unknown_button_name_logs_and_skips() -> None:
    eb = _make_boundary(button="no_such_button")
    try:
        eb._on_buttons(_make_buttons(right_secondary=True))
        assert eb.recorder.rotate_calls == 0
    finally:
        eb.stop()
