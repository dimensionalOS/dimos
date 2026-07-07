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

"""Tests for the ``progress`` context manager."""

from __future__ import annotations

import pytest

from dimos.memory2.type.observation import Observation
from dimos.memory2.utils.progress import progress


def _obs(ts: float) -> Observation[None]:
    return Observation(ts=ts, _data=None)


def test_completed_bar_persists_one_line(capsys: pytest.CaptureFixture[str]) -> None:
    with progress(3, "render") as bar:
        for i in range(3):
            bar(_obs(float(i)))
    out = capsys.readouterr().out
    assert out.count("render 100% [3/3]") == 1


def test_early_exit_finalizes_partial_bar(capsys: pytest.CaptureFixture[str]) -> None:
    with progress(4, "windowed") as bar:
        bar(_obs(0.0))
        bar(_obs(0.1))
    out = capsys.readouterr().out
    assert out.count("windowed 50% [2/4]") == 1


def test_cleanup_on_exception(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(RuntimeError, match="boom"):
        with progress(10, "crashy") as bar:
            bar(_obs(0.0))
            raise RuntimeError("boom")
    out = capsys.readouterr().out
    assert out.count("crashy 10% [1/10]") == 1


def test_calls_after_completion_are_ignored(capsys: pytest.CaptureFixture[str]) -> None:
    with progress(2, "tap") as bar:
        for i in range(5):  # observations may keep flowing after the bar completes
            bar(_obs(float(i)))
    out = capsys.readouterr().out
    assert out.count("tap 100% [2/2]") == 1


def test_zero_total_reports_complete(capsys: pytest.CaptureFixture[str]) -> None:
    with progress(0, "empty"):
        pass
    out = capsys.readouterr().out
    assert out.count("empty 100% [0/0]") == 1


def test_sequential_bars_each_persist(capsys: pytest.CaptureFixture[str]) -> None:
    for label in ("first", "second"):
        with progress(1, label) as bar:
            bar(_obs(0.0))
    out = capsys.readouterr().out
    assert out.count("first 100% [1/1]") == 1
    assert out.count("second 100% [1/1]") == 1
