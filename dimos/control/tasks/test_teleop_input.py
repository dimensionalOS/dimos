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

"""Freshness contract of the shared WebXR teleop input layer."""

from typing import Any, cast

from dimos.control.tasks.teleop_input import TeleopInput
from dimos.teleop.webxr.controller_types import Buttons


class _Pose:
    def __init__(self, x: float = 0.0) -> None:
        self.x = x


def _buttons(left: bool, right: bool) -> Buttons:
    msg = Buttons()
    msg.left_grip = left
    msg.right_grip = right
    return msg


def _input(left: bool = True, right: bool = True, t: float = 0.0) -> TeleopInput:
    source = TeleopInput()
    source.on_buttons(_buttons(left, right), t)
    source.on_pose("left", cast("Any", _Pose()), t)
    source.on_pose("right", cast("Any", _Pose()), t)
    return source


def test_fresh_needs_both_grips_and_every_source():
    source = _input()
    assert source.deadman
    assert source.fresh(0.1, 0.2)
    assert not _input(left=False).deadman
    assert not _input(left=False).fresh(0.1, 0.2)


def test_one_live_source_cannot_reauthorize_a_stale_one():
    source = _input(t=0.0)
    source.on_pose("right", cast("Any", _Pose()), 0.5)
    assert source.ages(0.5) == (0.5, 0.5, 0.0)
    assert not source.fresh(0.5, 0.2)


def test_missing_source_is_never_fresh():
    source = TeleopInput()
    assert source.ages(0.0) is None
    assert not source.fresh(0.0, 1.0)


def test_clear_drops_samples_and_grips():
    source = _input()
    source.clear()
    assert source.ages(0.1) is None
    assert not source.deadman
