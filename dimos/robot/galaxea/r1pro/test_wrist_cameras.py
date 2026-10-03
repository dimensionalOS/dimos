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

from __future__ import annotations

from typing import Any

from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.wrist_cameras import (
    WRIST_LEFT_COLOR_V4L2,
    WRIST_LEFT_DEPTH_V4L2,
    WRIST_RIGHT_COLOR_V4L2,
    WRIST_RIGHT_DEPTH_V4L2,
    WristLeftCamera,
    WristLeftColorDepth,
    WristRightCamera,
    WristRightColorDepth,
)


def _atoms(wrist_depth: bool) -> dict[type, dict[str, Any]]:
    blueprint = r1pro_control(wrist_depth=wrist_depth)
    return {atom.module: atom.kwargs for atom in blueprint.active_blueprints}


def test_colour_only_uses_the_colour_module() -> None:
    atoms = _atoms(wrist_depth=False)

    assert atoms[WristLeftCamera]["device"] == WRIST_LEFT_COLOR_V4L2
    assert atoms[WristRightCamera]["device"] == WRIST_RIGHT_COLOR_V4L2
    assert WristLeftColorDepth not in atoms and WristRightColorDepth not in atoms


def test_depth_opens_each_wrist_as_one_colour_depth_pair() -> None:
    atoms = _atoms(wrist_depth=True)

    assert WristLeftCamera not in atoms and WristRightCamera not in atoms
    left, right = atoms[WristLeftColorDepth], atoms[WristRightColorDepth]
    assert (left["color_device"], left["depth_device"]) == (
        WRIST_LEFT_COLOR_V4L2,
        WRIST_LEFT_DEPTH_V4L2,
    )
    assert (right["color_device"], right["depth_device"]) == (
        WRIST_RIGHT_COLOR_V4L2,
        WRIST_RIGHT_DEPTH_V4L2,
    )


def test_wrist_streams_keep_their_names_with_depth() -> None:
    remaps = set(r1pro_control(wrist_depth=True).remapping_map.values())

    assert {
        "wrist_left_color",
        "wrist_right_color",
        "wrist_left_depth",
        "wrist_right_depth",
    } <= remaps
