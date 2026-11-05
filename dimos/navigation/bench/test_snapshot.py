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

from dataclasses import replace
import math

import numpy as np
from numpy.typing import NDArray

from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.snapshot import (
    CORNERS_DEG,
    FOVY_DEG,
    SNAPSHOT_PX,
    View,
    _corner,
    key_points,
    sample,
    view,
)
from dimos.navigation.bench.suite import Case, Level
from dimos.simulation.scenes.procedural import Family, Scene


def _case(family: Family, param: str, value: float, seed: int = 1, **places: object) -> Case:
    base = {"start": (2.0, 3.0, 0.5), "goal": (6.0, 4.0, 0.0), "feature": None}
    return Case(
        family=family,
        seed=seed,
        params={param: value},
        tag=family,
        id=f"{family}-s{seed}-{value}",
        split="dev",
        difficulty=Difficulty(0.5, 0, 1.0, 0),
        route_length=3.0,
        scene_digest="",
        level=Level(param, value),
        **{**base, **places},  # type: ignore[arg-type]
    )


def _in_frame(seen: View, point: tuple[float, ...]) -> NDArray[np.float64]:
    """The point across and up the picture, each within one when it is in frame."""
    forward = seen.forward
    right = np.array([forward[1], -forward[0], 0.0]) / math.hypot(forward[0], forward[1])
    offset = np.array(point) - seen.position
    reach = math.tan(math.radians(FOVY_DEG) / 2) * (offset @ forward)
    aspect = SNAPSHOT_PX[0] / SNAPSHOT_PX[1]
    return np.array([offset @ right / (reach * aspect), offset @ np.cross(right, forward) / reach])


def _corners(scene: Scene) -> list[tuple[float, float, float]]:
    lo, hi = scene.bounds()
    return [(x, y, scene.start[2]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]


def test_sample_is_the_lowest_seed_at_the_middle_level() -> None:
    cases = [
        _case("block", "block_height", value, seed) for seed in (2, 1) for value in (0.1, 0.2, 0.4)
    ]
    assert sample(cases) is cases[4]
    assert sample([]) is None


def test_the_view_looks_from_the_start_toward_the_goal_and_frames_the_room() -> None:
    case = _case("block", "block_height", 0.2, feature=(5.0, 3.5))
    scene = case.scene()
    seen = view(case, scene)
    assert seen.azimuth == 45.0 and seen.faded == ()
    assert seen == view(case, scene)
    for point in (*_corners(scene), *key_points(case, scene)):
        assert np.abs(_in_frame(seen, point)).max() <= 1.0
    assert max(np.abs(_in_frame(seen, corner)).max() for corner in _corners(scene)) > 0.9
    back = view(replace(case, start=(*case.goal[:2], 0.0), goal=case.start), scene)
    assert back.azimuth == 225.0


def test_the_two_outer_walls_under_the_camera_are_cut_and_inner_walls_are_kept() -> None:
    case = _case("doorway", "door_width", 0.6, goal=(8.0, 4.0, 0.0))
    scene = case.scene()
    seen = view(case, scene)
    lo, _ = scene.bounds()
    assert len(seen.cut) == 2
    assert len(seen.faded) == min(len(_corner(case, scene, a).faded) for a in CORNERS_DEG)
    for i in seen.cut:
        box = scene.boxes[i]
        assert box.kind == "wall" and (
            box.center[0] - box.half[0] == lo[0] or box.center[1] - box.half[1] == lo[1]
        )
    (door,) = scene.doors
    inner = [i for i, box in enumerate(scene.boxes) if abs(box.center[0] - door.at) < 0.2]
    assert len(inner) > 1 and not set(inner) & set(seen.cut)
