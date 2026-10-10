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

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.navigation.bench.ground_truth import GO2, GroundTruth
from dimos.simulation.scenes.procedural import Door, Scene, office


def _room(width: float = 6.0, length: float = 4.0) -> Scene:
    """A closed empty room with its floor at z=0 and the start in one corner."""
    scene = Scene("room", start=(1.0, 1.0, 0.0))
    scene.add((0.0, 0.0, -0.15), (width, length, 0.0), "floor")
    scene.add((0.0, 0.0, 2.6), (width, length, 2.75), "ceiling")
    for lo, hi in (
        ((-0.1, -0.1), (0.0, length + 0.1)),
        ((width, -0.1), (width + 0.1, length + 0.1)),
        ((0.0, -0.1), (width, 0.0)),
        ((0.0, length), (width, length + 0.1)),
    ):
        scene.add((*lo, -0.15), (*hi, 2.75), "wall")
    return scene


@pytest.fixture(scope="module")
def office_truth() -> GroundTruth:
    return GroundTruth(office(1))


def test_floor_is_ground_and_walls_and_the_outside_are_not(office_truth: GroundTruth) -> None:
    gt = office_truth
    z0 = gt.scene.params["z0"]
    assert gt.height[gt.index((1.0, 1.0))] == pytest.approx(z0, abs=1e-6)
    assert np.isnan(gt.height[gt.index((-0.05, 3.0))])
    assert gt.stands((1.0, 1.0))
    assert not gt.stands((0.1, 3.0))
    assert gt.clearance[gt.index((1.0, 1.0))] == pytest.approx(1.0, abs=0.1)


def _far_goal(gt: GroundTruth) -> NDArray[np.float64]:
    far = np.unravel_index(
        np.argmax(np.where(gt.walkable, np.add.outer(*map(np.arange, gt.shape)), -1)), gt.shape
    )
    return gt.center(*far)


def test_route_through_the_office_stays_on_walkable_ground(office_truth: GroundTruth) -> None:
    gt = office_truth
    goal = _far_goal(gt)
    route = gt.route(gt.scene.start, goal)
    assert route is not None
    assert all(gt.stands(p) for p in route.points)
    assert np.allclose(route.points[:, 2], gt.scene.params["z0"], atol=1e-6)
    assert np.allclose(route.points[0, :2], gt.center(*gt.index(gt.scene.start))[:2])
    assert np.allclose(route.points[-1, :2], goal[:2])
    difficulty = gt.difficulty(route)
    assert difficulty.min_clearance >= GO2.radius
    assert difficulty.doors >= 1
    assert difficulty.detour >= 1.0
    centered = gt.route(gt.scene.start, goal, centered=True)
    assert centered is not None
    assert centered.clearance.mean() >= route.clearance.mean()
    assert centered.length >= route.length


def test_premap_cloud_is_what_the_lidar_sees_along_the_route(office_truth: GroundTruth) -> None:
    gt = office_truth
    goal = _far_goal(gt)
    route = gt.route(gt.scene.start, goal, centered=True)
    assert route is not None
    cloud = gt.premap_cloud(route.points)
    lo, hi = gt.scene.bounds()
    assert len(cloud) > 20_000
    assert np.all(cloud >= lo - gt.cell) and np.all(cloud <= hi + gt.cell)
    z0 = gt.scene.params["z0"]
    assert np.mean(np.abs(cloud[:, 2] - z0) < 0.05) > 0.1
    assert np.any(cloud[:, 2] > z0 + 2.5)
    near_route = np.min(
        np.linalg.norm(cloud[:, None, :2] - route.points[None, ::20, :2], axis=2), axis=1
    )
    assert np.mean(near_route < 3.0) > 0.5
    keys = np.unique(np.floor((cloud - lo) / gt.cell).astype(np.int64), axis=0)
    assert len(keys) > 0.99 * len(cloud)


def test_no_route_to_a_wall_or_an_unreachable_cell(office_truth: GroundTruth) -> None:
    assert office_truth.route(office_truth.scene.start, (0.05, 3.0)) is None


def test_low_overhang_blocks_and_high_overhang_does_not() -> None:
    scene = _room()
    scene.add((2.0, 1.0, 0.3), (3.0, 3.0, 0.34), "clutter")
    scene.add((4.0, 1.0, 0.7), (5.0, 3.0, 0.74), "clutter")
    gt = GroundTruth(scene)
    assert not gt.stands((2.5, 2.0))
    assert gt.stands((4.5, 2.0))
    assert gt.route((1.0, 1.0), (4.5, 2.0)) is not None


def test_a_small_step_is_ground_and_a_large_one_is_not() -> None:
    scene = _room()
    scene.add((2.0, 1.0, 0.0), (3.0, 3.0, 0.15), "clutter")
    scene.add((4.0, 1.0, 0.0), (5.0, 3.0, 0.3), "clutter")
    gt = GroundTruth(scene)
    assert gt.height[gt.index((2.5, 2.0))] == pytest.approx(0.15)
    assert gt.stands((2.5, 2.0))
    assert not gt.stands((4.5, 2.0))
    route = gt.route((1.0, 1.0), (2.5, 2.0))
    assert route is not None
    assert route.points[-1, 2] == pytest.approx(0.15)


def test_difficulty_counts_doors_and_nearby_clutter() -> None:
    scene = _room()
    scene.add((3.0, 0.0, 0.0), (3.1, 1.5, 2.6), "wall")
    scene.add((3.0, 2.5, 0.0), (3.1, 4.0, 2.6), "wall")
    scene.doors.append(Door(axis=0, at=3.0, start=1.5, width=1.0))
    scene.add((4.0, 1.5, 0.0), (4.4, 1.9, 0.5), "clutter")
    scene.add((1.0, 3.0, 0.0), (1.4, 3.4, 0.5), "clutter")
    gt = GroundTruth(scene)
    route = gt.route((1.0, 1.0), (5.0, 2.0))
    assert route is not None
    difficulty = gt.difficulty(route)
    assert difficulty.doors == 1
    assert difficulty.clutter_near == 1
    assert difficulty.detour > 1.0
    assert GO2.radius <= difficulty.min_clearance <= 0.5 + gt.cell
    same_side = gt.route((1.0, 1.0), (1.0, 2.5))
    assert same_side is not None
    assert gt.difficulty(same_side).doors == 0
