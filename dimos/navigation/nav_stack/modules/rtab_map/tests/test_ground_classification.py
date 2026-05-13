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

"""Validate ground vs obstacle segmentation via surface normal angle.

Mirrors RTAB-Map's ``Grid/MaxGroundAngle`` parameter: a point is ground iff
its locally-estimated surface normal is within ``max_ground_angle_deg`` of
the +Z axis. Flat floor passes; vertical wall fails.
"""

from __future__ import annotations

import numpy as np

from dimos.navigation.nav_stack.modules.rtab_map._rtab_runner import (
    LiteRtabRunner,
    LiteRtabRunnerConfig,
)


def _flat_ground_cloud() -> np.ndarray:
    xx, yy = np.meshgrid(np.linspace(-1.0, 1.0, 8), np.linspace(-1.0, 1.0, 8))
    return np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], axis=1)


def _vertical_wall_cloud() -> np.ndarray:
    yy, zz = np.meshgrid(np.linspace(-1.0, 1.0, 8), np.linspace(0.0, 2.0, 8))
    return np.stack([np.full(yy.size, 1.0), yy.ravel(), zz.ravel()], axis=1)


def test_flat_ground_classified_as_ground(lite_runner: LiteRtabRunner) -> None:
    cloud = _flat_ground_cloud()
    mask = lite_runner.classify_ground(cloud, max_ground_angle_deg=45.0)
    assert mask.all(), "every point on a flat floor should be ground"


def test_vertical_wall_classified_as_obstacle(lite_runner: LiteRtabRunner) -> None:
    cloud = _vertical_wall_cloud()
    mask = lite_runner.classify_ground(cloud, max_ground_angle_deg=45.0)
    assert not mask.any(), "every point on a vertical wall should be obstacle"


def test_tight_angle_rejects_almost_flat_slope(lite_runner: LiteRtabRunner) -> None:
    """At max_ground_angle=5°, a 20° slope is no longer ground."""
    xx, yy = np.meshgrid(np.linspace(-1.0, 1.0, 8), np.linspace(-1.0, 1.0, 8))
    slope_rad = np.deg2rad(20.0)
    zz = xx * np.tan(slope_rad)
    sloped = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)

    permissive = lite_runner.classify_ground(sloped, max_ground_angle_deg=45.0)
    strict = lite_runner.classify_ground(sloped, max_ground_angle_deg=5.0)
    assert permissive.all(), "20° slope passes at 45° tolerance"
    assert not strict.any(), "20° slope fails at 5° tolerance"


def test_mixed_floor_and_wall_classifies_separately() -> None:
    """The split must respect normal-angle independently per neighborhood —
    one cloud, half floor + half wall, well separated."""
    runner = LiteRtabRunner(LiteRtabRunnerConfig(ground_neighbor_count=6))
    floor = _flat_ground_cloud()
    wall = _vertical_wall_cloud() + np.array([10.0, 0.0, 0.0])  # far away
    cloud = np.concatenate([floor, wall])

    mask = runner.classify_ground(cloud, max_ground_angle_deg=45.0)
    n_floor = len(floor)
    assert mask[:n_floor].all(), "floor portion misclassified"
    assert not mask[n_floor:].any(), "wall portion misclassified"
