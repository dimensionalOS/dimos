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

from dataclasses import replace

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.local_planner.obstacles import (
    LOW,
    OBSTACLE_MODELS,
    BodyBand,
    hard_points,
    load,
)

# Three heights just above the ground exclusion, so the fixture follows LOW.
POST_HEIGHTS = (LOW + 0.02, LOW + 0.08, LOW + 0.14)


def _room(base_z: float) -> NDArray[np.float64]:
    """A ground slab 0..0.12 m thick under an obstacle just above LOW, lifted to base_z.

    The recording's geometry: the map's z origin is base height, so absolute z
    says nothing until it is referenced to the surface the feet stand on.
    """
    ground = np.array([[x, 0.0, z] for x in (-1.0, 0.0, 1.0) for z in (0.0, 0.04, 0.08, 0.12)])
    post = np.array([[2.0, 0.0, z] for z in POST_HEIGHTS])
    return (np.concatenate([ground, post]) + np.array([0.0, 0.0, base_z])).astype(np.float32)


def test_body_band_drops_the_ground_slab_and_keeps_the_obstacle():
    """The phantom regression: a quantised floor must not become a wall."""
    ground_z = -0.28  # base at +0.01, base_height 0.29
    out = hard_points(BodyBand(GO2), _room(ground_z), ground_z)
    assert len(out) == 3
    assert np.allclose(np.sort(out[:, 2]), POST_HEIGHTS, atol=1e-6)


def test_body_band_reaches_the_top_of_the_robot_and_no_further():
    # a table top above the robot is not something it can hit
    cloud = np.array([[1.0, 0.0, 0.3], [1.0, 0.0, GO2.height + 0.01]], dtype=np.float32)
    out = hard_points(BodyBand(GO2), cloud, 0.0)
    assert len(out) == 1 and out[0][2] == pytest.approx(0.3)


def test_the_ground_exclusion_is_two_voxel_layers():
    cloud = np.array([[0.0, 0.0, LOW], [0.0, 0.0, LOW + 0.01]], dtype=np.float32)
    assert len(hard_points(BodyBand(GO2), cloud, 0.0)) == 1


def test_without_the_lidar_a_wall_over_the_trunk_is_not_a_wall():
    trunk = replace(GO2, height=0.45)
    cloud = np.array([[1.0, 0.0, 0.55]], dtype=np.float32)
    assert len(hard_points(BodyBand(GO2), cloud, 0.0)) == 1
    assert not len(hard_points(BodyBand(trunk), cloud, 0.0))


def test_a_non_finite_return_is_dropped_not_planned_on():
    # one NaN x reaches the search as a NaN grid corner and the whole plan raises
    cloud = np.array(
        [[np.nan, 0.0, 0.3], [1.0, np.inf, 0.3], [1.0, 0.0, np.nan], [1.0, 0.0, 0.3]],
        dtype=np.float32,
    )
    out = hard_points(BodyBand(GO2), cloud, 0.0)
    assert len(out) == 1 and np.isfinite(out).all()


def test_the_registry_names_the_model():
    assert sorted(OBSTACLE_MODELS) == ["body_band"]
    assert isinstance(load("body_band", GO2), BodyBand)
    with pytest.raises(ValueError, match="unknown obstacle model"):
        load("floor_anchor", GO2)
