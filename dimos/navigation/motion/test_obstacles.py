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
import pytest

from dimos.navigation.motion.embodiment.go2 import GO2, GO2_PAYLOAD
from dimos.navigation.motion.embodiment.synthetic import DIFFDRIVE, SLIM
from dimos.navigation.motion.obstacles import (
    LOW,
    OBSTACLE_MODELS,
    BodyBand,
    hard_points,
    load,
)


def _room(base_z: float) -> np.ndarray:
    """A ground slab 0..0.12 m thick under a 0.30 m obstacle, lifted to base_z.

    The recording's geometry: the map's z origin is base height, so absolute z
    says nothing until it is referenced to the surface the feet stand on.
    """
    ground = np.array([[x, 0.0, z] for x in (-1.0, 0.0, 1.0) for z in (0.0, 0.04, 0.08, 0.12)])
    post = np.array([[2.0, 0.0, z] for z in (0.18, 0.24, 0.30)])
    return (np.concatenate([ground, post]) + np.array([0.0, 0.0, base_z])).astype(np.float32)


def test_body_band_drops_the_ground_slab_and_keeps_the_obstacle():
    """The phantom regression: a quantised floor must not become a wall."""
    ground_z = -0.28  # base at +0.01, base_height 0.29
    out = hard_points(BodyBand(GO2), _room(ground_z), ground_z)
    assert len(out) == 3
    assert np.allclose(np.sort(out[:, 2]), [0.18, 0.24, 0.30], atol=1e-6)


def test_body_band_looks_under_the_belly_not_over_it():
    # a table top above the body is not something the body can hit
    cloud = np.array([[1.0, 0.0, 0.3], [1.0, 0.0, GO2.height + 0.01]], dtype=np.float32)
    out = hard_points(BodyBand(GO2), cloud, 0.0)
    assert len(out) == 1 and out[0][2] == pytest.approx(0.3)


def test_the_ground_exclusion_is_two_voxel_layers():
    assert LOW == 0.16
    cloud = np.array([[0.0, 0.0, LOW], [0.0, 0.0, LOW + 0.01]], dtype=np.float32)
    assert len(hard_points(BodyBand(GO2), cloud, 0.0)) == 1


def test_a_tall_body_keeps_what_the_absolute_band_would_have_cut_off():
    # The latent bug the 2D search contract closes: a body taller than the old
    # 0.05..0.45 slice had its correctly-kept obstacles truncated by a SECOND
    # cut downstream. There is only one cut now, and it is this one.
    tall = replace(GO2, tag="tall", height=0.60)
    cloud = np.array([[1.0, 0.0, 0.55], [1.0, 0.5, 0.61]], dtype=np.float32)
    out = hard_points(BodyBand(tall), cloud, 0.0)
    assert len(out) == 1 and out[0][2] == pytest.approx(0.55)


def test_the_registry_names_the_model():
    assert sorted(OBSTACLE_MODELS) == ["body_band"]
    assert isinstance(load("body_band", GO2), BodyBand)
    with pytest.raises(ValueError, match="unknown obstacle model"):
        load("floor_anchor", GO2)


def test_every_embodiment_builds_every_model():
    for emb in (GO2, GO2_PAYLOAD, SLIM, DIFFDRIVE):
        for name in OBSTACLE_MODELS:
            assert load(name, emb) is not None
