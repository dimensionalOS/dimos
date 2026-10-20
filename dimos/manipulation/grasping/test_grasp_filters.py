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

import math

import numpy as np
import pytest

from dimos.manipulation.grasping import grasp_filters as gf

DOWN = np.asarray([[1.0, 0.0, 0.0, 0.0]])  # x y z w: half turn about x, approach -z


def _pose(position: tuple[float, float, float], quat: np.ndarray = DOWN) -> np.ndarray:
    return gf.pose_matrices(np.asarray([position]), quat)


def test_the_gripper_samples_cover_fingers_and_body() -> None:
    gripper = gf.box_surface_points()
    assert gripper[:, 2].max() == pytest.approx(0.0, abs=1e-9)  # fingertips end at the TCP
    assert gripper[:, 2].min() == pytest.approx(-0.17, abs=1e-3)
    assert np.abs(gripper[:, 1]).max() == pytest.approx(0.055, abs=1e-6)  # the body


def test_a_top_down_grasp_reaches_down_to_its_tcp() -> None:
    placed = gf.gripper_in_world(_pose((0.5, 0.0, 0.83)), gf.box_surface_points())
    assert placed[0, :, 2].min() == pytest.approx(0.83, abs=1e-6)
    assert gf.tilt_degrees(_pose((0.5, 0.0, 0.83)))[0] == pytest.approx(0.0, abs=1e-6)


def test_below_support_and_colliding() -> None:
    gripper = gf.box_surface_points()
    poses = np.concatenate([_pose((0.5, 0.0, 0.83)), _pose((0.5, 0.0, 0.801))])
    assert gf.below_support(poses, 0.80, gripper).tolist() == [False, True]
    table = np.column_stack(
        [np.random.default_rng(0).uniform([0.4, -0.1], [0.6, 0.1], (2000, 2)), np.full(2000, 0.80)]
    )
    assert gf.colliding(poses, table, gripper).tolist() == [False, True]


def test_support_height_is_the_densest_layer() -> None:
    rng = np.random.default_rng(1)
    table = np.column_stack([rng.uniform(0, 1, (500, 2)), rng.normal(0.80, 0.001, 500)])
    clutter = rng.uniform([0, 0, 0.81], [1, 1, 0.9], (100, 3))
    assert gf.support_height(np.vstack([table, clutter]), below=0.95) == pytest.approx(
        0.80, abs=0.003
    )
    assert gf.support_height(clutter[:10], below=0.95) is None


def test_a_side_approach_is_ninety_degrees_tilted() -> None:
    side = np.asarray(
        [[0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)]]
    )  # quarter turn about y
    assert gf.tilt_degrees(_pose((0.5, 0.0, 0.9), side))[0] == pytest.approx(90.0, abs=1e-6)
