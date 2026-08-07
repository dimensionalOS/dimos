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

"""The diagnosis tool's measurement primitives."""

from __future__ import annotations

import numpy as np

from dimos.navigation.motion.adapter.diagnose import (
    Crop,
    arclen,
    divergence,
    is_hold,
    resample,
    voxel_centers,
    voxel_keys,
)


def test_resample_walks_even_arc_length():
    line = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    out = resample(line, step=0.5)
    steps = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert np.allclose(steps, 0.5)
    assert abs(arclen(line) - 2.0) < 1e-9


def test_divergence_is_the_offset_between_parallel_plans():
    a = np.column_stack([np.linspace(0, 2, 21), np.zeros(21)])
    assert divergence(a, a) == 0.0
    assert abs(divergence(a, a + np.array([0.0, 0.3])) - 0.3) < 1e-6


def test_divergence_compares_only_the_shared_arc():
    a = np.column_stack([np.linspace(0, 4, 41), np.zeros(41)])
    assert divergence(a, a[:11]) < 1e-6  # a prefix of the same route has not changed its mind


def test_single_pose_path_is_a_hold():
    assert is_hold(np.zeros((1, 2)))
    assert not is_hold(np.zeros((2, 2)))


def test_voxel_keys_round_trip_through_centres():
    pts = np.array([[0.01, 0.01, 0.30], [0.05, 0.05, 0.31], [-1.0, 2.0, 0.0]])
    keys = voxel_keys(pts, 0.08)
    assert len(keys) == 2  # the first two land in the same 0.08 m voxel
    centres = voxel_centers(keys, 0.08)
    for p in pts:
        assert np.abs(centres - p).max(axis=1).min() <= 0.0401


def test_crop_margin_excludes_the_window_edge():
    crop = Crop(centre=np.array([0.0, 0.0]), radius=2.0, z_lo=-1.0, z_hi=1.0)
    pts = np.array([[0.0, 0.0, 0.0], [1.95, 0.0, 0.0], [0.0, 0.0, 0.99]])
    assert list(crop.inside(pts, margin=0.16)) == [True, False, False]
