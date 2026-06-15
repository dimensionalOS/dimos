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

import numpy as np
import pytest

pytest.importorskip("dimos_voxel_ray_tracing")

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper

ORIGIN = (0.0, 0.0, 0.0)


def make_mapper() -> VoxelRayMapper:
    return VoxelRayMapper(voxel_size=1.0, max_range=100.0, min_health=0, max_health=1)


def test_add_frame_populates_global_map() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN)

    assert mapper.voxel_count() == 1
    centers = mapper.global_map()
    assert centers.shape == (1, 3)
    assert centers.dtype == np.float32
    np.testing.assert_allclose(centers[0], [5.5, 0.5, 0.5])


def test_empty_frame_is_accepted() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.empty((0, 3), dtype=np.float32), ORIGIN)

    assert mapper.voxel_count() == 0
    assert mapper.global_map().shape == (0, 3)


def test_wrong_shape_is_rejected() -> None:
    mapper = make_mapper()
    with pytest.raises(ValueError):
        mapper.add_frame(np.zeros((2, 2), dtype=np.float32), ORIGIN)


def test_nonfinite_points_are_dropped() -> None:
    mapper = make_mapper()
    points = np.array(
        [
            [5.5, 0.5, 0.5],
            [np.nan, 0.5, 0.5],
            [np.inf, 0.5, 0.5],
        ],
        dtype=np.float32,
    )
    mapper.add_frame(points, ORIGIN)

    assert mapper.voxel_count() == 1


def test_local_map_filters_by_cylinder() -> None:
    mapper = make_mapper()
    points = np.array([[2.5, 0.5, 0.5], [50.5, 0.5, 0.5]], dtype=np.float32)
    mapper.add_frame(points, ORIGIN)

    assert mapper.voxel_count() == 2
    local = mapper.local_map(ORIGIN, radius=10.0, z_min=-5.0, z_max=5.0)
    assert local.shape == (1, 3)
    np.testing.assert_allclose(local[0], [2.5, 0.5, 0.5])


def test_clear_resets_map() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN)
    assert len(mapper) == 1

    mapper.clear()
    assert mapper.voxel_count() == 0
    assert len(mapper) == 0


def test_global_map_normals_matches_global_map() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN)

    centers, normals = mapper.global_map_normals()
    assert centers.shape == normals.shape == (mapper.voxel_count(), 3)
    assert centers.dtype == normals.dtype == np.float32
    np.testing.assert_allclose(centers, mapper.global_map())


def test_global_map_normals_empty_map() -> None:
    centers, normals = make_mapper().global_map_normals()
    assert centers.shape == normals.shape == (0, 3)
