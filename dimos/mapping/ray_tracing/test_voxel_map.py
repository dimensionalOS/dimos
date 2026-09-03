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
IDENTITY = (0.0, 0.0, 0.0, 1.0)


def make_mapper() -> VoxelRayMapper:
    return VoxelRayMapper(voxel_size=1.0, max_range=100.0, min_health=0, max_health=1)


def test_add_frame_populates_global_map() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN, IDENTITY)

    assert mapper.voxel_count() == 1
    centers = mapper.global_map()
    assert centers.shape == (1, 3)
    assert centers.dtype == np.float32
    np.testing.assert_allclose(centers[0], [5.5, 0.5, 0.5])


def test_add_frame_registers_by_pose() -> None:
    mapper = make_mapper()
    # Yaw 90 deg about z: sensor +x becomes world +y.
    half = np.sqrt(2.0) / 2.0
    mapper.add_frame(
        np.array([[3.5, 0.0, 0.5]], dtype=np.float32),
        (10.0, 0.0, 0.0),
        (0.0, 0.0, half, half),
    )
    world = mapper.registered_points()
    np.testing.assert_allclose(world[0], [10.0, 3.5, 0.5], atol=1e-5)
    np.testing.assert_allclose(mapper.global_map()[0], [10.5, 3.5, 0.5], atol=1e-5)


def test_empty_frame_is_accepted() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.empty((0, 3), dtype=np.float32), ORIGIN, IDENTITY)

    assert mapper.voxel_count() == 0
    assert mapper.global_map().shape == (0, 3)


def test_wrong_shape_is_rejected() -> None:
    mapper = make_mapper()
    with pytest.raises(ValueError):
        mapper.add_frame(np.zeros((2, 2), dtype=np.float32), ORIGIN, IDENTITY)


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
    mapper.add_frame(points, ORIGIN, IDENTITY)

    assert mapper.voxel_count() == 1


def test_local_map_filters_by_cylinder() -> None:
    # support_min=0 so the cylinder bound is the only filter under test.
    mapper = VoxelRayMapper(
        voxel_size=1.0, max_range=100.0, min_health=0, max_health=1, support_min=0
    )
    points = np.array([[2.5, 0.5, 0.5], [50.5, 0.5, 0.5]], dtype=np.float32)
    mapper.add_frame(points, ORIGIN, IDENTITY)

    assert mapper.voxel_count() == 2
    local = mapper.local_map(ORIGIN, radius=10.0, z_min=-5.0, z_max=5.0)
    assert local.shape == (1, 3)
    np.testing.assert_allclose(local[0], [2.5, 0.5, 0.5])


def test_frames_batch_only_when_emit_every_is_set() -> None:
    points = np.array([[5.5, 0.5, 0.5]], dtype=np.float32)

    silent = make_mapper()
    silent.add_frame(points, ORIGIN, IDENTITY)
    assert silent.take_local_bounds()[2] == 0.0

    batching = VoxelRayMapper(
        voxel_size=1.0, max_range=100.0, min_health=0, max_health=1, emit_every=1
    )
    batching.add_frame(points, ORIGIN, IDENTITY)
    assert batching.take_local_bounds()[2] > 0.0
    assert batching.take_local_bounds()[2] == 0.0


def test_seed_points_creates_only_absent_voxels() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN, IDENTITY)

    cloud = np.array([[5.6, 0.6, 0.6], [7.5, 0.5, 0.5]], dtype=np.float32)
    assert mapper.seed_points(cloud) == 1
    assert mapper.voxel_count() == 2
    centers = np.sort(mapper.global_map(), axis=0)
    np.testing.assert_allclose(centers, [[5.5, 0.5, 0.5], [7.5, 0.5, 0.5]])

    assert mapper.seed_points(cloud) == 0


def test_add_frame_world_registers_at_world_coordinates() -> None:
    mapper = make_mapper()
    points = np.array([[105.55, 200.05, 3.05]], dtype=np.float32)
    mapper.add_frame_world(points, (100.0, 200.0, 3.0))
    np.testing.assert_allclose(mapper.registered_points(), points)
    np.testing.assert_allclose(mapper.global_map()[0], [105.5, 200.5, 3.5])


def test_local_map_fine_emits_fine_centers() -> None:
    mapper = VoxelRayMapper(voxel_size=1.0, max_range=100.0, fine_divisor=2, support_min=0)
    mapper.add_frame(np.array([[5.1, 0.1, 0.1]], dtype=np.float32), ORIGIN, IDENTITY)
    fine = mapper.local_map_fine((5.0, 0.0, 0.0), 5.0, -1.0, 1.0)
    np.testing.assert_allclose(fine, [[5.25, 0.25, 0.25]])


def test_local_map_fine_requires_fine_divisor() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN, IDENTITY)
    with pytest.raises(ValueError, match="fine_divisor"):
        mapper.local_map_fine((0.0, 0.0, 0.0), 10.0, -1.0, 1.0)


def test_clear_resets_map() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN, IDENTITY)
    assert len(mapper) == 1

    mapper.clear()
    assert mapper.voxel_count() == 0
    assert len(mapper) == 0


def test_global_map_normals_matches_global_map() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN, IDENTITY)

    centers, normals = mapper.global_map_normals()
    assert centers.shape == normals.shape == (mapper.voxel_count(), 3)
    assert centers.dtype == normals.dtype == np.float32
    np.testing.assert_allclose(centers, mapper.global_map())


def test_global_map_normals_empty_map() -> None:
    centers, normals = make_mapper().global_map_normals()
    assert centers.shape == normals.shape == (0, 3)


def test_global_map_normal_fits_matches_shapes() -> None:
    mapper = make_mapper()
    mapper.add_frame(np.array([[5.5, 0.5, 0.5]], dtype=np.float32), ORIGIN, IDENTITY)

    centers, normals, min_eigs = mapper.global_map_normal_fits()
    assert centers.shape == normals.shape == (mapper.voxel_count(), 3)
    assert min_eigs.shape == (mapper.voxel_count(),)
    assert min_eigs.dtype == np.float32
    np.testing.assert_allclose(np.sort(centers, axis=0), np.sort(mapper.global_map(), axis=0))


def test_global_map_normal_fits_min_eigs_track_planarity() -> None:
    mapper = make_mapper()
    rng = np.random.default_rng(0)
    n = 64
    flat = np.column_stack(
        [rng.uniform(5.1, 5.9, n), rng.uniform(0.1, 0.9, n), np.full(n, 0.5)]
    ).astype(np.float32)
    rough = np.column_stack(
        [rng.uniform(9.1, 9.9, n), rng.uniform(5.1, 5.9, n), rng.uniform(3.4, 3.6, n)]
    ).astype(np.float32)
    mapper.add_frame(np.vstack([flat, rough]), ORIGIN, IDENTITY)

    centers, normals, min_eigs = mapper.global_map_normal_fits()
    flat_i = np.flatnonzero((centers == [5.5, 0.5, 0.5]).all(axis=1))[0]
    rough_i = np.flatnonzero((centers == [9.5, 5.5, 3.5]).all(axis=1))[0]
    assert min_eigs[flat_i] < 1e-5, "an exactly coplanar patch fits with near-zero residual"
    assert min_eigs[rough_i] > 1e-4, "a jittered patch fits with a visible residual"
    np.testing.assert_allclose(np.abs(normals[flat_i]), [0.0, 0.0, 1.0], atol=1e-3)
    np.testing.assert_allclose(np.abs(normals[rough_i]), [0.0, 0.0, 1.0], atol=0.05)
