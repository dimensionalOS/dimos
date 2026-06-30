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

from typing import cast

import numpy as np
import pytest
import rerun as rr

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.reconstruction_msgs.ReconstructionStatus import ReconstructionStatus
from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid


def test_tsdf_grid_shape_origin_and_voxel_position() -> None:
    distances = np.arange(24, dtype=np.float32).reshape(1, 2, 3, 4)
    grid = TSDFGrid(
        distances=distances,
        voxel_size=0.05,
        truncation_distance=0.15,
        origin=Vector3(1.0, 2.0, 3.0),
        frame_id="map",
        ts=12.25,
    )

    assert grid.distances.shape == (1, 2, 3, 4)
    assert grid.distances.dtype == np.float32
    assert grid.resolution == (2, 3, 4)
    assert grid.size == pytest.approx((0.1, 0.15, 0.2))
    assert (
        grid.voxel_position(0, 0, 0).x,
        grid.voxel_position(0, 0, 0).y,
        grid.voxel_position(0, 0, 0).z,
    ) == pytest.approx((1.0, 2.0, 3.0))
    assert (
        grid.voxel_position(1, 2, 3).x,
        grid.voxel_position(1, 2, 3).y,
        grid.voxel_position(1, 2, 3).z,
    ) == pytest.approx((1.05, 2.1, 3.15))


def test_tsdf_grid_lcm_round_trip_preserves_arrays_and_metadata() -> None:
    distances = np.linspace(-1.0, 1.0, 24, dtype=np.float32).reshape(1, 2, 3, 4)
    weights = np.linspace(0.0, 2.3, 24, dtype=np.float32).reshape(1, 2, 3, 4)
    grid = TSDFGrid(
        distances=distances,
        weights=weights,
        voxel_size=0.02,
        truncation_distance=0.08,
        origin=Vector3(-0.1, 0.2, 0.3),
        frame_id="camera",
        ts=42.5,
    )

    decoded = TSDFGrid.lcm_decode(grid.lcm_encode())

    assert decoded.distances.shape == distances.shape
    assert decoded.distances.dtype == np.float32
    np.testing.assert_array_equal(decoded.distances, distances)
    assert decoded.weights is not None
    assert decoded.weights.shape == weights.shape
    assert decoded.weights.dtype == np.float32
    np.testing.assert_array_equal(decoded.weights, weights)
    assert decoded.voxel_size == pytest.approx(0.02)
    assert decoded.truncation_distance == pytest.approx(0.08)
    assert (decoded.origin.x, decoded.origin.y, decoded.origin.z) == pytest.approx((-0.1, 0.2, 0.3))
    assert decoded.frame_id == "camera"
    assert decoded.ts == pytest.approx(42.5)


def test_tsdf_grid_rejects_invalid_shape() -> None:
    with pytest.raises(ValueError, match="shape"):
        TSDFGrid(np.zeros((2, 3, 4), dtype=np.float32), voxel_size=0.1, truncation_distance=0.2)

    with pytest.raises(ValueError, match="shape"):
        TSDFGrid(np.zeros((2, 2, 3, 4), dtype=np.float32), voxel_size=0.1, truncation_distance=0.2)


def test_tsdf_grid_to_rerun_filters_unobserved_surface_voxels() -> None:
    distances = np.ones((1, 2, 2, 2), dtype=np.float32)
    distances[0, 0, 0, 0] = 0.0
    distances[0, 0, 1, 1] = 0.08
    distances[0, 1, 1, 1] = 0.0
    weights = np.zeros((2, 2, 2), dtype=np.float32)
    weights[0, 1, 1] = 1.0
    weights[1, 1, 1] = 1.0
    grid = TSDFGrid(
        distances=distances,
        weights=weights,
        voxel_size=0.05,
        truncation_distance=0.2,
        origin=Vector3(),
    )

    archetype = grid.to_rerun()

    assert isinstance(archetype, rr.Points3D)
    points = cast("rr.Points3D", archetype)
    assert len(points.positions.as_arrow_array().to_pylist()) == 2


def test_reconstruction_status_lcm_round_trip() -> None:
    status = ReconstructionStatus(
        integrated_frames=7,
        dropped_frames=2,
        last_error="sensor timeout",
        active=True,
        paused=True,
        latest_integration_ts=99.5,
        workspace_origin=Vector3(0.1, -0.2, 0.3),
        workspace_size=0.75,
        frame_id="base",
        ts=101.125,
    )

    decoded = ReconstructionStatus.lcm_decode(status.lcm_encode())

    assert decoded.integrated_frames == 7
    assert decoded.dropped_frames == 2
    assert decoded.last_error == "sensor timeout"
    assert decoded.active is True
    assert decoded.paused is True
    assert decoded.latest_integration_ts == pytest.approx(99.5)
    assert (
        decoded.workspace_origin.x,
        decoded.workspace_origin.y,
        decoded.workspace_origin.z,
    ) == pytest.approx((0.1, -0.2, 0.3))
    assert decoded.workspace_size == pytest.approx(0.75)
    assert decoded.frame_id == "base"
    assert decoded.ts == pytest.approx(101.125)
