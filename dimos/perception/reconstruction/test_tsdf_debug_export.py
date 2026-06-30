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
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid
from dimos.perception.reconstruction.tsdf_debug_export import export_tsdf_debug_files


def test_export_tsdf_debug_files_writes_raw_grid_and_open3d_pointclouds(tmp_path) -> None:  # type: ignore[no-untyped-def]
    distances = np.ones((1, 3, 3, 3), dtype=np.float32)
    distances[0, 1, 1, 1] = 0.0
    weights = np.zeros((3, 3, 3), dtype=np.float32)
    weights[1, 1, 1] = 2.0
    weights[2, 2, 2] = 1.0
    grid = TSDFGrid(
        distances=distances,
        weights=weights,
        voxel_size=0.05,
        truncation_distance=0.2,
        origin=Vector3(0.1, 0.2, 0.3),
        frame_id="world",
        ts=12.0,
    )

    paths = export_tsdf_debug_files(grid, tmp_path, "target masked/tsdf")

    assert {path.name for path in paths} == {
        "target_masked_tsdf.npz",
        "target_masked_tsdf_near_surface.ply",
        "target_masked_tsdf_observed.ply",
        "target_masked_tsdf_slices.png",
        "target_masked_tsdf_summary.json",
    }
    with np.load(tmp_path / "target_masked_tsdf.npz", allow_pickle=False) as data:
        np.testing.assert_array_equal(data["distances"], distances)
        np.testing.assert_array_equal(data["weights"], weights)
        np.testing.assert_allclose(data["origin"], np.array([0.1, 0.2, 0.3], dtype=np.float32))
        assert str(data["frame_id"]) == "world"

    near_surface = o3d.io.read_point_cloud(str(tmp_path / "target_masked_tsdf_near_surface.ply"))
    observed = o3d.io.read_point_cloud(str(tmp_path / "target_masked_tsdf_observed.ply"))
    assert len(near_surface.points) == 1
    assert len(observed.points) == 2
    assert (tmp_path / "target_masked_tsdf_slices.png").stat().st_size > 0
    assert '"observed_voxels": 2' in (tmp_path / "target_masked_tsdf_summary.json").read_text()
    assert '"surface_voxels": 1' in (tmp_path / "target_masked_tsdf_summary.json").read_text()
