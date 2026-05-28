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

"""Load a 3D mesh, sample points on its surfaces, voxel-downsample."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def load_voxelized_mesh(
    path: str | Path,
    voxel_size: float = 0.1,
    num_sample_points: int = 2_000_000,
    swap_y_up_to_z_up: bool = True,
    recenter: bool = True,
) -> np.ndarray:
    """Load a mesh file, sample its surface, voxel-downsample.

    GLB/glTF files use Y-up; we rotate to Z-up so the result drops into the
    planner's frame. ``recenter`` translates so the XY bbox is centered on
    the origin and the floor (minimum z) sits at z=0.
    """
    mesh = o3d.io.read_triangle_mesh(str(path))
    if len(mesh.vertices) == 0:
        raise ValueError(f"Mesh {path!r} has no vertices")
    logger.info(
        "Mesh loaded",
        path=str(path),
        vertices=len(mesh.vertices),
        triangles=len(mesh.triangles),
    )

    o3d.utility.random.seed(42)
    pcd = mesh.sample_points_uniformly(number_of_points=num_sample_points)
    points = np.asarray(pcd.points)

    if swap_y_up_to_z_up:
        # 90 deg rotation around X: (x, y, z) to (x, -z, y).
        points = np.column_stack([points[:, 0], -points[:, 2], points[:, 1]])

    if recenter:
        xy_center = (points[:, :2].max(axis=0) + points[:, :2].min(axis=0)) / 2
        points[:, :2] -= xy_center
        points[:, 2] -= points[:, 2].min()

    # Snap each occupied cell to its voxel-grid center, with the grid
    # anchored at world origin so cells line up cleanly across scenarios.
    quantized = (np.floor(points / voxel_size) + 0.5) * voxel_size
    centers = np.unique(quantized, axis=0).astype(np.float32)

    logger.info(
        "Voxelized mesh ready",
        voxels=len(centers),
        voxel_size=voxel_size,
        bbox_min=centers.min(axis=0).round(2).tolist(),
        bbox_max=centers.max(axis=0).round(2).tolist(),
    )
    return centers
