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

from __future__ import annotations

from pathlib import Path
import re

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid


def export_tsdf_debug_files(tsdf: TSDFGrid, output_dir: str | Path, prefix: str) -> list[Path]:
    """Write TSDF debug artifacts for offline inspection.

    The raw ``.npz`` is the authoritative grid. The ``.ply`` files are helper
    visualizations that can be opened in Open3D, MeshLab, or CloudCompare.
    """
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    safe_prefix = _safe_prefix(prefix)

    npz_path = root / f"{safe_prefix}.npz"
    near_surface_path = root / f"{safe_prefix}_near_surface.ply"
    observed_path = root / f"{safe_prefix}_observed.ply"

    np.savez_compressed(
        npz_path,
        distances=tsdf.distances,
        weights=tsdf.weights if tsdf.weights is not None else np.array([], dtype=np.float32),
        origin=np.array([tsdf.origin.x, tsdf.origin.y, tsdf.origin.z], dtype=np.float32),
        voxel_size=np.array(tsdf.voxel_size, dtype=np.float32),
        truncation_distance=np.array(tsdf.truncation_distance, dtype=np.float32),
        frame_id=np.array(tsdf.frame_id),
        ts=np.array(tsdf.ts, dtype=np.float64),
    )

    _write_point_cloud(near_surface_path, _near_surface_points(tsdf))
    _write_point_cloud(observed_path, _observed_points(tsdf))
    return [npz_path, near_surface_path, observed_path]


def _near_surface_points(tsdf: TSDFGrid) -> np.ndarray:
    field = tsdf.distances[0]
    mask = np.abs(field) <= tsdf.voxel_size
    if tsdf.weights is not None:
        weights = tsdf.weights[0] if tsdf.weights.ndim == 4 else tsdf.weights
        mask = np.logical_and(mask, weights > 0.0)
    return _points_from_mask(tsdf, mask)


def _observed_points(tsdf: TSDFGrid) -> np.ndarray:
    if tsdf.weights is None:
        return np.empty((0, 3), dtype=np.float64)
    weights = tsdf.weights[0] if tsdf.weights.ndim == 4 else tsdf.weights
    return _points_from_mask(tsdf, weights > 0.0)


def _points_from_mask(tsdf: TSDFGrid, mask: np.ndarray) -> np.ndarray:
    indices = np.argwhere(mask)
    if len(indices) == 0:
        return np.empty((0, 3), dtype=np.float64)
    origin = np.array([tsdf.origin.x, tsdf.origin.y, tsdf.origin.z], dtype=np.float64)
    return origin + indices.astype(np.float64) * tsdf.voxel_size


def _write_point_cloud(path: Path, points: np.ndarray) -> None:
    pcd = o3d.geometry.PointCloud()
    if len(points) > 0:
        pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(str(path), pcd, write_ascii=False)


def _safe_prefix(prefix: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", prefix).strip("_") or "tsdf"
