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

import json
from pathlib import Path
import re
from typing import Protocol

import matplotlib  # type: ignore[import-not-found]

matplotlib.use("Agg")
from matplotlib import pyplot as plt  # type: ignore[import-not-found]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid

DEFAULT_SURFACE_LEVEL = 0.1


class _AxisLike(Protocol):
    def imshow(self, values: object, **kwargs: object) -> object: ...

    def set_title(self, title: str) -> object: ...

    def set_xlabel(self, label: str) -> object: ...

    def set_ylabel(self, label: str) -> object: ...


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
    slices_path = root / f"{safe_prefix}_slices.png"
    summary_path = root / f"{safe_prefix}_summary.json"

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
    _write_slice_image(slices_path, tsdf)
    _write_summary(summary_path, tsdf)
    return [npz_path, near_surface_path, observed_path, slices_path, summary_path]


def _near_surface_points(tsdf: TSDFGrid) -> np.ndarray:
    field = tsdf.distances[0]
    mask = np.abs(field) <= DEFAULT_SURFACE_LEVEL
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


def _write_slice_image(path: Path, tsdf: TSDFGrid) -> None:
    field = tsdf.distances[0]
    weights = _weights(tsdf)
    observed = weights > 0.0 if weights is not None else np.ones_like(field, dtype=bool)
    surface = np.logical_and(np.abs(field) <= DEFAULT_SURFACE_LEVEL, observed)

    x_mid = field.shape[0] // 2
    y_mid = field.shape[1] // 2
    z_mid = field.shape[2] // 2

    fig, axes = plt.subplots(2, 3, figsize=(12, 8), constrained_layout=True)
    fig.suptitle(
        f"TSDF {tsdf.frame_id} origin=({tsdf.origin.x:.3f}, {tsdf.origin.y:.3f}, {tsdf.origin.z:.3f}) "
        f"voxel={tsdf.voxel_size:.4f} observed={int(observed.sum())} surface={int(surface.sum())}"
    )

    _plot_slice(axes[0, 0], field[x_mid, :, :], f"distance x={x_mid}")
    _plot_slice(axes[0, 1], field[:, y_mid, :], f"distance y={y_mid}")
    _plot_slice(axes[0, 2], field[:, :, z_mid], f"distance z={z_mid}")
    _plot_mask(axes[1, 0], observed.max(axis=2), "observed XY projection")
    _plot_mask(axes[1, 1], surface.max(axis=2), "surface XY projection")
    if weights is None:
        _plot_mask(axes[1, 2], observed[:, :, z_mid], f"observed z={z_mid}")
    else:
        _plot_weight(axes[1, 2], weights[:, :, z_mid], f"weight z={z_mid}")

    fig.savefig(path, dpi=160)
    plt.close(fig)


def _write_summary(path: Path, tsdf: TSDFGrid) -> None:
    field = tsdf.distances[0]
    weights = _weights(tsdf)
    observed = weights > 0.0 if weights is not None else np.ones_like(field, dtype=bool)
    surface = np.logical_and(np.abs(field) <= DEFAULT_SURFACE_LEVEL, observed)
    observed_values = field[observed]
    summary = {
        "frame_id": tsdf.frame_id,
        "ts": tsdf.ts,
        "origin": [tsdf.origin.x, tsdf.origin.y, tsdf.origin.z],
        "voxel_size": tsdf.voxel_size,
        "truncation_distance": tsdf.truncation_distance,
        "resolution": list(tsdf.resolution),
        "surface_level": DEFAULT_SURFACE_LEVEL,
        "observed_voxels": int(observed.sum()),
        "surface_voxels": int(surface.sum()),
        "distance_min": float(observed_values.min()) if observed_values.size else None,
        "distance_max": float(observed_values.max()) if observed_values.size else None,
        "distance_mean": float(observed_values.mean()) if observed_values.size else None,
    }
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")


def _weights(tsdf: TSDFGrid) -> np.ndarray | None:
    if tsdf.weights is None:
        return None
    return tsdf.weights[0] if tsdf.weights.ndim == 4 else tsdf.weights


def _plot_slice(axis: _AxisLike, values: np.ndarray, title: str) -> None:
    image = axis.imshow(values.T, origin="lower", cmap="coolwarm", vmin=-1.0, vmax=1.0)
    axis.set_title(title)
    axis.set_xlabel("i")
    axis.set_ylabel("j")
    plt.colorbar(image, ax=axis, fraction=0.046, pad=0.04)


def _plot_mask(axis: _AxisLike, values: np.ndarray, title: str) -> None:
    axis.imshow(values.T, origin="lower", cmap="gray", vmin=0.0, vmax=1.0)
    axis.set_title(title)
    axis.set_xlabel("x")
    axis.set_ylabel("y")


def _plot_weight(axis: _AxisLike, values: np.ndarray, title: str) -> None:
    image = axis.imshow(values.T, origin="lower", cmap="viridis")
    axis.set_title(title)
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    plt.colorbar(image, ax=axis, fraction=0.046, pad=0.04)


def _safe_prefix(prefix: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", prefix).strip("_") or "tsdf"
