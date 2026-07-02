# Copyright 2025-2026 Dimensional Inc.
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

"""Reachability-map scene layer for the Viser manipulation visualizer."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.reachability.capability_map import CapabilityMap
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.visualization.viser.scene import ViserManipulationScene

logger = setup_logger()

_EMPTY_GRAY = (45, 45, 55)


def body_point_cloud(
    cap: CapabilityMap, min_dexterity: float, min_count: int = 1
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return occupied body-frame cell centers and per-cell dexterity."""
    params = cap.params
    dexterity = cap.body_dexterity()
    keep = (cap.body_counts >= min_count) & (dexterity >= min_dexterity)
    iz, ix, iy = np.nonzero(keep)
    if len(iz) == 0:
        return np.empty((0, 3)), np.empty(0)
    centers = (np.arange(params.n_xy) + 0.5) * params.cell - params.r_xy
    z_centers = (np.arange(params.n_z) + 0.5) * params.cell + params.z_min
    points = np.stack([centers[ix], centers[iy], z_centers[iz]], axis=1)
    return points, dexterity[iz, ix, iy]


def body_voxel_mesh(
    cap: CapabilityMap, min_dexterity: float, min_count: int = 1
) -> tuple[Any, int]:
    """Return a Trimesh box mesh of occupied body-frame cells."""
    import trimesh

    params = cap.params
    dexterity = cap.body_dexterity()
    keep = (cap.body_counts >= min_count) & (dexterity >= min_dexterity)
    n_voxels = int(keep.sum())
    if n_voxels == 0:
        return None, 0

    matrix = keep.transpose(1, 2, 0)
    import matplotlib

    colors = np.zeros((*matrix.shape, 4), dtype=np.uint8)
    rgba = matplotlib.colormaps["RdYlGn"](dexterity.transpose(1, 2, 0))
    colors[..., :3] = (rgba[..., :3] * 255).astype(np.uint8)
    colors[..., 3] = 255

    transform = np.eye(4)
    transform[0, 0] = transform[1, 1] = transform[2, 2] = params.cell
    transform[:3, 3] = (
        -params.r_xy + params.cell / 2,
        -params.r_xy + params.cell / 2,
        params.z_min + params.cell / 2,
    )
    grid = trimesh.voxel.VoxelGrid(matrix, transform=transform)  # type: ignore[no-untyped-call]
    return grid.as_boxes(colors=colors), n_voxels  # type: ignore[no-untyped-call]


def slice_image_yaw(
    cap: CapabilityMap, yaw_deg: float, px_per_cell: int = 6
) -> tuple[NDArray[np.uint8], float, float]:
    """Dexterity cross-section along a vertical plane through the map origin."""
    params = cap.params
    n_s = params.n_xy * px_per_cell
    n_z = params.n_z * px_per_cell
    s = np.linspace(-params.r_xy, params.r_xy, n_s)
    z = np.linspace(params.z_max, params.z_min, n_z)
    yaw = np.deg2rad(yaw_deg)
    xs = np.cos(yaw) * s
    ys = np.sin(yaw) * s
    positions = np.stack(
        [
            np.broadcast_to(xs, (n_z, n_s)).reshape(-1),
            np.broadcast_to(ys, (n_z, n_s)).reshape(-1),
            np.broadcast_to(z[:, None], (n_z, n_s)).reshape(-1),
        ],
        axis=1,
    )
    image = _dexterity_image(cap, positions, (n_z, n_s))
    return image, 2 * params.r_xy, params.z_max - params.z_min


def slice_image_height(
    cap: CapabilityMap, z: float, px_per_cell: int = 6
) -> tuple[NDArray[np.uint8], float, float]:
    """Dexterity cross-section on a horizontal plane."""
    params = cap.params
    n = params.n_xy * px_per_cell
    axis = np.linspace(-params.r_xy, params.r_xy, n)
    xx, yy = np.meshgrid(axis, -axis)
    positions = np.stack(
        [xx.reshape(-1), yy.reshape(-1), np.full(n * n, z)],
        axis=1,
    )
    image = _dexterity_image(cap, positions, (n, n))
    return image, 2 * params.r_xy, 2 * params.r_xy


def score_colors(scores: NDArray[np.float64], vmax: float | None = None) -> NDArray[np.uint8]:
    """Map scalar scores to red-to-green uint8 colors."""
    import matplotlib

    vmax = vmax or max(float(scores.max(initial=1.0)), 1.0)
    rgba = matplotlib.colormaps["RdYlGn"](np.clip(scores / vmax, 0, 1))
    return (rgba[:, :3] * 255).astype(np.uint8)


def vertical_slice_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion placing an image plane vertically at the given yaw."""
    from scipy.spatial.transform import Rotation

    matrix = np.array(
        [
            [np.cos(yaw), 0.0, np.sin(yaw)],
            [np.sin(yaw), 0.0, -np.cos(yaw)],
            [0.0, 1.0, 0.0],
        ]
    )
    x, y, z, w = Rotation.from_matrix(matrix).as_quat()
    return (w, x, y, z)


def _dexterity_image(
    cap: CapabilityMap, positions: NDArray[np.float64], shape: tuple[int, int]
) -> NDArray[np.uint8]:
    import matplotlib

    dexterity = cap.body_dexterity()
    iz, ix, iy, valid = cap.body_indices(positions)
    values = np.zeros(len(positions))
    occupied = np.zeros(len(positions), dtype=bool)
    values[valid] = dexterity[iz[valid], ix[valid], iy[valid]]
    occupied[valid] = cap.body_counts[iz[valid], ix[valid], iy[valid]] > 0

    rgba = matplotlib.colormaps["RdYlGn"](np.clip(values / max(values.max(), 1e-9), 0, 1))
    image = (rgba[:, :3] * 255).astype(np.uint8)
    image[~occupied] = _EMPTY_GRAY
    return np.asarray(image.reshape(*shape, 3))


class ReachabilityMapLayer:
    """Layer object for reachability volumes and slices in a Viser scene."""

    def __init__(self, scene: ViserManipulationScene, root: str = "/reachability") -> None:
        self._scene = scene
        self._root = root.rstrip("/") or "/reachability"
        self._handles: dict[str, Any] = {}

    def show_points(
        self,
        points: NDArray[np.float64],
        colors: NDArray[np.uint8],
        *,
        point_size: float,
    ) -> None:
        """Show a reachability point cloud."""
        self.clear_volume()
        if len(points) == 0 or point_size <= 0.0:
            return
        self._handles["points"] = self._scene.server.scene.add_point_cloud(
            f"{self._root}/points",
            points=points.astype(np.float32),
            colors=colors,
            point_size=point_size,
            point_shape="circle",
        )

    def show_voxel_mesh(self, mesh: Any | None) -> None:
        """Show a reachability voxel mesh."""
        self.clear_volume()
        if mesh is None:
            return
        self._handles["voxels"] = self._scene.server.scene.add_mesh_trimesh(
            f"{self._root}/core", mesh
        )

    def clear_volume(self) -> None:
        """Remove volume handles."""
        self._remove("points")
        self._remove("voxels")

    def show_vertical_slice(
        self,
        image: NDArray[np.uint8],
        *,
        width: float,
        height: float,
        center_z: float,
        wxyz: tuple[float, float, float, float],
    ) -> None:
        """Show a vertical reachability slice."""
        self._remove("vertical_slice")
        self._handles["vertical_slice"] = self._scene.server.scene.add_image(
            f"{self._root}/slices/vertical",
            np.ascontiguousarray(image[::-1]),
            render_width=width,
            render_height=height,
            position=(0.0, 0.0, center_z),
            wxyz=wxyz,
        )

    def show_horizontal_slice(
        self,
        image: NDArray[np.uint8],
        *,
        width: float,
        height: float,
        z: float,
    ) -> None:
        """Show a horizontal reachability slice."""
        self._remove("horizontal_slice")
        self._handles["horizontal_slice"] = self._scene.server.scene.add_image(
            f"{self._root}/slices/horizontal",
            np.ascontiguousarray(image[::-1]),
            render_width=width,
            render_height=height,
            position=(0.0, 0.0, z),
            wxyz=(1.0, 0.0, 0.0, 0.0),
        )

    def clear_vertical_slice(self) -> None:
        """Remove the vertical slice."""
        self._remove("vertical_slice")

    def clear_horizontal_slice(self) -> None:
        """Remove the horizontal slice."""
        self._remove("horizontal_slice")

    def clear_slices(self) -> None:
        """Remove all slice handles."""
        self.clear_vertical_slice()
        self.clear_horizontal_slice()

    def close(self) -> None:
        """Remove all reachability handles."""
        self.clear_volume()
        self.clear_slices()

    def _remove(self, key: str) -> None:
        handle = self._handles.pop(key, None)
        if handle is None:
            return
        remove = getattr(handle, "remove", None)
        if callable(remove):
            try:
                remove()
            except Exception:
                logger.warning("Could not remove reachability layer handle %s", key, exc_info=True)


__all__ = [
    "ReachabilityMapLayer",
    "body_point_cloud",
    "body_voxel_mesh",
    "score_colors",
    "slice_image_height",
    "slice_image_yaw",
    "vertical_slice_wxyz",
]
