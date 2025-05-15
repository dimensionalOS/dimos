import open3d as o3d
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.costmap import Costmap

from reactivex.observable import Observable
import reactivex.operators as ops


@dataclass
class Map:
    pointcloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
    voxel_size: float = 0.05
    cost_resolution: float = 0.05

    def add_frame(self, frame: LidarMessage) -> "Map":
        """Voxelise *frame* and splice it into the running map."""
        new_pct = frame.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)
        self.pointcloud = splice_cylinder(self.pointcloud, new_pct, shrink=0.5)
        return self

    def consume(self, observable: Observable[LidarMessage]) -> Observable["Map"]:
        """Reactive operator that folds a stream of `LidarMessage` into the map."""
        return observable.pipe(ops.map(self.add_frame))

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self.pointcloud

    @property
    def costmap(self) -> Costmap:
        """Return a fully inflated cost-map in a `Costmap` wrapper."""
        inflate_radius_m = 0.5 * self.voxel_size if self.voxel_size > self.cost_resolution else 0.0
        grid, origin_xy = pointcloud_to_costmap(
            self.pointcloud,
            resolution=self.cost_resolution,
            inflate_radius_m=inflate_radius_m,
        )
        return Costmap(grid=grid, origin=[*origin_xy, 0.0], resolution=self.cost_resolution)


def splice_sphere(
    map_pcd: o3d.geometry.PointCloud,
    patch_pcd: o3d.geometry.PointCloud,
    shrink: float = 0.95,
) -> o3d.geometry.PointCloud:
    center = patch_pcd.get_center()
    radius = np.linalg.norm(np.asarray(patch_pcd.points) - center, axis=1).max() * shrink
    dists = np.linalg.norm(np.asarray(map_pcd.points) - center, axis=1)
    victims = np.nonzero(dists < radius)[0]
    survivors = map_pcd.select_by_index(victims, invert=True)
    return survivors + patch_pcd


def splice_cylinder(
    map_pcd: o3d.geometry.PointCloud,
    patch_pcd: o3d.geometry.PointCloud,
    axis: int = 2,
    shrink: float = 0.95,
) -> o3d.geometry.PointCloud:
    center = patch_pcd.get_center()
    patch_pts = np.asarray(patch_pcd.points)

    # Axes perpendicular to cylinder
    axes = [0, 1, 2]
    axes.remove(axis)

    planar_dists = np.linalg.norm(patch_pts[:, axes] - center[axes], axis=1)
    radius = planar_dists.max() * shrink

    axis_min = (patch_pts[:, axis].min() - center[axis]) * shrink + center[axis]
    axis_max = (patch_pts[:, axis].max() - center[axis]) * shrink + center[axis]

    map_pts = np.asarray(map_pcd.points)
    planar_dists_map = np.linalg.norm(map_pts[:, axes] - center[axes], axis=1)

    victims = np.nonzero((planar_dists_map < radius) & (map_pts[:, axis] >= axis_min) & (map_pts[:, axis] <= axis_max))[
        0
    ]

    survivors = map_pcd.select_by_index(victims, invert=True)
    return survivors + patch_pcd


def _inflate_lethal(costmap: np.ndarray, radius: int, lethal_val: int = 100) -> np.ndarray:
    """Return *costmap* with lethal cells dilated by *radius* grid steps (circular)."""
    if radius <= 0 or not np.any(costmap == lethal_val):
        return costmap

    mask = costmap == lethal_val
    dilated = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius or (dx == 0 and dy == 0):
                continue
            dilated |= np.roll(mask, shift=(dy, dx), axis=(0, 1))

    out = costmap.copy()
    out[dilated] = lethal_val
    return out


def pointcloud_to_costmap(
    pcd: o3d.geometry.PointCloud,
    *,
    resolution: float = 0.05,
    ground_z: float = 0.0,
    obs_min_height: float = 0.15,
    max_height: Optional[float] = 0.5,
    inflate_radius_m: Optional[float] = None,
    default_unknown: int = -1,
    cost_free: int = 0,
    cost_lethal: int = 100,
) -> Tuple[np.ndarray, np.ndarray]:
    """Rasterise *pcd* into a 2-D int8 cost-map with optional obstacle inflation.

    Grid origin is **aligned** to the `resolution` lattice so that when
    `resolution == voxel_size` every voxel centroid lands squarely inside a cell
    (no alternating blank lines).
    """

    pts = np.asarray(pcd.points, dtype=np.float32)
    if pts.size == 0:
        return np.full((1, 1), default_unknown, np.int8), np.zeros(2, np.float32)

    # 0. Ceiling filter --------------------------------------------------------
    if max_height is not None:
        pts = pts[pts[:, 2] <= max_height]
        if pts.size == 0:
            return np.full((1, 1), default_unknown, np.int8), np.zeros(2, np.float32)

    # 1. Bounding box & aligned origin ---------------------------------------
    xy_min = pts[:, :2].min(axis=0)
    xy_max = pts[:, :2].max(axis=0)

    # Align origin to the resolution grid (anchor = 0,0)
    origin = np.floor(xy_min / resolution) * resolution

    # Grid dimensions (inclusive) -------------------------------------------
    Nx, Ny = (np.ceil((xy_max - origin) / resolution).astype(int) + 1).tolist()

    # 2. Bin points ------------------------------------------------------------
    idx_xy = np.floor((pts[:, :2] - origin) / resolution).astype(np.int32)
    np.clip(idx_xy[:, 0], 0, Nx - 1, out=idx_xy[:, 0])
    np.clip(idx_xy[:, 1], 0, Ny - 1, out=idx_xy[:, 1])

    lin = idx_xy[:, 1] * Nx + idx_xy[:, 0]
    z_max = np.full(Nx * Ny, -np.inf, np.float32)
    np.maximum.at(z_max, lin, pts[:, 2])
    z_max = z_max.reshape(Ny, Nx)

    # 3. Cost rules -----------------------------------------------------------
    costmap = np.full_like(z_max, default_unknown, np.int8)
    known = z_max != -np.inf
    costmap[known] = cost_free

    lethal = z_max >= (ground_z + obs_min_height)
    costmap[lethal] = cost_lethal

    # 4. Optional inflation ----------------------------------------------------
    if inflate_radius_m and inflate_radius_m > 0:
        cells = int(np.ceil(inflate_radius_m / resolution))
        costmap = _inflate_lethal(costmap, cells, lethal_val=cost_lethal)

    return costmap, origin.astype(np.float32)
