import open3d as o3d
import numpy as np
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.costmap import Costmap
from dataclasses import dataclass
from reactivex.observable import Observable
import reactivex.operators as ops
from typing import Tuple


@dataclass
class Map:
    pointcloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
    voxel_size: float = 0.25  # metres

    def add_frame(self, frame: LidarMessage) -> "Map":
        """Ingest a single LIDAR scan, voxelise it and splice it into the map."""
        new_pct = frame.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)
        self.pointcloud = splice_cylinder(self.pointcloud, new_pct, shrink=0.6)
        return self

    def consume(self, observable: Observable[LidarMessage]) -> Observable["Map"]:
        """Reactive operator that folds a stream of `LidarMessage` into a running map."""
        return observable.pipe(ops.map(self.add_frame))

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self.pointcloud

    @property
    def costmap(self) -> Costmap:
        """Return a Costmap object whose origin is the SW corner of the grid (world frame)."""
        grid, origin_xy = pointcloud_to_costmap(self.pointcloud, resolution=self.voxel_size)
        # Costmap.origin is expected to be an XYZ vector → pad Z with zero
        return Costmap(grid=grid, origin=[*origin_xy, 0.0], resolution=self.voxel_size)


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
    axis: int = 2,  # default Z‑axis
    shrink: float = 0.95,
) -> o3d.geometry.PointCloud:
    center = patch_pcd.get_center()
    patch_points = np.asarray(patch_pcd.points)

    # Axes perpendicular to the cylinder axis
    axes = list(range(3))
    axes.remove(axis)

    planar_dists = np.linalg.norm(patch_points[:, axes] - center[axes], axis=1)
    radius = planar_dists.max() * shrink

    axis_min = (patch_points[:, axis].min() - center[axis]) * shrink + center[axis]
    axis_max = (patch_points[:, axis].max() - center[axis]) * shrink + center[axis]

    map_points = np.asarray(map_pcd.points)
    planar_dists_map = np.linalg.norm(map_points[:, axes] - center[axes], axis=1)

    inside_radius = planar_dists_map < radius
    inside_height = (map_points[:, axis] >= axis_min) & (map_points[:, axis] <= axis_max)
    victims = np.nonzero(inside_radius & inside_height)[0]

    survivors = map_pcd.select_by_index(victims, invert=True)
    return survivors + patch_pcd


def pointcloud_to_costmap(
    pcd: o3d.geometry.PointCloud,
    *,
    resolution: float = 0.05,  # metres / cell
    ground_z: float = 0.0,
    obs_min_height: float = 0.15,
    max_height: float | None = 0.5,
    default_unknown: int = -1,
    cost_free: int = 0,
    cost_lethal: int = 100,
) -> Tuple[np.ndarray, np.ndarray]:
    """Rasterise a 3-D point-cloud into a 2-D cost-map.

    Returns
    -------
    costmap : np.ndarray[int8]
        Matrix indexed as costmap[y, x].
    origin_xy : np.ndarray[float32, shape=(2,)]
        SW corner of grid in the *world* frame.  Use
        `world_xy = origin_xy + resolution * (ixy + 0.5)` to convert indices.
    """

    pts = np.asarray(pcd.points, dtype=np.float32)
    if pts.size == 0:
        return np.full((1, 1), default_unknown, dtype=np.int8), np.zeros(2, np.float32)

    if max_height is not None:
        pts = pts[pts[:, 2] <= max_height]
        if pts.size == 0:
            return (
                np.full((1, 1), default_unknown, dtype=np.int8),
                np.zeros(2, np.float32),
            )

    xy_min = pts[:, :2].min(axis=0)
    xy_max = pts[:, :2].max(axis=0)

    # dims = (Nx, Ny)
    dims = np.ceil((xy_max - xy_min) / resolution).astype(int) + 1
    Nx, Ny = dims
    origin = xy_min.astype(np.float32)

    idx_xy = np.floor((pts[:, :2] - origin) / resolution).astype(np.int32)
    np.clip(idx_xy[:, 0], 0, Nx - 1, out=idx_xy[:, 0])
    np.clip(idx_xy[:, 1], 0, Ny - 1, out=idx_xy[:, 1])

    lin = idx_xy[:, 1] * Nx + idx_xy[:, 0]
    z_max = np.full(Nx * Ny, -np.inf, dtype=np.float32)
    np.maximum.at(z_max, lin, pts[:, 2])
    z_max = z_max.reshape(Ny, Nx)

    costmap = np.full_like(z_max, default_unknown, dtype=np.int8)
    known = z_max != -np.inf
    costmap[known] = cost_free

    lethal = z_max >= (ground_z + obs_min_height)
    costmap[lethal] = cost_lethal

    return costmap, origin
