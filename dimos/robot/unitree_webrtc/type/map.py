import open3d as o3d
import numpy as np
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.costmap import Costmap
from dataclasses import dataclass
from reactivex.observable import Observable
import reactivex.operators as ops
from typing import Callable
from dimos.types.vector import Vector


@dataclass
class Map:
    pointcloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
    voxel_size: float = 0.25
    pos: Callable[[], Vector] = None

    def add_frame(self, frame: LidarMessage) -> "Map":
        new_pct = frame.pointcloud = frame.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)
        self.pointcloud = splice_cylinder(self.pointcloud, new_pct, shrink=0.5)
        return self

    def consume(self, observable: Observable[LidarMessage]) -> Observable["Map"]:
        return observable.pipe(ops.map(self.add_frame))

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self.pointcloud

    @property
    def costmap(self) -> Costmap:
        grid = pointcloud_to_costmap(self.pointcloud, resolution=self.voxel_size)
        return Costmap(
            grid=grid,
            origin=self.pos(),
            origin_theta=0.0,
            resolution=self.voxel_size,
        )


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
    axis: int = 2,  # Default axis is Z (2)
    shrink: float = 0.95,
) -> o3d.geometry.PointCloud:
    center = patch_pcd.get_center()
    patch_points = np.asarray(patch_pcd.points)

    # Calculate distances in the plane perpendicular to the specified axis
    axes = list(range(3))
    axes.remove(axis)

    # Calculate radius as the maximum distance in the perpendicular plane
    planar_dists = np.linalg.norm(patch_points[:, axes] - center[axes], axis=1)
    radius = planar_dists.max() * shrink

    # Calculate min and max along the cylinder axis
    axis_min = (patch_points[:, axis].min() - center[axis]) * shrink + center[axis]
    axis_max = (patch_points[:, axis].max() - center[axis]) * shrink + center[axis]

    # Check which points in the map are inside the cylinder
    map_points = np.asarray(map_pcd.points)
    planar_dists_map = np.linalg.norm(map_points[:, axes] - center[axes], axis=1)

    # Points are inside the cylinder if:
    # 1. They are within the radius in the perpendicular plane
    # 2. They are between the min and max along the cylinder axis
    inside_radius = planar_dists_map < radius
    inside_height = (map_points[:, axis] >= axis_min) & (map_points[:, axis] <= axis_max)
    victims = np.nonzero(inside_radius & inside_height)[0]

    # Select points outside the cylinder
    survivors = map_pcd.select_by_index(victims, invert=True)

    return survivors + patch_pcd


def pointcloud_to_costmap(
    pcd: o3d.geometry.PointCloud,
    *,
    resolution: float = 0.05,  # metres / cell
    ground_z: float = 0.0,  # reference "floor" height
    obs_min_height: float = 0.15,  # ≥ this above ground ⇒ cost 100
    max_height: float | None = 0.5,  # ignore points with z > max_height
    default_unknown: int = -1,
    cost_free: int = 0,
    cost_lethal: int = 100,
) -> np.ndarray:
    """
    3-D point-cloud → 2-D int8 costmap.

    • If max_height is given, points with z > max_height are dropped
      (useful for ignoring desk/table tops when the robot can pass under).
    • Returns (costmap[y,x], origin_xy, resolution).
    """
    pts = np.asarray(pcd.points, dtype=np.float32)
    if pts.size == 0:
        raise ValueError("empty point-cloud")

    # ------------------------------------------------------------------
    # 0. Optional ceiling filter
    # ------------------------------------------------------------------
    if max_height is not None:
        pts = pts[pts[:, 2] <= max_height]
        if pts.size == 0:  # all points removed → unknown grid
            origin = np.array([0.0, 0.0], dtype=np.float32)
            return np.full((1, 1), default_unknown, dtype=np.int8)

    # ------------------------------------------------------------------
    # 1. Grid extents in X-Y
    # ------------------------------------------------------------------
    xy_min = pts[:, :2].min(axis=0)
    xy_max = pts[:, :2].max(axis=0)
    dims = np.ceil((xy_max - xy_min) / resolution).astype(int) + 1  # Nx, Ny
    Nx, Ny = dims
    origin = xy_min

    # ------------------------------------------------------------------
    # 2. Bin points → per-cell max-Z
    # ------------------------------------------------------------------
    idx_xy = np.floor((pts[:, :2] - origin) / resolution).astype(np.int32)
    np.clip(idx_xy[:, 0], 0, Nx - 1, out=idx_xy[:, 0])
    np.clip(idx_xy[:, 1], 0, Ny - 1, out=idx_xy[:, 1])

    lin = idx_xy[:, 1] * Nx + idx_xy[:, 0]
    z_max = np.full(Nx * Ny, -np.inf, dtype=np.float32)
    np.maximum.at(z_max, lin, pts[:, 2])
    z_max = z_max.reshape(Ny, Nx)

    # ------------------------------------------------------------------
    # 3. Cost rules
    # ------------------------------------------------------------------
    costmap = np.full_like(z_max, default_unknown, dtype=np.int8)

    known = z_max != -np.inf
    costmap[known] = cost_free

    lethal = z_max >= ground_z + obs_min_height
    costmap[lethal] = cost_lethal

    return costmap
