# Copyright 2025 Dimensional Inc.
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

from dataclasses import dataclass
from typing import Optional, Tuple
import time
import numpy as np
import open3d as o3d
import reactivex.operators as ops
from reactivex import interval
from reactivex.observable import Observable

from dimos.core import In, Module, Out, rpc
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Map(Module):
    lidar: In[LidarMessage] = None
    global_map: Out[OccupancyGrid] = None
    local_costmap: Out[OccupancyGrid] = None
    pointcloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
    _latest_lidar: Optional[LidarMessage] = None

    def __init__(
        self,
        voxel_size: float = 0.05,
        cost_resolution: float = 0.05,
        global_publish_interval: Optional[float] = None,
        inflation_radius: float = 0.5,
        z_min: float = 0.1,
        z_max: float = 2.0,
        **kwargs,
    ):
        self.voxel_size = voxel_size
        self.cost_resolution = cost_resolution
        self.global_publish_interval = global_publish_interval
        self.inflation_radius = inflation_radius
        self.z_min = z_min
        self.z_max = z_max
        super().__init__(**kwargs)

    @rpc
    def start(self):
        self.lidar.subscribe(self.add_frame)

        if self.global_publish_interval is not None:
            interval(self.global_publish_interval).subscribe(lambda _: self.publish_global_map())

    def publish_global_map(self):
        """Convert accumulated pointcloud to OccupancyGrid and publish."""
        if self.pointcloud and len(self.pointcloud.points) > 0:
            # Create a PointCloud2 from the accumulated pointcloud
            # Use the frame_id from the latest lidar if available
            frame_id = self._latest_lidar.frame_id if self._latest_lidar else "world"
            pc2 = PointCloud2(pointcloud=self.pointcloud, frame_id=frame_id, ts=time.time())

            # Convert to OccupancyGrid
            occupancy_grid = OccupancyGrid.from_pointcloud(
                pc2,
                resolution=self.cost_resolution,
                min_height=self.z_min,
                max_height=self.z_max,
                inflate_radius=self.inflation_radius,
                frame_id=frame_id,
            )

            self.global_map.publish(occupancy_grid)

    @rpc
    def add_frame(self, frame: LidarMessage) -> "Map":
        """Voxelise *frame* and splice it into the running map."""
        # Store latest lidar for local costmap
        self._latest_lidar = frame

        # Publish local costmap from latest lidar frame
        if self.local_costmap:
            local_grid = OccupancyGrid.from_pointcloud(
                frame,  # LidarMessage is a PointCloud2
                resolution=self.cost_resolution,
                min_height=self.z_min,
                max_height=self.z_max,
                inflate_radius=self.inflation_radius,
                frame_id=frame.frame_id,
            )
            self.local_costmap.publish(local_grid)

        # Add to accumulated map
        new_pct = frame.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)
        self.pointcloud = splice_cylinder(self.pointcloud, new_pct, shrink=0.5)

        return self

    def consume(self, observable: Observable[LidarMessage]) -> Observable["Map"]:
        """Reactive operator that folds a stream of `LidarMessage` into the map."""
        return observable.pipe(ops.map(self.add_frame))

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self.pointcloud


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

    victims = np.nonzero(
        (planar_dists_map < radius)
        & (map_pts[:, axis] >= axis_min)
        & (map_pts[:, axis] <= axis_max)
    )[0]

    survivors = map_pcd.select_by_index(victims, invert=True)
    return survivors + patch_pcd
