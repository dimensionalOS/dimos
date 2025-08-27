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

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import open3d as o3d
import reactivex.operators as ops
from reactivex import interval
from reactivex.observable import Observable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.msgs.geometry_msgs import PoseStamped
from collections import deque



class Map(Module):
    lidar: In[LidarMessage] = None
    odom: In[PoseStamped] = None  # New input for odometry messages
    global_map: Out[LidarMessage] = None
    global_costmap: Out[OccupancyGrid] = None
    local_costmap: Out[OccupancyGrid] = None

    pointcloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()

    def __init__(
        self,
        voxel_size: float = 0.05,
        cost_resolution: float = 0.05,
        global_publish_interval: Optional[float] = None,
        min_height: float = 0.15,
        max_height: float = 0.6,
        frame_id: str = "world",
        max_map_points: int = 100000,  # Limit accumulated map size
        max_odom_history: int = 1000,  # Maximum odometry messages to store
        time_tolerance: float = 0.1,  # Maximum time difference for synchronization (seconds)

        **kwargs,
    ):
        self.voxel_size = voxel_size
        self.cost_resolution = cost_resolution
        self.global_publish_interval = global_publish_interval
        self.min_height = min_height
        self.max_height = max_height
        self.frame_id = frame_id  # Target frame for map
        self.max_map_points = max_map_points
        self.max_odom_history = max_odom_history
        self.time_tolerance = time_tolerance
        self.odom_history = deque(maxlen=max_odom_history)  # Buffer for odometry messages

        super().__init__(**kwargs)

    @rpc
    def start(self):
        import logging

        logger = logging.getLogger(__name__)
        logger.info(f"Map.start() called - global_publish_interval={self.global_publish_interval}")
        # Subscribe to odometry messages to build history
        if self.odom:
            self.odom.subscribe(self.store_odom)
            logger.info("Subscribed to odometry messages for timestamp synchronization")


        self.lidar.subscribe(self.add_frame)

        def publish(_):
            logger.info(f"Publishing global_map with {len(self.pointcloud.points)} points")
            self.global_map.publish(self.to_lidar_message())

            # temporary, not sure if it belogs in mapper
            # used only for visualizations, not for any algo
            occupancygrid = OccupancyGrid.from_pointcloud(
                self.to_lidar_message(),
                resolution=self.cost_resolution,
                min_height=self.min_height,
                max_height=self.max_height,
            )

            self.global_costmap.publish(occupancygrid)

        if self.global_publish_interval is not None:
            logger.info(
                f"Setting up interval publishing every {self.global_publish_interval} seconds"
            )
            interval(self.global_publish_interval).subscribe(publish)
        else:
            logger.warning("No global_publish_interval set - global_map will NOT be published!")

    @rpc
    def store_odom(self, odom: PoseStamped):
        """Store odometry message in history buffer."""
        self.odom_history.append(odom)
    
    def find_closest_odom(self, target_ts: float) -> Optional[PoseStamped]:
        """Find the odometry message closest in time to the target timestamp."""
        if not self.odom_history:
            return None
        
        # Find the closest odometry message
        closest_odom = None
        min_time_diff = float('inf')
        
        for odom in self.odom_history:
            time_diff = abs(odom.ts - target_ts)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_odom = odom
        
        # Check if the closest is within tolerance
        if min_time_diff <= self.time_tolerance:
            return closest_odom
        else:
            return None


    def to_PointCloud2(self) -> PointCloud2:
        return PointCloud2(
            pointcloud=self.pointcloud,
            ts=time.time(),
        )

    def to_lidar_message(self) -> LidarMessage:
        return LidarMessage(
            pointcloud=self.pointcloud,
            origin=[0.0, 0.0, 0.0],
            resolution=self.voxel_size,
            ts=time.time(),
        )

    @rpc
    def add_frame(self, frame: LidarMessage) -> "Map":
        """Transform to world frame if needed, then add to map."""
        import logging

        logger = logging.getLogger(__name__)
        # Transform to world frame if needed
        if frame.frame_id != self.frame_id:
            # Try to find synchronized odometry transform
            closest_odom = self.find_closest_odom(frame.ts)
            print("human readable frame ts:", time.ctime(frame.ts), "closest odom ts:", time.ctime(closest_odom.ts) if closest_odom else 'None')
            print('diff', abs(closest_odom.ts - frame.ts) if closest_odom else 'N/A')
            print('------')
            
            if closest_odom:
                # Use the synchronized odometry transform
                try:
                    from dimos.msgs.geometry_msgs import Transform
                    # Create transform from the synchronized pose
                    tf = Transform.from_pose(closest_odom.frame_id, closest_odom)
                    frame = frame.transform(tf)
                    logger.debug(
                        f"Using synchronized transform (time diff: {abs(closest_odom.ts - frame.ts):.3f}s)"
                    )
                except Exception as e:
                    logger.warning(f"Failed to apply synchronized transform: {e}")
                    return
            else:
                # No close odometry message found, print warning
                logger.warning(
                    f"No odometry message found within {self.time_tolerance}s of lidar timestamp {frame.ts}. "
                    f"Skipping this frame for global map accumulation."
                )
                # Still publish local costmap even without global map update
                local_costmap = OccupancyGrid.from_pointcloud(
                    frame,
                    resolution=self.cost_resolution,
                    min_height=self.min_height,
                    max_height=self.max_height,
                ).gradient(max_distance=0.25)
                self.local_costmap.publish(local_costmap)
                return

        # Combine old map with new raw pointcloud (no pre-downsampling)
        # This preserves all detail from the new scan
        self.pointcloud = self.pointcloud + frame.pointcloud

        # Single voxel downsample of the combined pointcloud
        # This merges ALL points (old and new) within each voxel into one representative point
        self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)

        # Additional downsampling if still too large (use smaller voxel size)
        if len(self.pointcloud.points) > self.max_map_points:
            # Use slightly larger voxel size to reduce point count
            aggressive_voxel_size = self.voxel_size * 1.2
            self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=aggressive_voxel_size)
            logger.info(
                f"Map downsampled to {len(self.pointcloud.points)} points with voxel size {aggressive_voxel_size:.3f}m"
            )

        # Local costmap unchanged
        local_costmap = OccupancyGrid.from_pointcloud(
            frame,
            resolution=self.cost_resolution,
            min_height=self.min_height,
            max_height=self.max_height,
        ).gradient(max_distance=0.25)
        self.local_costmap.publish(local_costmap)

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
