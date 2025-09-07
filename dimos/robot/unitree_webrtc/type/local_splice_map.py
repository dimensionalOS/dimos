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
import numpy as np
import open3d as o3d
from reactivex import interval

from dimos.core import In, Module, Out, rpc
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class LocalSpliceMap(Module):
    """
    Map module that extracts a local cylinder (3m radius) from incoming lidar data
    and splices it into a persistent global map.

    Unlike PassthroughMap which just passes through the ZED's world map,
    this builds its own global map by accumulating local scans at the robot's position.
    """

    lidar: In[LidarMessage] = None
    odom: In[PoseStamped] = None
    global_map: Out[LidarMessage] = None
    global_costmap: Out[OccupancyGrid] = None
    local_costmap: Out[OccupancyGrid] = None

    def __init__(
        self,
        cylinder_radius: float = 3.0,
        voxel_size: float = 0.05,
        cost_resolution: float = 0.05,
        global_publish_interval: float = 2.5,
        min_height: float = 0.2,
        max_height: float = 1.5,
        frame_id: str = "world",
        **kwargs,
    ):
        self.cylinder_radius = cylinder_radius
        self.voxel_size = voxel_size
        self.cost_resolution = cost_resolution
        self.global_publish_interval = global_publish_interval
        self.min_height = min_height
        self.max_height = max_height
        self.frame_id = frame_id

        # Internal global map accumulator
        self.global_pointcloud = o3d.geometry.PointCloud()

        # Current robot position
        self.current_odom: PoseStamped = None

        # Latest local scan for local costmap
        self.latest_local_scan: LidarMessage = None

        super().__init__(**kwargs)

    @rpc
    def start(self):
        logger.info(f"LocalSpliceMap.start() - Cylinder radius: {self.cylinder_radius}m")
        logger.info(f"Publishing global map every {self.global_publish_interval}s")

        # Subscribe to inputs
        self.lidar.subscribe(self.process_lidar_message)
        self.odom.subscribe(self.update_odom)

        # Publish global map periodically
        def publish(_):
            if len(self.global_pointcloud.points) > 0:
                # Convert to LidarMessage
                global_map_msg = LidarMessage(
                    pointcloud=self.global_pointcloud,
                    origin=[0.0, 0.0, 0.0],
                    resolution=self.voxel_size,
                    frame_id=self.frame_id,
                    ts=time.time(),
                )
                self.global_map.publish(global_map_msg)

                # Generate and publish global costmap
                occupancygrid = OccupancyGrid.from_pointcloud_adaptive(
                    global_map_msg,
                    resolution=self.cost_resolution,
                    neighborhood_radius=0.15,
                    height_diff_threshold=0.3,
                    min_neighbors=3,
                    mark_free_radius=0.4,
                )
                self.global_costmap.publish(occupancygrid)

                logger.debug(
                    f"Published global map with {len(self.global_pointcloud.points)} points"
                )

        if self.global_publish_interval is not None:
            interval(self.global_publish_interval).subscribe(publish)

    @rpc
    def update_odom(self, msg: PoseStamped):
        """Update current robot position from odometry."""
        self.current_odom = msg
        logger.debug(f"Updated robot position: ({msg.position.x:.2f}, {msg.position.y:.2f})")

    @rpc
    def process_lidar_message(self, frame: LidarMessage):
        """Process incoming lidar messages."""
        if self.current_odom is None:
            logger.warning("No odometry available yet, skipping lidar frame")
            return

        # Store for local costmap
        self.latest_local_scan = frame

        # Publish local costmap
        local_costmap = OccupancyGrid.from_pointcloud(
            frame,
            resolution=self.cost_resolution,
            min_height=self.min_height,
            max_height=self.max_height,
        ).gradient(max_distance=0.25)
        self.local_costmap.publish(local_costmap)

        # Extract local cylinder from lidar data at robot position
        robot_pos = np.array(
            [
                self.current_odom.position.x,
                self.current_odom.position.y,
                self.current_odom.position.z,
            ]
        )

        # Convert lidar data to numpy array
        points = np.asarray(frame.pointcloud.points)

        if len(points) == 0:
            logger.debug("Empty lidar frame, skipping")
            return

        # Transform points to world frame if needed
        if frame.frame_id != self.frame_id:
            # For now, assume lidar is in camera_link frame and transform based on robot pose
            # In reality, you'd use proper TF transforms here
            # Simple translation for now (assuming no rotation for simplicity)
            points_world = points + robot_pos
        else:
            points_world = points

        # Extract cylinder around robot position
        # Calculate distance from each point to robot position in XY plane
        xy_distances = np.linalg.norm(points_world[:, :2] - robot_pos[:2], axis=1)

        # Select points within cylinder radius
        cylinder_mask = xy_distances <= self.cylinder_radius
        cylinder_points = points_world[cylinder_mask]

        if len(cylinder_points) == 0:
            logger.debug("No points within cylinder radius")
            return

        # Create pointcloud from cylinder points
        cylinder_pcd = o3d.geometry.PointCloud()
        cylinder_pcd.points = o3d.utility.Vector3dVector(cylinder_points)

        # Voxelize the cylinder
        cylinder_voxelized = cylinder_pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Splice into global map - remove old points in cylinder and add new ones
        self.splice_cylinder_into_map(cylinder_voxelized, robot_pos)

        logger.debug(
            f"Added {len(cylinder_points)} points from cylinder, "
            f"global map now has {len(self.global_pointcloud.points)} points"
        )

    def splice_cylinder_into_map(self, new_pcd: o3d.geometry.PointCloud, center: np.ndarray):
        """
        Splice a new cylinder of points into the global map.
        Removes old points within the cylinder radius and adds new ones.
        """
        if len(self.global_pointcloud.points) == 0:
            # First frame, just set it
            self.global_pointcloud = new_pcd
            return

        # Remove points from global map that are within cylinder radius of center
        global_points = np.asarray(self.global_pointcloud.points)

        # Calculate XY distances from center
        xy_distances = np.linalg.norm(global_points[:, :2] - center[:2], axis=1)

        # Keep points outside the cylinder
        survivors_mask = (
            xy_distances > self.cylinder_radius * 0.95
        )  # Slightly smaller to avoid gaps
        survivors_indices = np.where(survivors_mask)[0]

        # Select surviving points
        survivors = self.global_pointcloud.select_by_index(survivors_indices)

        # Combine survivors with new points
        self.global_pointcloud = survivors + new_pcd
