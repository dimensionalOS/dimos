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

from reactivex import interval

from dimos.core import In, Module, Out, rpc
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class PassthroughMap(Module):
    """
    Map module that passes through the world map from ZED spatial mapping.

    Unlike the regular Map module, this doesn't accumulate pointclouds because
    ZED's spatial mapping already provides the full accumulated world map.
    It simply passes through the world map and generates costmaps from it.
    """

    lidar: In[LidarMessage] = None
    global_map: Out[LidarMessage] = None
    global_costmap: Out[OccupancyGrid] = None
    local_costmap: Out[OccupancyGrid] = None

    def __init__(
        self,
        cost_resolution: float = 0.05,
        global_publish_interval: float = 2.5,
        min_height: float = 0.2,
        max_height: float = 1.5,
        frame_id: str = "world",
        **kwargs,
    ):
        self.cost_resolution = cost_resolution
        self.global_publish_interval = global_publish_interval
        self.min_height = min_height
        self.max_height = max_height
        self.frame_id = frame_id

        # Store the latest world map from ZED
        self.latest_world_map: LidarMessage = None
        self.latest_local_scan: LidarMessage = None

        super().__init__(**kwargs)

    @rpc
    def start(self):
        logger.info(f"PassthroughMap.start() - Publishing every {self.global_publish_interval}s")
        logger.info("This map module passes through ZED's spatial map without accumulation")

        # Subscribe to lidar messages
        self.lidar.subscribe(self.process_lidar_message)

        # Publish global map periodically
        def publish(_):
            if self.latest_world_map is not None:
                self.global_map.publish(self.latest_world_map)

                # Generate and publish global costmap
                occupancygrid = OccupancyGrid.from_pointcloud_adaptive(
                    self.latest_world_map,
                    resolution=self.cost_resolution,
                    neighborhood_radius=0.15,
                    height_diff_threshold=0.3,
                    min_neighbors=3,
                    mark_free_radius=0.4,
                )
                self.global_costmap.publish(occupancygrid)

        if self.global_publish_interval is not None:
            logger.info(
                f"Setting up interval publishing every {self.global_publish_interval} seconds"
            )
            interval(self.global_publish_interval).subscribe(publish)

    @rpc
    def process_lidar_message(self, frame: LidarMessage):
        """Process incoming lidar messages."""
        # Check if this is a world frame message (from ZED spatial mapping)
        if frame.frame_id == "world":
            # This is the full world map from ZED spatial mapping
            self.latest_world_map = frame
            logger.debug(f"Received world map with {len(frame.pointcloud.points)} points")
        else:
            # This is a local scan (camera_link frame)
            self.latest_local_scan = frame

            # Publish local costmap from the local scan
            local_costmap = OccupancyGrid.from_pointcloud(
                frame,
                resolution=self.cost_resolution,
                min_height=self.min_height,
                max_height=self.max_height,
            ).gradient(max_distance=0.25)
            self.local_costmap.publish(local_costmap)
