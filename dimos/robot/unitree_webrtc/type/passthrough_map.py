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
from dimos.msgs.geometry_msgs import PoseStamped
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
    odom: In[PoseStamped] = None  # Keep for compatibility but not used
    global_map: Out[LidarMessage] = None
    global_costmap: Out[OccupancyGrid] = None
    local_costmap: Out[OccupancyGrid] = None

    def __init__(
        self,
        voxel_size: float = 0.1,
        global_publish_interval: float = 2.5,
        min_height: float = 0.15,
        max_height: float = 1.5,
        **kwargs,
    ):
        self.voxel_size = voxel_size
        self.global_publish_interval = global_publish_interval
        self.min_height = min_height
        self.max_height = max_height
        self.latest_world_map: LidarMessage = None

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
                logger.info("Publishing global_map from ZED spatial mapping")
                self.global_map.publish(self.latest_world_map)

                # Generate and publish global costmap
                occupancygrid = OccupancyGrid.from_pointcloud(
                    self.latest_world_map,
                    resolution=self.voxel_size,
                    min_height=self.min_height,
                    max_height=self.max_height,
                )
                self.global_costmap.publish(occupancygrid)

        if self.global_publish_interval is not None:
            logger.info(
                f"Setting up interval publishing every {self.global_publish_interval} seconds"
            )
            interval(self.global_publish_interval).subscribe(publish)

    @rpc
    def process_lidar_message(self, frame: LidarMessage):
        self.latest_world_map = frame
