#!/usr/bin/env python3
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

"""
ZED Module wrapper that filters pointclouds to match Unitree lidar specs.
"""

import time
import logging
import numpy as np
import open3d as o3d

from dimos.core import Module, Out, rpc
from dimos.hardware.zed_camera import ZEDModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__, level=logging.INFO)


class FilteredZEDModule(ZEDModule):
    """
    ZED Module that filters pointclouds to match Unitree lidar specifications.
    Inherits from ZEDModule and overrides pointcloud publishing.
    """

    def __init__(self, *args, **kwargs):
        # Filtering parameters - more aggressive for performance
        self.voxel_size = kwargs.pop("filter_voxel_size", 0.08)
        self.max_distance = kwargs.pop("filter_max_distance", 3.0)
        self.min_distance = kwargs.pop("filter_min_distance", 0.1)
        self.min_z = kwargs.pop("filter_min_z", -0.5)
        self.max_z = kwargs.pop("filter_max_z", 0.7)
        self.ground_threshold = kwargs.pop("filter_ground_threshold", -0.45)
        self.target_points = kwargs.pop("filter_target_points", 10000)
        self.filter_frame_count = 0

        super().__init__(*args, **kwargs)

        logger.info(f"FilteredZEDModule initialized with filters:")
        logger.info(f"  Voxel size: {self.voxel_size}m")
        logger.info(f"  Distance range: [{self.min_distance}, {self.max_distance}]m")
        logger.info(f"  Z range: [{self.min_z}, {self.max_z}]m")
        logger.info(f"  Target points: ~{self.target_points}")

    def _publish_pointcloud(self, header):
        """Override to filter pointcloud before publishing."""
        try:
            start_time = time.perf_counter()
            pcd = None
            frame_id = "camera_link"

            # If spatial mapping is enabled, use the full world map
            if self.enable_spatial_mapping and self.zed_camera.mapping_enabled:
                # Get the spatial map (full accumulated world map)
                pcd = self.zed_camera.get_spatial_map()

                if pcd is not None and len(pcd.points) > 0:
                    original_points = len(pcd.points)

                    # Downsample to target voxel size
                    if self.voxel_size > 0:
                        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

                    # The spatial map is already in world frame
                    frame_id = "world"

                    filtered_points = len(pcd.points)
                    elapsed_ms = (time.perf_counter() - start_time) * 1000

                    self.filter_frame_count += 1
                    if self.filter_frame_count % 30 == 0:  # Log every 30 frames
                        logger.info(
                            f"[SPATIAL MAP] {original_points:,} → {filtered_points:,} points "
                            f"(voxel downsample {self.voxel_size}m) "
                            f"in {elapsed_ms:.1f}ms"
                        )
                else:
                    # No spatial map available yet, skip
                    return
            else:
                # Fall back to regular pointcloud (current frame only)
                # Get the pointcloud data from ZED
                point_cloud_data = self.zed_camera.point_cloud.get_data()

                # Extract XYZ only - skip color processing entirely for performance
                points = point_cloud_data.reshape(-1, 4)[:, :3]  # Only XYZ, ignore RGBA

                # Only remove invalid points (NaN/Inf)
                valid = np.isfinite(points).all(axis=1)
                points = points[valid]

                if len(points) == 0:
                    return

                original_points = len(points)

                # Create Open3D pointcloud directly - NO FILTERING
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)

                # Only apply voxel downsampling to reduce point count
                if self.voxel_size > 0:
                    pcd = pcd.voxel_down_sample(self.voxel_size)

                # Log performance metrics
                filtered_points = len(pcd.points)
                elapsed_ms = (time.perf_counter() - start_time) * 1000

                self.filter_frame_count += 1
                if self.filter_frame_count % 30 == 0:  # Log every 30 frames
                    logger.info(
                        f"[CURRENT FRAME] {original_points:,} → {filtered_points:,} points "
                        f"(voxel downsample {self.voxel_size}m) "
                        f"in {elapsed_ms:.1f}ms"
                    )

                    if filtered_points > 0:
                        points_arr = np.asarray(pcd.points)
                        logger.info(
                            f"  Bounds: X:[{points_arr[:, 0].min():.2f},{points_arr[:, 0].max():.2f}] "
                            f"Y:[{points_arr[:, 1].min():.2f},{points_arr[:, 1].max():.2f}] "
                            f"Z:[{points_arr[:, 2].min():.2f},{points_arr[:, 2].max():.2f}]"
                        )

            # Create LidarMessage with filtered pointcloud
            if pcd is not None and len(pcd.points) > 0:
                lidar_msg = LidarMessage(
                    pointcloud=pcd,
                    origin=[0.0, 0.0, 0.0],
                    resolution=self.voxel_size,
                    ts=time.time(),
                    frame_id=frame_id,
                )

                # Publish filtered pointcloud
                self.pointcloud_msg.publish(lidar_msg)

        except Exception as e:
            logger.error(f"Error in optimized pointcloud publishing: {e}")


def test_filtered_module():
    """Test the filtered ZED module."""
    import asyncio
    from dimos import core
    from dimos.protocol import pubsub

    async def run_test():
        pubsub.lcm.autoconf()

        # Start Dimos
        dimos = core.start(2)

        # Deploy filtered ZED module
        zed = dimos.deploy(
            FilteredZEDModule,
            camera_id=0,
            resolution="HD720",
            depth_mode="NEURAL",
            fps=15,
            enable_tracking=True,
            publish_rate=10.0,
            frame_id="camera_link",
            # Filtering parameters
            filter_voxel_size=0.05,
            filter_max_distance=4.0,
            filter_target_points=22600,
        )

        # Configure transports
        from dimos.msgs.sensor_msgs import Image
        from dimos.msgs.geometry_msgs import PoseStamped

        zed.pointcloud_msg.transport = core.LCMTransport("/zed/filtered_pointcloud", LidarMessage)
        zed.color_image.transport = core.LCMTransport("/zed/color_image", Image)
        zed.pose.transport = core.LCMTransport("/zed/pose", PoseStamped)

        # Start module
        zed.start()

        logger.info("FilteredZEDModule test running for 10 seconds...")
        await asyncio.sleep(10)

        # Stop
        zed.stop()
        core.stop(dimos)
        logger.info("Test complete")

    asyncio.run(run_test())


if __name__ == "__main__":
    test_filtered_module()
