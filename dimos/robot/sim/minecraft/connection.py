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

import functools
import pickle
from typing import Optional

import minedojo
import numpy as np
import open3d as o3d
import reactivex as rx
import reactivex.operators as ops

from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol import pubsub
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.utils.data import get_data
from dimos.utils.reactive import backpressure, callback_to_observable
from dimos.utils.testing import TimedSensorReplay


class Minecraft(Module):
    movecmd: In[Vector3] = None
    odom: Out[Vector3] = None
    lidar: Out[PointCloud2] = None
    video: Out[Image] = None
    ip: str

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Minecraft block size in meters (1 block = 0.5m)
        self.block_size = 0.5
        # Point cloud resolution in meters
        self.resolution = 0.1
        # Points per block dimension
        self.points_per_block = int(self.block_size / self.resolution)
        # Lidar frequency in Hz (10Hz is typical for robot lidars)
        self.lidar_frequency = 10.0

        self.tf = core.TF()

        # MineDojo environment setup (disabled while using pickle)
        self.env = None
        # self.env = minedojo.make(
        #     task_id="creative:1",
        #     image_size=(800, 1280),
        #     world_seed="dimensional",
        #     use_voxel=True,
        #     voxel_size=dict(xmin=-5, ymin=-2, zmin=-5, xmax=5, ymax=2, zmax=5),
        # )

        # Load observation from pickle for testing
        try:
            import os

            pickle_path = os.path.join(os.path.dirname(__file__), "observation.pkl")
            with open(pickle_path, "rb") as f:
                self.obs = pickle.load(f)
                print(f"Loaded observation from {pickle_path}")
        except Exception as e:
            print(f"Could not load observation.pkl: {e}")
            self.obs = None

    def _voxel_to_pointcloud(self, voxel_data) -> PointCloud2:
        """Convert Minecraft voxel data to PointCloud2 message."""
        blocks_movement = voxel_data["blocks_movement"]

        # Get voxel grid dimensions
        x_dim, y_dim, z_dim = blocks_movement.shape

        # Create point cloud
        points = []

        # Iterate through voxel grid
        for x in range(x_dim):
            for y in range(y_dim):
                for z in range(z_dim):
                    # Skip if block doesn't block movement (is passable)
                    if not blocks_movement[x, y, z]:
                        continue

                    # Convert voxel indices to world coordinates
                    # Center of voxel grid is at (5, 2, 5) in voxel space
                    world_x = (x - 5) * self.block_size
                    world_y = (y - 2) * self.block_size
                    world_z = (z - 5) * self.block_size

                    # Generate points within this block
                    for dx in range(self.points_per_block):
                        for dy in range(self.points_per_block):
                            for dz in range(self.points_per_block):
                                px = world_x + dx * self.resolution
                                pz = world_y + dy * self.resolution
                                py = world_z + dz * self.resolution
                                points.append([px, py, pz])

        # Convert to numpy array
        points_array = np.array(points, dtype=np.float32)

        # Create Open3D point cloud
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(points_array)

        # Create PointCloud2 wrapper
        pc2 = PointCloud2(pointcloud=o3d_pc, frame_id="world")

        return pc2

    # @functools.cache
    # def tf_stream(self) -> Subject[Transform]:
    #    base_link = functools.partial(Transform.from_pose, "base_link")
    #    return backpressure(self.odom_stream().pipe(ops.map(base_link)))

    @functools.cache
    def lidar_stream(self):
        print("lidar stream start")
        period = 1.0 / self.lidar_frequency  # 10Hz = 0.1s

        def create_pointcloud(_):
            if self.obs and "voxels" in self.obs:
                return self._voxel_to_pointcloud(self.obs["voxels"])
            else:
                # Return empty point cloud if no voxel data
                empty_pc = o3d.geometry.PointCloud()
                return PointCloud2(pointcloud=empty_pc, frame_id="world")

        return rx.interval(period).pipe(ops.map(create_pointcloud))

    @functools.cache
    def video_stream(self):
        print("video stream start")
        period = 1.0 / 10.0  # 10 FPS video stream

        def create_image(_):
            if self.obs and "rgb" in self.obs:
                # Convert from CHW to HWC format
                rgb_chw = self.obs["rgb"]  # (3, 800, 1280)
                rgb_hwc = np.transpose(rgb_chw, (1, 2, 0))  # (800, 1280, 3)

                # Create Image message
                return Image(data=rgb_hwc, format=ImageFormat.RGB, frame_id="world")
            else:
                # Return empty image if no RGB data
                empty_img = np.zeros((480, 640, 3), dtype=np.uint8)
                return Image(data=empty_img, format=ImageFormat.RGB, frame_id="world")

        return rx.interval(period).pipe(ops.map(create_image))

    def move(self, vector: Vector3):
        """Move the Minecraft character based on velocity vector."""
        if self.env is None:
            return

        # Create action from vector
        action = self.env.action_space.no_op()

        # Map vector to Minecraft movement
        # Forward/backward (x in robot frame -> forward in Minecraft)
        action[0] = np.clip(vector.x * 2, -1, 1)

        # Strafe left/right (y in robot frame -> strafe in Minecraft)
        action[1] = np.clip(vector.y * 2, -1, 1)

        # Jump if z velocity is positive
        if vector.z > 0.1:
            action[2] = 1

        # Step the environment
        self.obs, reward, done, info = self.env.step(action)

    def close(self):
        """Close the MineDojo environment."""
        if self.env:
            self.env.close()

    @rpc
    def start(self):
        self.lidar_stream().subscribe(self.lidar.publish)
        self.video_stream().subscribe(self.video.publish)
        # self.tf_stream().subscribe(self.tf.publish)


if __name__ == "__main__":
    import logging
    import time

    pubsub.lcm.autoconf()

    dimos = core.start(2)
    robot = dimos.deploy(Minecraft)
    bridge = dimos.deploy(FoxgloveBridge)
    robot.lidar.transport = core.LCMTransport("/lidar", PointCloud2)
    robot.video.transport = core.LCMTransport("/video", Image)

    bridge.start()
    robot.start()

    while True:
        time.sleep(1)
