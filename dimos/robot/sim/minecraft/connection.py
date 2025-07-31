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
import random
from typing import Optional

import minedojo
import numpy as np
import open3d as o3d
import reactivex as rx
import reactivex.operators as ops

from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
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
        self.resolution = 0.2
        # Points per block dimension
        self.points_per_block = int(self.block_size / self.resolution)
        # Lidar frequency in Hz (10Hz is typical for robot lidars)
        self.lidar_frequency = 2.0

        self.tf = core.TF()

        # Origin offset - will be set to first position we see
        self.origin_offset = None

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

                    # Convert voxel indices to coordinates relative to player
                    # Voxel grid: x:[0,10] maps to [-5,5], y:[0,4] maps to [-2,2], z:[0,10] maps to [-5,5]
                    # These are Minecraft coordinates relative to player
                    mc_x = (x - 5) * self.block_size  # Minecraft X (forward/back)
                    mc_y = (y - 2) * self.block_size  # Minecraft Y (up/down)
                    mc_z = (z - 5) * self.block_size  # Minecraft Z (left/right)

                    # Convert to robot frame (relative to base_link)
                    # Minecraft X -> Robot X (forward)
                    # Minecraft Z -> Robot Y (left)
                    # Minecraft Y -> Robot Z (up)
                    world_x = mc_x
                    world_y = mc_z
                    world_z = mc_y

                    # Generate points within this block
                    for dx in range(self.points_per_block):
                        for dy in range(self.points_per_block):
                            for dz in range(self.points_per_block):
                                px = world_x + dx * self.resolution
                                pz = world_y + dy * self.resolution
                                py = world_z + dz * self.resolution
                                points.append([px, pz, py])

        # Convert to numpy array
        points_array = np.array(points, dtype=np.float32)

        # Create Open3D point cloud
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(points_array)

        # Create PointCloud2 wrapper
        pc2 = PointCloud2(pointcloud=o3d_pc, frame_id="base_link")

        return pc2

    def _create_transform_from_location(self, _):
        """Create a Transform from world to base_link using player location."""
        if self.obs and "location_stats" in self.obs:
            loc = self.obs["location_stats"]
            pos = loc["pos"]  # [x, y, z] in Minecraft coordinates
            yaw = loc["yaw"][0]  # Yaw in degrees
            pitch = loc["pitch"][0]  # Pitch in degrees

            # Set origin on first position
            if self.origin_offset is None:
                self.origin_offset = pos.copy()
                print(f"Setting world origin at Minecraft position: {self.origin_offset}")

            # Calculate position relative to origin
            rel_pos = pos - self.origin_offset

            # Convert Minecraft coordinates to robot coordinates
            # Minecraft Y is up, robot Z is up
            # Scale by block_size to convert to meters
            x = rel_pos[0] * self.block_size
            y = rel_pos[2] * self.block_size  # Minecraft Z -> Robot Y
            z = rel_pos[1] * self.block_size  # Minecraft Y -> Robot Z

            # Convert yaw and pitch from degrees to radians
            yaw_rad = np.radians(yaw)
            pitch_rad = np.radians(pitch)

            # Create quaternion from Euler angles (RPY)
            # Roll = 0, Pitch = pitch, Yaw = yaw
            cy = np.cos(yaw_rad * 0.5)
            sy = np.sin(yaw_rad * 0.5)
            cp = np.cos(pitch_rad * 0.5)
            sp = np.sin(pitch_rad * 0.5)
            cr = 1.0  # cos(0/2)
            sr = 0.0  # sin(0/2)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy

            # Create transform
            transform = Transform(
                parent_frame_id="world",
                child_frame_id="base_link",
                translation=Vector3(x, y, z),
                rotation=Quaternion(qx, qy, qz, qw),
            )

            return transform
        else:
            # Return identity transform if no location data
            return Transform(
                parent_frame_id="world",
                child_frame_id="base_link",
                translation=Vector3(x=0, y=0, z=0),
                rotation=Quaternion(x=0, y=0, z=0, w=1),
            )

    @functools.cache
    def tf_stream(self):
        """Stream transforms at 10Hz."""
        print("tf stream start")
        period = 0.1  # 10Hz

        return rx.interval(period).pipe(ops.map(self._create_transform_from_location))

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
                return PointCloud2(pointcloud=empty_pc, frame_id="base_link")

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

    def close(self):
        """Close the MineDojo environment."""
        if self.env:
            self.env.close()

    @rpc
    def start(self):
        self.env = minedojo.make(
            task_id="creative:1",
            image_size=(800, 1280),
            world_seed="dimensional",
            use_voxel=True,
            voxel_size=dict(xmin=-5, ymin=-2, zmin=-5, xmax=5, ymax=2, zmax=5),
        )
        self.env.reset()

        self.lidar_stream().subscribe(self.lidar.publish)
        self.video_stream().subscribe(self.video.publish)
        self.tf_stream().subscribe(self.tf.publish)

        i = 0
        while True:
            i += 1
            act = self.env.action_space.no_op()
            act[0] = 1  # forward/backward
            print(i)
            if i % 100 == 0:
                act[2] = 1  # jump
            obs, reward, terminated, truncated, info = self.env.step(act)
            self.obs = obs
            time.sleep(0.05)


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
