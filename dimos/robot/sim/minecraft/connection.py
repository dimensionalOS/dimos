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
import os
import pickle
import random
import threading
import time
from typing import Optional

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


class Engine:
    _act = None
    frequency: float

    def __init__(self, frequency=20.0):
        self.frequency = frequency
        self._stop_event = threading.Event()
        self.loop_thread = None
        import minedojo

        self.env = minedojo.make(
            task_id="creative:1",
            image_size=(800, 1280),
            world_seed="dimensionalxx",
            use_voxel=True,
            voxel_size=dict(xmin=-10, ymin=-2, zmin=-10, xmax=10, ymax=2, zmax=10),
        )

    def noop(self):
        return self.env.action_space.no_op()

    def act(self, act):
        self._act = act

    def start(self):
        self._act = self.noop()
        self.env.reset()
        self._stop_event.clear()

    def get_stream(self):
        self.start()
        self._done = False

        def step_environment(_):
            if self._stop_event.is_set() or self._done:
                return None

            try:
                obs, reward, terminated, truncated, info = self.env.step(self._act)
                if terminated or truncated:
                    self._done = True
                return (obs, reward, terminated, truncated, info)
            except RuntimeError as e:
                if "done=True" in str(e):
                    self._done = True
                    return None
                raise

        def on_dispose():
            self.stop()

        return rx.interval(1.0 / self.frequency).pipe(
            ops.map(step_environment),
            ops.take_while(lambda x: x is not None),
            ops.finally_action(on_dispose),
        )

    def stop(self):
        self._stop_event.set()
        if hasattr(self, "env") and self.env:
            self.env.close()


class MinecraftConnection:
    def __init__(self, *args, **kwargs):
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

        # Current velocity command
        self.current_velocity = Vector3(0, 0, 0)

        # Use Engine instead of direct MineDojo
        self.engine = Engine(frequency=20.0)
        self.obs = None
        self.obs_subscription = None

        self.start()

    def _voxel_to_pointcloud(self, voxel_data, in_world_frame=True) -> PointCloud2:
        """Convert Minecraft voxel data to PointCloud2 message."""
        blocks_movement = voxel_data["blocks_movement"]

        # Get occupied voxel indices
        occupied_indices = np.where(blocks_movement)

        if len(occupied_indices[0]) == 0:
            # No occupied voxels
            points_array = np.empty((0, 3), dtype=np.float32)
        else:
            # Get player's continuous position and calculate offset from voxel grid center
            if self.obs and "location_stats" in self.obs:
                player_pos = self.obs["location_stats"]["pos"]

                # The voxel grid is centered on the player's current block (floor of position)
                # When player crosses into a new block, the whole grid shifts
                grid_block_x = np.floor(player_pos[0])
                grid_block_y = np.floor(player_pos[1])
                grid_block_z = np.floor(player_pos[2])

                # The voxel grid represents blocks in world space
                # We don't need offsets - the grid is absolute, not relative
                offset_x = 0
                offset_y = 0
                offset_z = 0

                # Debug logging to file with voxel grid info
                with open("/tmp/minecraft_lidar_debug.log", "a") as f:
                    f.write(
                        f"Player pos: {player_pos}, Grid shape: {blocks_movement.shape}, "
                        f"Player block: ({grid_block_x}, {grid_block_y}, {grid_block_z}), "
                        f"Occupied count: {len(occupied_indices[0])}\n"
                    )
            else:
                offset_x = offset_y = offset_z = 0.0

            # Convert occupied voxel indices to coordinates
            x_indices, y_indices, z_indices = occupied_indices

            # Get voxel grid center based on actual dimensions
            x_center = (blocks_movement.shape[0] - 1) / 2.0  # For 21x5x21 this is 10
            y_center = (blocks_movement.shape[1] - 1) / 2.0  # For 21x5x21 this is 2
            z_center = (blocks_movement.shape[2] - 1) / 2.0  # For 21x5x21 this is 10

            # Convert voxel indices to world block positions
            # The voxel grid shows blocks from (player_block - 10) to (player_block + 10)
            # Index 0 = player_block - 10, Index 10 = player_block, Index 20 = player_block + 10
            player_pos = self.obs["location_stats"]["pos"]
            player_block_x = np.floor(player_pos[0])
            player_block_y = np.floor(player_pos[1])
            player_block_z = np.floor(player_pos[2])

            # Convert indices to world block positions
            world_block_x = player_block_x + (x_indices - x_center)
            world_block_y = player_block_y + (y_indices - y_center)
            world_block_z = player_block_z + (z_indices - z_center)

            # Convert to continuous world coordinates
            # For Y, use block bottom instead of center to avoid player being inside blocks
            world_x = (world_block_x + 0.5) * self.block_size
            world_y = world_block_y * self.block_size  # Block bottom, not center
            world_z = (world_block_z + 0.5) * self.block_size

            # Convert to coordinates relative to player's actual position
            mc_x = world_x - player_pos[0] * self.block_size
            mc_y = world_y - player_pos[1] * self.block_size
            mc_z = world_z - player_pos[2] * self.block_size

            # Convert to robot frame (Minecraft X->Robot X, Z->Y, Y->Z)
            base_x = mc_x
            base_y = mc_z
            base_z = mc_y

            # Generate sub-voxel points using meshgrid
            sub_offsets = np.arange(self.points_per_block) * self.resolution
            dx, dy, dz = np.meshgrid(sub_offsets, sub_offsets, sub_offsets, indexing="ij")
            dx, dy, dz = dx.flatten(), dy.flatten(), dz.flatten()

            # Broadcast occupied voxel positions with sub-voxel offsets
            num_voxels = len(base_x)
            num_subpoints = len(dx)

            # Repeat base positions for each sub-point
            all_x = np.repeat(base_x, num_subpoints) + np.tile(dx, num_voxels)
            all_y = np.repeat(base_y, num_subpoints) + np.tile(dy, num_voxels)
            all_z = np.repeat(base_z, num_subpoints) + np.tile(dz, num_voxels)

            # Stack into points array
            points_array = np.column_stack([all_x, all_y, all_z]).astype(np.float32)

        # Transform to world frame if requested
        if in_world_frame and len(points_array) > 0:
            # Get current transform from world to base_link
            transform = self._create_transform_from_location()

            # Only apply translation, ignore rotation
            translation = np.array(
                [transform.translation.x, transform.translation.y, transform.translation.z]
            )
            points_array = points_array + translation

            frame_id = "world"
        else:
            frame_id = "base_link" if not in_world_frame else "world"

        # Create Open3D point cloud
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(points_array)

        # Create PointCloud2 wrapper
        pc2 = PointCloud2(pointcloud=o3d_pc, frame_id=frame_id)

        return pc2

    def _create_transform_from_location(self):
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
            z = rel_pos[1] * self.block_size + 0.3  # Minecraft Y -> Robot Z

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
                translation=Vector3(0, 0, 0),
                rotation=Quaternion(0, 0, 0, 1),
            )

    @functools.cache
    def odom_stream(self):
        """Stream transforms at 10Hz."""
        period = 0.1  # 10Hz
        return rx.interval(period).pipe(
            ops.map(lambda _: self._create_transform_from_location().to_pose())
        )

    @functools.cache
    def tf_stream(self):
        """Stream transforms at 10Hz."""
        print("tf stream start")
        period = 0.1  # 10Hz

        return rx.interval(period).pipe(ops.map(lambda _: self._create_transform_from_location()))

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
        """Handle movement commands."""
        self.current_velocity = vector

        # Convert velocity to MineDojo actions
        act = self.engine.noop()

        # Forward/backward movement
        if vector.y > 0.5:
            act[0] = 1  # forward
        elif vector.y < -0.5:
            act[0] = 2  # back
        else:
            act[0] = 0  # no movement

        # Strafe left/right
        if vector.x > 0.5:
            act[1] = 1  # left
        elif vector.x < -0.5:
            act[1] = 2  # right
        else:
            act[1] = 0  # no strafe

        # Jump if z velocity is positive
        if vector.z > 0.1:
            act[2] = 1  # jump
        else:
            act[2] = 0  # no jump

        self.engine.act(act)

    def close(self):
        """Close the MineDojo environment."""
        if self.obs_subscription:
            self.obs_subscription.dispose()
            self.obs_subscription = None
        self.engine.stop()

    def dispose(self):
        """Dispose of resources, including stopping the engine."""
        self.close()

    def start(self):
        # Subscribe to observation stream
        def on_observation(data):
            obs, reward, terminated, truncated, info = data
            self.obs = obs

        self.obs_subscription = self.engine.get_stream().subscribe(
            on_next=on_observation,
            on_error=lambda e: print(f"Engine error: {e}"),
            on_completed=lambda: print("Engine stream completed"),
        )


class MinecraftModule(Module, MinecraftConnection):
    movecmd: In[Vector3] = None
    odom: Out[Vector3] = None
    lidar: Out[PointCloud2] = None
    video: Out[Image] = None

    @rpc
    def move(self, vector: Vector3):
        """RPC method to handle move commands."""
        super().move(vector)

    @rpc
    def start(self):
        # Subscribe to movement commands
        self.movecmd.subscribe(self.move)

        # Start publishing sensor streams
        self.lidar_stream().subscribe(self.lidar.publish)
        self.video_stream().subscribe(self.video.publish)
        self.tf_stream().subscribe(self.tf.publish)

    def dispose(self):
        """Override dispose to ensure engine stops."""
        super().dispose()  # Call parent class dispose methods


if __name__ == "__main__":
    import logging

    pubsub.lcm.autoconf()

    dimos = core.start(2)
    robot = dimos.deploy(MinecraftModule)
    bridge = dimos.deploy(FoxgloveBridge)

    # Configure transports
    robot.movecmd.transport = core.LCMTransport("/movecmd", Vector3)
    robot.lidar.transport = core.LCMTransport("/lidar", PointCloud2)
    robot.video.transport = core.LCMTransport("/video", Image)

    bridge.start()
    robot.start()

    while True:
        time.sleep(1)
