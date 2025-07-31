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

"""Test PointCloud2 generation from Minecraft voxel data."""

import pickle
import numpy as np
from dimos.robot.sim.minecraft.connection import Minecraft


def test_pointcloud_from_pickle():
    """Test PointCloud2 generation using pickled observation data."""
    # Load pickled observation
    with open("observation.pkl", "rb") as f:
        obs = pickle.load(f)

    # Create Minecraft instance (without connecting to actual env)
    mc = Minecraft()
    mc.obs = obs  # Use pickled observation

    # Generate PointCloud2
    pc2 = mc._voxel_to_pointcloud(obs["voxels"])

    print("=== PointCloud2 Generated ===")
    print(f"Frame ID: {pc2.frame_id}")
    print(f"Timestamp: {pc2.ts}")

    # Get points from Open3D pointcloud
    points_data = np.asarray(pc2.pointcloud.points)
    print(f"Number of points: {len(points_data)}")
    print(f"\nFirst 5 points:")
    for i in range(min(5, len(points_data))):
        print(
            f"  Point {i}: x={points_data[i][0]:.3f}, y={points_data[i][1]:.3f}, z={points_data[i][2]:.3f}"
        )

    print(f"\nPoint cloud bounds:")
    print(f"  X: [{points_data[:, 0].min():.3f}, {points_data[:, 0].max():.3f}]")
    print(f"  Y: [{points_data[:, 1].min():.3f}, {points_data[:, 1].max():.3f}]")
    print(f"  Z: [{points_data[:, 2].min():.3f}, {points_data[:, 2].max():.3f}]")

    # Check voxel data that was converted
    blocks_movement = obs["voxels"]["blocks_movement"]
    solid_blocks = np.sum(blocks_movement)
    print(f"\nVoxel data:")
    print(f"  Total solid blocks: {solid_blocks}")
    print(f"  Points per block: {mc.points_per_block}")
    print(f"  Expected points: {solid_blocks * mc.points_per_block**3}")

    return pc2


if __name__ == "__main__":
    test_pointcloud_from_pickle()
