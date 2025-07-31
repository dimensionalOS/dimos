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

"""Test PointCloud2 generation with Open3D visualization."""

import pickle
import numpy as np
import open3d as o3d
from dimos.robot.sim.minecraft.connection import Minecraft


def test_pointcloud_visualization():
    """Test PointCloud2 generation and visualize with Open3D."""
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
    print(f"Number of points: {len(np.asarray(pc2.pointcloud.points))}")

    # Get the Open3D point cloud
    o3d_pc = pc2.pointcloud

    # Add colors based on height (z-coordinate)
    points = np.asarray(o3d_pc.points)
    z_min, z_max = points[:, 2].min(), points[:, 2].max()

    # Normalize z values to [0, 1] for coloring
    z_normalized = (points[:, 2] - z_min) / (z_max - z_min + 1e-8)

    # Create color map (blue to red based on height)
    colors = np.zeros((len(points), 3))
    colors[:, 0] = z_normalized  # Red channel
    colors[:, 2] = 1 - z_normalized  # Blue channel

    o3d_pc.colors = o3d.utility.Vector3dVector(colors)

    # Create coordinate frame for reference
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

    # Visualize
    print("\nOpening Open3D viewer...")
    print("Controls:")
    print("  - Mouse: Rotate view")
    print("  - Scroll: Zoom")
    print("  - Q: Close viewer")

    o3d.visualization.draw_geometries(
        [o3d_pc, coordinate_frame],
        window_name="Minecraft Virtual Lidar - PointCloud2",
        width=1280,
        height=720,
        left=50,
        top=50,
        point_show_normal=False,
    )

    # Also test with voxel downsampling for performance
    print("\nCreating downsampled version...")
    downsampled = o3d_pc.voxel_down_sample(voxel_size=0.1)
    print(f"Downsampled points: {len(np.asarray(downsampled.points))}")

    o3d.visualization.draw_geometries(
        [downsampled, coordinate_frame],
        window_name="Minecraft Virtual Lidar - Downsampled (0.1m voxels)",
        width=1280,
        height=720,
        left=50,
        top=50,
        point_show_normal=False,
    )


if __name__ == "__main__":
    test_pointcloud_visualization()
