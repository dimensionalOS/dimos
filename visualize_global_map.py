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

"""Visualize global map voxels using Open3D with wireframes and height-based colors."""

import time

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.protocol.pubsub import lcm  # type: ignore[attr-defined]
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage

# Configure LCM
lcm.autoconf()
lcm_instance = lcm.LCM()

# Store the latest map
latest_map = None
map_lock = False


def _height_to_color(height: float, min_height: float, max_height: float) -> list[float]:
    """Convert height to a color gradient (blue -> green -> yellow -> orange -> red).

    Args:
        height: Z-coordinate (height) value
        min_height: Minimum height in the point cloud
        max_height: Maximum height in the point cloud

    Returns:
        RGB color [r, g, b] in range [0, 1]
    """
    if max_height == min_height:
        return [0.0, 0.5, 1.0]  # Default to blue

    # Normalize height to [0, 1]
    t = (height - min_height) / (max_height - min_height)
    t = np.clip(t, 0.0, 1.0)

    # Create gradient: blue (0.0) -> cyan -> green -> yellow -> orange -> red (1.0)
    if t < 0.2:
        # Blue to cyan
        r = 0.0
        g = t * 5.0 * 0.5  # 0 to 0.5
        b = 1.0
    elif t < 0.4:
        # Cyan to green
        r = 0.0
        g = 0.5 + (t - 0.2) * 5.0 * 0.5  # 0.5 to 1.0
        b = 1.0 - (t - 0.2) * 5.0 * 0.5  # 1.0 to 0.5
    elif t < 0.6:
        # Green to yellow
        r = (t - 0.4) * 5.0  # 0 to 1.0
        g = 1.0
        b = 0.5 - (t - 0.4) * 5.0 * 0.5  # 0.5 to 0.0
    elif t < 0.8:
        # Yellow to orange
        r = 1.0
        g = 1.0 - (t - 0.6) * 5.0 * 0.5  # 1.0 to 0.5
        b = 0.0
    else:
        # Orange to red
        r = 1.0
        g = max(0.0, 0.5 - (t - 0.8) * 5.0 * 0.5)  # 0.5 to 0.0, ensure >= 0
        b = 0.0

    # Ensure values are in [0, 1] range
    r = np.clip(r, 0.0, 1.0)
    g = np.clip(g, 0.0, 1.0)
    b = np.clip(b, 0.0, 1.0)

    return [float(r), float(g), float(b)]


def _create_voxel_wireframe(
    voxel_grid: o3d.geometry.VoxelGrid, edge_color: list[float] | None = None
) -> o3d.geometry.LineSet:
    """Create a wireframe LineSet showing the edges of all voxels.

    Args:
        voxel_grid: The VoxelGrid to create wireframe for
        edge_color: RGB color for edges [r, g, b] in range [0, 1]

    Returns:
        LineSet with all voxel edges
    """
    if edge_color is None:
        edge_color = [1.0, 1.0, 1.0]
    points = []
    lines = []

    half_size = voxel_grid.voxel_size / 2.0

    # For each voxel, create the 12 edges of a cube
    for voxel in voxel_grid.get_voxels():
        # Get voxel center
        center = voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
        cx, cy, cz = center

        # Define the 8 corners of the cube
        corners = [
            [cx - half_size, cy - half_size, cz - half_size],  # 0: bottom-left-back
            [cx + half_size, cy - half_size, cz - half_size],  # 1: bottom-right-back
            [cx + half_size, cy + half_size, cz - half_size],  # 2: top-right-back
            [cx - half_size, cy + half_size, cz - half_size],  # 3: top-left-back
            [cx - half_size, cy - half_size, cz + half_size],  # 4: bottom-left-front
            [cx + half_size, cy - half_size, cz + half_size],  # 5: bottom-right-front
            [cx + half_size, cy + half_size, cz + half_size],  # 6: top-right-front
            [cx - half_size, cy + half_size, cz + half_size],  # 7: top-left-front
        ]

        # Add corners to points list and get their indices
        base_idx = len(points)
        points.extend(corners)

        # Define the 12 edges of a cube (each edge connects two corners)
        cube_edges = [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],  # Back face
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],  # Front face
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],  # Connecting edges
        ]

        # Add edges with correct indices
        for edge in cube_edges:
            lines.append([base_idx + edge[0], base_idx + edge[1]])

    # Create LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(points))
    line_set.lines = o3d.utility.Vector2iVector(np.array(lines))
    line_set.paint_uniform_color(edge_color)

    return line_set


def _create_voxel_grid_from_pointcloud(
    pcd: o3d.geometry.PointCloud, voxel_size: float
) -> o3d.geometry.VoxelGrid:
    """Create a VoxelGrid from a point cloud with height-based color gradient.

    Args:
        pcd: Input point cloud (points at voxel centers)
        voxel_size: Size of each voxel cube

    Returns:
        VoxelGrid with height-based colors
    """
    if len(pcd.points) == 0:
        return o3d.geometry.VoxelGrid()

    # Color points based on height (z-coordinate)
    points = np.asarray(pcd.points)
    if len(points) > 0:
        z_coords = points[:, 2]
        min_z = float(np.min(z_coords))
        max_z = float(np.max(z_coords))

        # Create colors for each point based on height
        colors = np.array([_height_to_color(z, min_z, max_z) for z in z_coords])
        pcd_colored = o3d.geometry.PointCloud(pcd)
        pcd_colored.colors = o3d.utility.Vector3dVector(colors)
    else:
        pcd_colored = o3d.geometry.PointCloud(pcd)

    # Create voxel grid from colored point cloud
    # Open3D will assign colors to voxels based on the point cloud colors
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_colored, voxel_size=voxel_size)

    return voxel_grid


def visualize_map(msg: LidarMessage) -> None:
    """Visualize a global map with wireframes and height-based colors."""
    pcd = msg.pointcloud
    voxel_size = msg.resolution

    if len(pcd.points) == 0:
        print("Warning: Empty point cloud, nothing to visualize")
        return

    print("\nVisualizing global map:")
    print(f"  Points: {len(pcd.points)}")
    print(f"  Voxel size: {voxel_size}m")

    # Get height range
    points = np.asarray(pcd.points)
    z_coords = points[:, 2]
    min_z = float(np.min(z_coords))
    max_z = float(np.max(z_coords))
    print(f"  Height range: {min_z:.2f}m to {max_z:.2f}m")

    # Create voxel grid with height-based colors
    print("  Creating voxel grid with height-based color gradient...")
    voxel_grid = _create_voxel_grid_from_pointcloud(pcd, voxel_size)
    num_voxels = len(voxel_grid.get_voxels())
    print(f"  Voxels: {num_voxels}")

    # Create wireframe
    print("  Creating wireframe edges...")
    voxel_wireframe = _create_voxel_wireframe(voxel_grid, edge_color=[1.0, 1.0, 1.0])

    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Global Map (Voxels with Wireframes)", width=1280, height=720)

    # Add geometries
    vis.add_geometry(voxel_grid)
    vis.add_geometry(voxel_wireframe)
    vis.add_geometry(coord_frame)

    # Configure render options
    render_opt = vis.get_render_option()
    render_opt.background_color = [0.05, 0.05, 0.05]  # Very dark background
    render_opt.point_size = 1.5
    render_opt.mesh_show_wireframe = False
    render_opt.mesh_show_back_face = True
    render_opt.line_width = 1.5
    render_opt.show_coordinate_frame = True

    # Set up view
    map_center = pcd.get_center()
    view_ctl = vis.get_view_control()
    view_ctl.set_front([0, 0, -1])
    view_ctl.set_lookat(map_center)
    view_ctl.set_up([0, -1, 0])
    view_ctl.set_zoom(0.7)

    vis.poll_events()
    vis.update_renderer()
    vis.run()


def on_global_map(msg: LidarMessage, _) -> None:
    """Handle global map messages."""
    global latest_map, map_lock
    if not map_lock:
        latest_map = msg
        print(
            f"Received global map: {len(msg.pointcloud.points)} points, resolution={msg.resolution}"
        )


# Subscribe to topic
print("Subscribing to /global_map...")
print("Waiting for messages. Press Ctrl+C to visualize the latest map.\n")

lcm_instance.subscribe(lcm.Topic("/global_map", LidarMessage), on_global_map)
lcm_instance.start()

try:
    while True:
        time.sleep(0.1)
        if latest_map is not None:
            # Visualize the latest map
            print(f"\n{'=' * 60}")
            print("Visualizing global map with wireframes and height-based colors...")
            print("Close the Open3D window to continue receiving updates.")
            map_lock = True
            visualize_map(latest_map)
            map_lock = False
            latest_map = None  # Reset to get next update
except KeyboardInterrupt:
    if latest_map is not None:
        print(f"\n{'=' * 60}")
        print("Visualizing final map...")
        visualize_map(latest_map)
    else:
        print("\nNo map received yet.")
    lcm_instance.stop()
