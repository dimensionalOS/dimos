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
Debug script for testing map building with multiple frames.

Loads pickled lidar messages and visualizes map building by splicing multiple frames
sequentially, matching the production Map module behavior.

Usage:
    python3 voxels_from_data.py --multi 100                         # processes 100 frames
    python3 voxels_from_data.py --multi 100 --show-timing-graph     # shows the graph of the processing times for each frame and the load times for each frame

    python3 voxels_from_data.py --multi 100 --start 0 --voxel-size 0.5 --splice-method cylinder --dataset unitree_go2_office_walk2 --show-timing-graph

"""

import argparse
from pathlib import Path
import sys
import time

import matplotlib.pyplot as plt  # type: ignore[import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import splice_cylinder, splice_sphere
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay


def load_lidar_frame(dataset_name: str, frame_idx: int) -> LidarMessage:
    """Load a single lidar frame from the dataset.

    Handles both cases:
    - Datasets with raw messages (needs autocast)
    - Datasets with already-converted LidarMessage objects (no autocast needed)
    """
    # Try loading without autocast first (for datasets where data is already LidarMessage)
    replay: TimedSensorReplay = TimedSensorReplay(f"{dataset_name}/lidar", autocast=None)
    result = replay.load_one(f"{frame_idx:03d}")

    if isinstance(result, tuple):
        # TimedSensorReplay returns (timestamp, data) tuple
        data = result[1]
        if isinstance(data, LidarMessage):
            # Already a LidarMessage, return it
            return data
        else:
            # Raw message, need to convert
            return LidarMessage.from_msg(data)
    else:
        # Not a tuple, check if it's already a LidarMessage
        if isinstance(result, LidarMessage):
            return result
        # Otherwise it's a raw message
        return LidarMessage.from_msg(result)


def visualize_multiple_frames(
    frames: list[LidarMessage],
    start_idx: int,
    total_frames: int | None = None,
    voxel_size: float = 0.5,
    splice_method: str = "cylinder",
    shrink: float = 0.5,
    frame_load_times: list[float] | None = None,
    show_timing_graph: bool = False,
) -> None:
    """Visualize map building by splicing multiple frames sequentially.

    Args:
        frames: List of LidarMessage frames to process
        start_idx: Starting frame index for display purposes
        total_frames: Total number of frames in dataset (for progress display)
        voxel_size: Size of each voxel cube in meters
        splice_method: Method to use for splicing ("cylinder" or "sphere")
        shrink: Shrink factor for splicing (default: 0.5)
        frame_load_times: Optional list of frame loading times for statistics
        show_timing_graph: If True, display timing graph visualization (default: False)
    """
    num_frames = len(frames)
    total_info = f"/{total_frames}" if total_frames is not None else ""

    print(f"\n{'=' * 60}")
    print(f"Building map from {num_frames} frames (starting at frame {start_idx}{total_info})")
    print(f"Voxel size: {voxel_size}, Splice method: {splice_method}, Shrink: {shrink}")
    print(f"{'=' * 60}\n")

    # Track processing times for each frame
    processing_times: list[float] = []
    frame_indices: list[int] = []

    # Start overall timer
    overall_start_time = time.time()

    # Initialize map with first frame using proper voxelization (same as Map._voxelize_pointcloud)
    frame_start_time = time.time()
    map_pcd, _ = _voxelize_pointcloud(frames[0].pointcloud, voxel_size, use_height_gradient=False)
    frame_processing_time = time.time() - frame_start_time
    processing_times.append(frame_processing_time)
    frame_indices.append(start_idx)

    # Sequentially add frames (like Map.add_frame does)
    for i, frame in enumerate(frames[1:], start=1):
        frame_idx = start_idx + i
        progress_info = (
            f"processed {i}/{num_frames} frames" if total_frames else f"frame {frame_idx}"
        )

        # Time the processing of this frame
        frame_start_time = time.time()

        # Use proper voxelization method (same as Map._voxelize_pointcloud in production)
        new_pcd, _ = _voxelize_pointcloud(frame.pointcloud, voxel_size, use_height_gradient=False)

        if len(new_pcd.points) == 0:
            frame_processing_time = time.time() - frame_start_time
            processing_times.append(frame_processing_time)
            frame_indices.append(frame_idx)
            print(
                f"Frame {frame_idx}: Skipped (empty after voxelization) - {progress_info} [{frame_processing_time * 1000:.1f}ms]"
            )
            continue

        # Splice into map
        if splice_method == "cylinder":
            map_pcd = splice_cylinder(map_pcd, new_pcd, shrink=shrink)
        else:
            map_pcd = splice_sphere(map_pcd, new_pcd, shrink=shrink)

        frame_processing_time = time.time() - frame_start_time
        processing_times.append(frame_processing_time)
        frame_indices.append(frame_idx)
        print(
            f"Frame {frame_idx}: {len(frame.pointcloud.points)} → {len(new_pcd.points)} points → Map: {len(map_pcd.points)} points - {progress_info} [{frame_processing_time * 1000:.1f}ms]"
        )

    overall_processing_time = time.time() - overall_start_time

    print(f"\n{'=' * 60}")
    print("Final Map Statistics:")
    print(f"  Total frames processed: {len(frames)}")
    print(f"  Final map points: {len(map_pcd.points)}")
    print(f"  Average points per frame in map: {len(map_pcd.points) / len(frames):.1f}")

    # Timing statistics
    print(f"\n{'=' * 60}")
    print("Timing Statistics:")
    print(f"  Overall processing time: {overall_processing_time:.2f}s")
    if frame_load_times:
        total_load_time = sum(frame_load_times)
        print(f"  Total frame loading time: {total_load_time:.2f}s")
        print(f"  Total time (load + process): {total_load_time + overall_processing_time:.2f}s")
        print(
            f"  Average load time per frame: {total_load_time / len(frame_load_times) * 1000:.1f}ms"
        )
    print(f"  Average processing time per frame: {np.mean(processing_times) * 1000:.1f}ms")
    print(f"  Min processing time: {np.min(processing_times) * 1000:.1f}ms")
    print(f"  Max processing time: {np.max(processing_times) * 1000:.1f}ms")
    print(f"  Median processing time: {np.median(processing_times) * 1000:.1f}ms")

    # Get height range for info
    points = np.asarray(map_pcd.points)
    if len(points) > 0:
        z_coords = points[:, 2]
        min_z = float(np.min(z_coords))
        max_z = float(np.max(z_coords))
        print(f"  Height range: {min_z:.2f}m to {max_z:.2f}m")

    # Create timing graph (only if requested)
    if show_timing_graph:
        print(f"\n{'=' * 60}")
        print("Generating timing graph...")
        _plot_timing_graph(
            frame_indices, processing_times, frame_load_times, overall_processing_time
        )

    # Create voxel grid from final map with height-based color gradient
    print("\nCreating voxel grid visualization...")
    final_vox_pcd, final_vox_grid = _voxelize_pointcloud(
        map_pcd, voxel_size, use_height_gradient=True
    )
    num_voxels = len(final_vox_grid.get_voxels())
    print(f"  Final voxel grid: {num_voxels} voxels")

    # Visualize final map with wireframes and height gradient
    print("\nVisualizing final accumulated map...")
    print("  - Voxel cubes (height-colored: blue→green→yellow→red)")
    print("  - White wireframe edges")
    print("  - Coordinate frame")

    # Create wireframe for voxel grid
    voxel_wireframe = _create_voxel_wireframe(
        final_vox_grid, edge_color=[1.0, 1.0, 1.0]
    )  # White edges

    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    # Create visualizer with proper settings
    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=f"Global Map from {len(frames)} frames (voxel={voxel_size}, method={splice_method})",
        width=1280,
        height=720,
    )

    # Add geometries
    vis.add_geometry(final_vox_grid)
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

    # Set up view to look at the center of the map
    map_center = map_pcd.get_center()
    view_ctl = vis.get_view_control()
    view_ctl.set_front([0, 0, -1])
    view_ctl.set_lookat(map_center)
    view_ctl.set_up([0, -1, 0])
    view_ctl.set_zoom(0.7)  # Zoom out a bit to see more

    vis.poll_events()
    vis.update_renderer()
    vis.run()


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
    points: list[list[float]] = []
    lines: list[list[int]] = []

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
        cube_edges: list[list[int]] = [
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


def _plot_timing_graph(
    frame_indices: list[int],
    processing_times: list[float],
    frame_load_times: list[float] | None,
    overall_time: float,
) -> None:
    """Plot timing graph showing time per frame.

    Creates a two-panel matplotlib figure showing:
    - Top panel: Processing time per frame with mean line
    - Bottom panel: Load time, processing time, and combined time (if load times available),
      or a bar chart of processing times if load times are not available

    Args:
        frame_indices: List of frame indices corresponding to each timing measurement
        processing_times: List of processing times in seconds for each frame
        frame_load_times: Optional list of frame loading times in seconds
        overall_time: Total overall processing time in seconds
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Convert to milliseconds
    processing_times_ms = [t * 1000 for t in processing_times]

    # Plot 1: Processing time per frame
    ax1 = axes[0]
    ax1.plot(
        frame_indices,
        processing_times_ms,
        "b-o",
        markersize=4,
        linewidth=1.5,
        label="Processing time",
    )
    ax1.axhline(
        y=np.mean(processing_times_ms),
        color="r",
        linestyle="--",
        linewidth=1,
        label=f"Mean: {np.mean(processing_times_ms):.1f}ms",
    )
    ax1.set_xlabel("Frame Index", fontsize=11)
    ax1.set_ylabel("Processing Time (ms)", fontsize=11)
    ax1.set_title("Frame Processing Time", fontsize=12, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Plot 2: Combined load + processing time (if available)
    ax2 = axes[1]
    if frame_load_times and len(frame_load_times) == len(frame_indices):
        load_times_ms = [t * 1000 for t in frame_load_times]
        combined_times_ms = [
            load + proc for load, proc in zip(load_times_ms, processing_times_ms, strict=False)
        ]

        ax2.plot(
            frame_indices,
            load_times_ms,
            "g-o",
            markersize=4,
            linewidth=1.5,
            label="Load time",
            alpha=0.7,
        )
        ax2.plot(
            frame_indices,
            processing_times_ms,
            "b-o",
            markersize=4,
            linewidth=1.5,
            label="Processing time",
            alpha=0.7,
        )
        ax2.plot(
            frame_indices,
            combined_times_ms,
            "r-o",
            markersize=4,
            linewidth=2,
            label="Total time (load + process)",
        )
        ax2.set_ylabel("Time (ms)", fontsize=11)
        ax2.set_title("Frame Load + Processing Time", fontsize=12, fontweight="bold")
    else:
        # If no load times, just show processing times again with different view
        ax2.bar(
            frame_indices,
            processing_times_ms,
            width=0.8,
            alpha=0.7,
            color="steelblue",
            label="Processing time",
        )
        ax2.axhline(
            y=np.mean(processing_times_ms),
            color="r",
            linestyle="--",
            linewidth=2,
            label=f"Mean: {np.mean(processing_times_ms):.1f}ms",
        )
        ax2.set_ylabel("Processing Time (ms)", fontsize=11)
        ax2.set_title("Frame Processing Time (Bar Chart)", fontsize=12, fontweight="bold")

    ax2.set_xlabel("Frame Index", fontsize=11)
    ax2.grid(True, alpha=0.3, axis="y")
    ax2.legend()

    # Add overall statistics text
    stats_text = (
        f"Overall Statistics:\n"
        f"Total frames: {len(frame_indices)}\n"
        f"Total processing time: {overall_time:.2f}s\n"
        f"Avg processing: {np.mean(processing_times_ms):.1f}ms\n"
        f"Min/Max: {np.min(processing_times_ms):.1f}ms / {np.max(processing_times_ms):.1f}ms"
    )
    if frame_load_times:
        total_load = sum(frame_load_times)
        stats_text += f"\nTotal load time: {total_load:.2f}s\n"
        stats_text += f"Total time: {total_load + overall_time:.2f}s"

    fig.text(
        0.02,
        0.02,
        stats_text,
        fontsize=9,
        verticalalignment="bottom",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    plt.tight_layout()
    plt.show()
    print("  ✓ Timing graph displayed")


def _voxelize_pointcloud(
    pcd: o3d.geometry.PointCloud, voxel_size: float, use_height_gradient: bool = True
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.VoxelGrid]:
    """Properly voxelize a point cloud using Open3D VoxelGrid.

    Returns both the voxelized point cloud (points at voxel centers) and the VoxelGrid object.

    Args:
        pcd: Input point cloud
        voxel_size: Size of each voxel cube
        use_height_gradient: If True, color voxels based on height (blue to red gradient).
                            If False, uses original point cloud colors.
    """
    if len(pcd.points) == 0:
        return pcd, o3d.geometry.VoxelGrid()

    # Create a copy of the point cloud for coloring
    pcd_colored = o3d.geometry.PointCloud(pcd)

    if use_height_gradient:
        # Color points based on height (z-coordinate)
        points = np.asarray(pcd.points)
        if len(points) > 0:
            z_coords = points[:, 2]
            min_z = float(np.min(z_coords))
            max_z = float(np.max(z_coords))

            # Create colors for each point based on height
            colors = np.array([_height_to_color(z, min_z, max_z) for z in z_coords])
            pcd_colored.colors = o3d.utility.Vector3dVector(colors)

    # Create voxel grid from colored point cloud
    # Open3D will assign colors to voxels based on the point cloud colors
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_colored, voxel_size=voxel_size)

    # Extract voxel centers to get properly voxelized point cloud
    voxel_centers = []
    for voxel in voxel_grid.get_voxels():
        # Get the voxel center position
        center = voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
        voxel_centers.append(center)

    if len(voxel_centers) == 0:
        return o3d.geometry.PointCloud(), voxel_grid

    # Create new point cloud from voxel centers
    voxelized_pcd = o3d.geometry.PointCloud()
    voxelized_pcd.points = o3d.utility.Vector3dVector(np.array(voxel_centers))

    return voxelized_pcd, voxel_grid


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Debug map building with multiple frames - visualizes map building by splicing frames sequentially"
    )
    parser.add_argument(
        "--dataset",
        type=str,
        default="unitree_go2_office_walk2",
        help="Dataset name (default: unitree_go2_office_walk2)",
    )
    parser.add_argument(
        "--voxel-size", type=float, default=0.5, help="Voxel size in meters (default: 0.5)"
    )
    parser.add_argument(
        "--splice-method",
        type=str,
        choices=["cylinder", "sphere"],
        default="cylinder",
        help="Splicing method: cylinder or sphere (default: cylinder)",
    )
    parser.add_argument(
        "--shrink", type=float, default=0.5, help="Shrink factor for splicing (default: 0.5)"
    )
    parser.add_argument("--list-frames", action="store_true", help="List available frames and exit")
    parser.add_argument(
        "--multi",
        type=int,
        metavar="N",
        help="Number of frames to process (builds map incrementally). Required unless --list-frames is used.",
    )
    parser.add_argument("--start", type=int, default=0, help="Starting frame index (default: 0)")
    parser.add_argument(
        "--show-timing-graph",
        action="store_true",
        help="Show timing graph visualization (default: False)",
    )

    args = parser.parse_args()

    # Check if dataset exists
    try:
        data_dir = get_data(args.dataset)
        lidar_dir = data_dir / "lidar"
        if not lidar_dir.exists():
            print(f"Error: Dataset '{args.dataset}' not found or missing lidar/ directory")
            print(f"Looking for: {lidar_dir}")
            sys.exit(1)
    except Exception as e:
        print(f"Error loading dataset '{args.dataset}': {e}")
        sys.exit(1)

    # List frames if requested
    if args.list_frames:
        replay = TimedSensorReplay(f"{args.dataset}/lidar", autocast=LidarMessage.from_msg)
        files = replay.files
        total_frames = len(files)
        print(f"\n{'=' * 60}")
        print(f"Dataset: {args.dataset}")
        print(f"{'=' * 60}")
        print(f"Total available frames: {total_frames}")
        print(f"Frame indices: 0 to {total_frames - 1}")
        if total_frames > 0:
            print("\nFirst few frames:")
            for i, f in enumerate(files[:10]):
                print(f"  Frame {i}: {Path(f).name}")
            if total_frames > 10:
                print(f"  ... and {total_frames - 10} more frames")
            print("\nLast few frames:")
            for i, f in enumerate(files[-5:], start=total_frames - 5):
                print(f"  Frame {i}: {Path(f).name}")
        print(f"{'=' * 60}\n")
        sys.exit(0)

    # Require --multi if not listing frames (always multi-frame mode)
    if args.multi is None:
        parser.error("--multi is required (unless using --list-frames)")

    num_frames = args.multi
    start_idx = args.start

    # Get total frame count from dataset
    replay = TimedSensorReplay(f"{args.dataset}/lidar", autocast=LidarMessage.from_msg)
    total_frames = len(replay.files)

    print(
        f"Loading {num_frames} frames starting from frame {start_idx} (total available: {total_frames})..."
    )
    try:
        frames = []
        frame_load_times: list[float] = []
        load_start_time = time.time()

        for i in range(num_frames):
            frame_idx = start_idx + i
            if frame_idx >= total_frames:
                print(
                    f"Warning: Frame {frame_idx} exceeds total frames ({total_frames}), stopping early"
                )
                break

            # Time the loading of this frame
            frame_load_start = time.time()
            frame = load_lidar_frame(args.dataset, frame_idx)
            frame_load_time = time.time() - frame_load_start
            frame_load_times.append(frame_load_time)

            frames.append(frame)
            # Show progress like dimos replay
            current_num = i + 1
            print(
                f"Loaded frame {current_num}/{num_frames} (frame index {frame_idx}/{total_frames - 1}) [{frame_load_time * 1000:.1f}ms]",
                end="\r",
            )

        total_load_time = time.time() - load_start_time
        print()  # New line after progress
        print(
            f"Total loading time: {total_load_time:.2f}s (avg: {total_load_time / len(frames) * 1000:.1f}ms per frame)"
        )

        visualize_multiple_frames(
            frames,
            start_idx,
            total_frames=total_frames,
            voxel_size=args.voxel_size,
            splice_method=args.splice_method,
            shrink=args.shrink,
            frame_load_times=frame_load_times,
            show_timing_graph=args.show_timing_graph,
        )
    except Exception as e:
        print(f"Error loading frames: {e}")
        print("\nTry --list-frames to see available frame indices")
        sys.exit(1)


if __name__ == "__main__":
    main()
