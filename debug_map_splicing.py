#!/usr/bin/env python3
"""
Debug script for testing map building with multiple frames.

Loads pickled lidar messages and visualizes map building by splicing multiple frames
sequentially, matching the production Map module behavior.

Usage:
    python3 debug_map_splicing.py --multi 10 [--voxel-size 0.1] [--splice-method cylinder]
    python3 debug_map_splicing.py --list-frames  # List available frames
"""

import argparse
import sys
from pathlib import Path

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
    replay = TimedSensorReplay(f"{dataset_name}/lidar", autocast=None)
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
) -> None:
    """Visualize map building by splicing multiple frames sequentially."""
    num_frames = len(frames)
    total_info = f"/{total_frames}" if total_frames is not None else ""
    
    print(f"\n{'='*60}")
    print(f"Building map from {num_frames} frames (starting at frame {start_idx}{total_info})")
    print(f"Voxel size: {voxel_size}, Splice method: {splice_method}, Shrink: {shrink}")
    print(f"{'='*60}\n")
    
    # Initialize map with first frame using proper voxelization (same as Map._voxelize_pointcloud)
    map_pcd, _ = _voxelize_pointcloud(frames[0].pointcloud, voxel_size, use_height_gradient=False)
    processed = 1
    progress_info = f"processed {processed}/{num_frames} frames" if total_frames else f"frame {start_idx}"
    print(f"Frame {start_idx}: {len(frames[0].pointcloud.points)} → {len(map_pcd.points)} points (after proper voxelization) - {progress_info}")
    
    # Sequentially add frames (like Map.add_frame does)
    for i, frame in enumerate(frames[1:], start=1):
        frame_idx = start_idx + i
        processed = i + 1
        progress_info = f"processed {processed}/{num_frames} frames" if total_frames else f"frame {frame_idx}"
        # Use proper voxelization method (same as Map._voxelize_pointcloud in production)
        new_pcd, _ = _voxelize_pointcloud(frame.pointcloud, voxel_size, use_height_gradient=False)
        
        if len(new_pcd.points) == 0:
            print(f"Frame {frame_idx}: Skipped (empty after voxelization) - {progress_info}")
            continue
        
        # Splice into map
        if splice_method == "cylinder":
            map_pcd = splice_cylinder(map_pcd, new_pcd, shrink=shrink)
        else:
            map_pcd = splice_sphere(map_pcd, new_pcd, shrink=shrink)
        
        print(f"Frame {frame_idx}: {len(frame.pointcloud.points)} → {len(new_pcd.points)} points → Map: {len(map_pcd.points)} points - {progress_info}")
    
    print(f"\n{'='*60}")
    print(f"Final Map Statistics:")
    print(f"  Total frames processed: {len(frames)}")
    print(f"  Final map points: {len(map_pcd.points)}")
    print(f"  Average points per frame in map: {len(map_pcd.points) / len(frames):.1f}")
    
    # Get height range for info
    points = np.asarray(map_pcd.points)
    if len(points) > 0:
        z_coords = points[:, 2]
        min_z = float(np.min(z_coords))
        max_z = float(np.max(z_coords))
        print(f"  Height range: {min_z:.2f}m to {max_z:.2f}m")
    
    # Create voxel grid from final map with height-based color gradient
    print(f"\nCreating voxel grid visualization...")
    final_vox_pcd, final_vox_grid = _voxelize_pointcloud(map_pcd, voxel_size, use_height_gradient=True)
    num_voxels = len(final_vox_grid.get_voxels())
    print(f"  Final voxel grid: {num_voxels} voxels")
    
    # Visualize final map with wireframes and height gradient
    print(f"\nVisualizing final accumulated map...")
    print(f"  - Voxel cubes (height-colored: blue→green→yellow→red)")
    print(f"  - White wireframe edges")
    print(f"  - Coordinate frame")
    
    # Create wireframe for voxel grid
    voxel_wireframe = _create_voxel_wireframe(final_vox_grid, edge_color=[1.0, 1.0, 1.0])  # White edges
    
    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    
    # Create visualizer with proper settings
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Global Map from {len(frames)} frames (voxel={voxel_size}, method={splice_method})", width=1280, height=720)
    
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


def _create_voxel_wireframe(voxel_grid: o3d.geometry.VoxelGrid, edge_color: list[float] = [1.0, 1.0, 1.0]) -> o3d.geometry.LineSet:
    """Create a wireframe LineSet showing the edges of all voxels.
    
    Args:
        voxel_grid: The VoxelGrid to create wireframe for
        edge_color: RGB color for edges [r, g, b] in range [0, 1]
    
    Returns:
        LineSet with all voxel edges
    """
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
            [0, 1], [1, 2], [2, 3], [3, 0],  # Back face
            [4, 5], [5, 6], [6, 7], [7, 4],  # Front face
            [0, 4], [1, 5], [2, 6], [3, 7],  # Connecting edges
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


def _voxelize_pointcloud(pcd: o3d.geometry.PointCloud, voxel_size: float, use_height_gradient: bool = True) -> tuple[o3d.geometry.PointCloud, o3d.geometry.VoxelGrid]:
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
        help="Dataset name (default: unitree_go2_office_walk2)"
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.5,
        help="Voxel size in meters (default: 0.1)"
    )
    parser.add_argument(
        "--splice-method",
        type=str,
        choices=["cylinder", "sphere"],
        default="cylinder",
        help="Splicing method: cylinder or sphere (default: cylinder)"
    )
    parser.add_argument(
        "--shrink",
        type=float,
        default=0.5,
        help="Shrink factor for splicing (default: 0.5)"
    )
    parser.add_argument(
        "--list-frames",
        action="store_true",
        help="List available frames and exit"
    )
    parser.add_argument(
        "--multi",
        type=int,
        metavar="N",
        help="Number of frames to process (builds map incrementally). Required unless --list-frames is used."
    )
    parser.add_argument(
        "--start",
        type=int,
        default=0,
        help="Starting frame index (default: 0)"
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
        print(f"\n{'='*60}")
        print(f"Dataset: {args.dataset}")
        print(f"{'='*60}")
        print(f"Total available frames: {total_frames}")
        print(f"Frame indices: 0 to {total_frames-1}")
        if total_frames > 0:
            print(f"\nFirst few frames:")
            for i, f in enumerate(files[:10]):
                print(f"  Frame {i}: {Path(f).name}")
            if total_frames > 10:
                print(f"  ... and {total_frames - 10} more frames")
            print(f"\nLast few frames:")
            for i, f in enumerate(files[-5:], start=total_frames-5):
                print(f"  Frame {i}: {Path(f).name}")
        print(f"{'='*60}\n")
        sys.exit(0)
    
    # Multi-frame mode - require --multi if not listing frames
    if args.multi is None:
        parser.error("--multi is required (unless using --list-frames)")
    
    num_frames = args.multi
    start_idx = args.start
    
    # Get total frame count from dataset
    replay = TimedSensorReplay(f"{args.dataset}/lidar", autocast=LidarMessage.from_msg)
    total_frames = len(replay.files)
    
    print(f"Loading {num_frames} frames starting from frame {start_idx} (total available: {total_frames})...")
    try:
        frames = []
        for i in range(num_frames):
            frame_idx = start_idx + i
            if frame_idx >= total_frames:
                print(f"Warning: Frame {frame_idx} exceeds total frames ({total_frames}), stopping early")
                break
            frame = load_lidar_frame(args.dataset, frame_idx)
            frames.append(frame)
            # Show progress like dimos replay
            current_num = i + 1
            print(f"Loaded frame {current_num}/{num_frames} (frame index {frame_idx}/{total_frames-1})", end="\r")
        print()  # New line after progress
        visualize_multiple_frames(
            frames,
            start_idx,
            total_frames=total_frames,
            voxel_size=args.voxel_size,
            splice_method=args.splice_method,
            shrink=args.shrink,
        )
    except Exception as e:
        print(f"Error loading frames: {e}")
        print(f"\nTry --list-frames to see available frame indices")
        sys.exit(1)


if __name__ == "__main__":
    main()

