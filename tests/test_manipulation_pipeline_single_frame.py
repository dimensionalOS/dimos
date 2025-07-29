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

"""Test manipulation processor with direct visualization and grasp data output."""

import os
import cv2
import numpy as np
import argparse
import matplotlib
import tests.test_header
from dimos.utils.data import get_data

# Try to use TkAgg backend for live display, fallback to Agg if not available
try:
    matplotlib.use("TkAgg")
except:
    try:
        matplotlib.use("Qt5Agg")
    except:
        matplotlib.use("Agg")  # Fallback to non-interactive
import matplotlib.pyplot as plt
import open3d as o3d

from dimos.perception.pointcloud.utils import visualize_clustered_point_clouds, visualize_voxel_grid
from dimos.manipulation.manip_aio_processer import ManipulationProcessor
from dimos.perception.pointcloud.utils import (
    load_camera_matrix_from_yaml,
    visualize_pcd,
    combine_object_pointclouds,
)
from dimos.utils.logging_config import setup_logger

from dimos.perception.grasp_generation.utils import visualize_grasps_3d, create_grasp_overlay

logger = setup_logger("test_pipeline_viz")


def load_first_frame(data_dir: str):
    """Load first RGB-D frame and camera intrinsics."""
    # Load images
    color_img = cv2.imread(os.path.join(data_dir, "color", "00000.png"))
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

    depth_img = cv2.imread(os.path.join(data_dir, "depth", "00000.png"), cv2.IMREAD_ANYDEPTH)
    if depth_img.dtype == np.uint16:
        depth_img = depth_img.astype(np.float32) / 1000.0
    # Load intrinsics
    camera_matrix = load_camera_matrix_from_yaml(os.path.join(data_dir, "color_camera_info.yaml"))
    intrinsics = [
        camera_matrix[0, 0],
        camera_matrix[1, 1],
        camera_matrix[0, 2],
        camera_matrix[1, 2],
    ]

    return color_img, depth_img, intrinsics


def create_point_cloud(color_img, depth_img, intrinsics):
    """Create Open3D point cloud."""
    fx, fy, cx, cy = intrinsics
    height, width = depth_img.shape

    o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    color_o3d = o3d.geometry.Image(color_img)
    depth_o3d = o3d.geometry.Image((depth_img * 1000).astype(np.uint16))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d, depth_scale=1000.0, convert_rgb_to_intensity=False
    )

    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsics)


def run_processor(color_img, depth_img, intrinsics, grasp_server_url=None):
    """Run processor and collect results."""
    processor_kwargs = {
        "camera_intrinsics": intrinsics,
        "enable_grasp_generation": True,
        "enable_segmentation": True,
    }

    if grasp_server_url:
        processor_kwargs["grasp_server_url"] = grasp_server_url

    processor = ManipulationProcessor(**processor_kwargs)

    # Process frame without grasp generation
    results = processor.process_frame(color_img, depth_img, generate_grasps=False)

    # Run grasp generation separately
    grasps = processor.run_grasp_generation(results["all_objects"], results["full_pointcloud"])
    results["grasps"] = grasps
    results["grasp_overlay"] = create_grasp_overlay(color_img, grasps, intrinsics)

    processor.cleanup()
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default=get_data("rgbd_frames"))
    parser.add_argument("--wait-time", type=float, default=5.0)
    parser.add_argument(
        "--grasp-server-url",
        default="ws://18.224.39.74:8000/ws/grasp",
        help="WebSocket URL for Dimensional Grasp server",
    )
    args = parser.parse_args()

    # Load data
    color_img, depth_img, intrinsics = load_first_frame(args.data_dir)
    logger.info(f"Loaded images: color {color_img.shape}, depth {depth_img.shape}")

    # Run processor
    results = run_processor(color_img, depth_img, intrinsics, args.grasp_server_url)

    # Print results summary
    print(f"Processing time: {results.get('processing_time', 0):.3f}s")
    print(f"Detection objects: {len(results.get('detected_objects', []))}")
    print(f"All objects processed: {len(results.get('all_objects', []))}")

    # Print grasp summary
    grasp_data = results["grasps"]
    total_grasps = len(grasp_data) if isinstance(grasp_data, list) else 0
    best_score = max(grasp["score"] for grasp in grasp_data) if grasp_data else 0

    print(f"Grasps: {total_grasps} total (best score: {best_score:.3f})")

    # Create visualizations
    plot_configs = []
    if results["detection_viz"] is not None:
        plot_configs.append(("detection_viz", "Object Detection"))
    if results["segmentation_viz"] is not None:
        plot_configs.append(("segmentation_viz", "Semantic Segmentation"))
    if results["pointcloud_viz"] is not None:
        plot_configs.append(("pointcloud_viz", "All Objects Point Cloud"))
    if results["detected_pointcloud_viz"] is not None:
        plot_configs.append(("detected_pointcloud_viz", "Detection Objects Point Cloud"))
    if results["misc_pointcloud_viz"] is not None:
        plot_configs.append(("misc_pointcloud_viz", "Misc/Background Points"))
    if results["grasp_overlay"] is not None:
        plot_configs.append(("grasp_overlay", "Grasp Overlay"))

    # Create subplot layout
    num_plots = len(plot_configs)
    if num_plots <= 3:
        fig, axes = plt.subplots(1, num_plots, figsize=(6 * num_plots, 5))
    else:
        rows = 2
        cols = (num_plots + 1) // 2
        fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5 * rows))

    if num_plots == 1:
        axes = [axes]
    elif num_plots > 2:
        axes = axes.flatten()

    # Plot each result
    for i, (key, title) in enumerate(plot_configs):
        axes[i].imshow(results[key])
        axes[i].set_title(title)
        axes[i].axis("off")

    # Hide unused subplots
    if num_plots > 3:
        for i in range(num_plots, len(axes)):
            axes[i].axis("off")

    plt.tight_layout()
    plt.savefig("manipulation_results.png", dpi=150, bbox_inches="tight")
    plt.show(block=True)
    plt.close()

    point_clouds = [obj["point_cloud"] for obj in results["all_objects"]]
    colors = [obj["color"] for obj in results["all_objects"]]
    combined_pcd = combine_object_pointclouds(point_clouds, colors)

    # 3D Grasp visualization
    if grasp_data:
        # Convert grasp format to visualization format for 3D display
        viz_grasps = []
        for grasp in grasp_data:
            translation = grasp.get("translation", [0, 0, 0])
            rotation_matrix = np.array(grasp.get("rotation_matrix", np.eye(3).tolist()))
            score = grasp.get("score", 0.0)
            width = grasp.get("width", 0.08)

            viz_grasp = {
                "translation": translation,
                "rotation_matrix": rotation_matrix,
                "width": width,
                "score": score,
            }
            viz_grasps.append(viz_grasp)

        # Use unified 3D visualization
        visualize_grasps_3d(combined_pcd, viz_grasps)

    # Visualize full point cloud
    visualize_pcd(
        results["full_pointcloud"],
        window_name="Full Scene Point Cloud",
        point_size=2.0,
        show_coordinate_frame=True,
    )

    # Visualize all objects point cloud
    visualize_pcd(
        combined_pcd,
        window_name="All Objects Point Cloud",
        point_size=3.0,
        show_coordinate_frame=True,
    )

    # Visualize misc clusters
    visualize_clustered_point_clouds(
        results["misc_clusters"],
        window_name="Misc/Background Clusters (DBSCAN)",
        point_size=3.0,
        show_coordinate_frame=True,
    )

    # Visualize voxel grid
    visualize_voxel_grid(
        results["misc_voxel_grid"],
        window_name="Misc/Background Voxel Grid",
        show_coordinate_frame=True,
    )


if __name__ == "__main__":
    main()
