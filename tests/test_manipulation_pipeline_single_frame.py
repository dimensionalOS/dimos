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
import sys
import cv2
import numpy as np
import time
import argparse
import matplotlib

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
from typing import Dict, List

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.perception.manip_aio_processer import ManipulationProcessor
from dimos.perception.grasp_generation.utils import visualize_grasps_3d
from dimos.perception.pointcloud.utils import load_camera_matrix_from_yaml, visualize_pcd
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_pipeline_viz")


def load_first_frame(data_dir: str):
    """Load first RGB-D frame and camera intrinsics."""
    # Load images
    color_img = cv2.imread(os.path.join(data_dir, "color", "00300.png"))
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

    depth_img = cv2.imread(os.path.join(data_dir, "depth", "00300.png"), cv2.IMREAD_ANYDEPTH)
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


def run_processor(color_img, depth_img, intrinsics):
    """Run processor and collect results."""
    # Create processor
    processor = ManipulationProcessor(
        camera_intrinsics=intrinsics,
        grasp_server_url="ws://10.0.0.125:8000/ws/grasp",
        enable_grasp_generation=False,
        enable_segmentation=True,
        segmentation_model="FastSAM-x.pt",
    )

    # Process single frame directly
    results = processor.process_frame(color_img, depth_img)

    # Debug: print available results
    print(f"Available results: {list(results.keys())}")

    processor.cleanup()

    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default="assets/rgbd_data")
    parser.add_argument("--wait-time", type=float, default=5.0)
    args = parser.parse_args()

    # Load data
    color_img, depth_img, intrinsics = load_first_frame(args.data_dir)
    logger.info(f"Loaded images: color {color_img.shape}, depth {depth_img.shape}")

    # Run processor
    results = run_processor(color_img, depth_img, intrinsics)

    # Debug: Print what we received
    print(f"\n✅ Processor Results:")
    print(f"   Available results: {list(results.keys())}")
    print(f"   Processing time: {results.get('processing_time', 0):.3f}s")

    # Show timing breakdown if available
    if "timing_breakdown" in results:
        breakdown = results["timing_breakdown"]
        print(f"   Timing breakdown:")
        print(f"     - Detection: {breakdown.get('detection', 0):.3f}s")
        print(f"     - Segmentation: {breakdown.get('segmentation', 0):.3f}s")
        print(f"     - Point cloud: {breakdown.get('pointcloud', 0):.3f}s")
        print(f"     - Misc extraction: {breakdown.get('misc_extraction', 0):.3f}s")

    # Print object information
    detected_count = len(results.get("detected_objects", []))
    all_count = len(results.get("all_objects", []))

    print(f"   Detection objects: {detected_count}")
    print(f"   All objects processed: {all_count}")

    # Print misc clusters information
    if "misc_clusters" in results and results["misc_clusters"]:
        cluster_count = len(results["misc_clusters"])
        total_misc_points = sum(
            len(np.asarray(cluster.points)) for cluster in results["misc_clusters"]
        )
        print(f"   Misc clusters: {cluster_count} clusters with {total_misc_points} total points")
    else:
        print(f"   Misc clusters: None")

    # Print grasp summary
    if "grasps" in results and results["grasps"]:
        total_grasps = 0
        best_score = 0
        for grasp in results["grasps"]:
            score = grasp.get("score", 0)
            if score > best_score:
                best_score = score
            total_grasps += 1
        print(f"   Grasps generated: {total_grasps} (best score: {best_score:.3f})")
    else:
        print("   Grasps: None generated")

    # Determine number of subplots based on what results we have
    num_plots = 0
    plot_configs = []

    if "detection_viz" in results and results["detection_viz"] is not None:
        plot_configs.append(("detection_viz", "Object Detection"))
        num_plots += 1

    if "segmentation_viz" in results and results["segmentation_viz"] is not None:
        plot_configs.append(("segmentation_viz", "Semantic Segmentation"))
        num_plots += 1

    if "pointcloud_viz" in results and results["pointcloud_viz"] is not None:
        plot_configs.append(("pointcloud_viz", "All Objects Point Cloud"))
        num_plots += 1

    if "detected_pointcloud_viz" in results and results["detected_pointcloud_viz"] is not None:
        plot_configs.append(("detected_pointcloud_viz", "Detection Objects Point Cloud"))
        num_plots += 1

    if "misc_pointcloud_viz" in results and results["misc_pointcloud_viz"] is not None:
        plot_configs.append(("misc_pointcloud_viz", "Misc/Background Points"))
        num_plots += 1

    if "grasp_overlay" in results and results["grasp_overlay"] is not None:
        plot_configs.append(("grasp_overlay", "Grasp Overlay"))
        num_plots += 1

    if num_plots == 0:
        print("No visualization results to display")
        return

    # Create subplot layout
    if num_plots <= 3:
        fig, axes = plt.subplots(1, num_plots, figsize=(6 * num_plots, 5))
    else:
        rows = 2
        cols = (num_plots + 1) // 2
        fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5 * rows))

    # Ensure axes is always a list for consistent indexing
    if num_plots == 1:
        axes = [axes]
    elif num_plots > 2:
        axes = axes.flatten()

    # Plot each result
    for i, (key, title) in enumerate(plot_configs):
        axes[i].imshow(results[key])
        axes[i].set_title(title)
        axes[i].axis("off")

    # Hide unused subplots if any
    if num_plots > 3:
        for i in range(num_plots, len(axes)):
            axes[i].axis("off")

    plt.tight_layout()

    # Save and show the plot
    output_path = "manipulation_results.png"
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Results visualization saved to: {output_path}")

    # Show plot live as well
    plt.show(block=True)
    plt.close()

    # 3D visualization with grasps (if enabled)
    if "grasps" in results and results["grasps"]:
        pcd = create_point_cloud(color_img, depth_img, intrinsics)
        all_grasps = results["grasps"]

        if all_grasps:
            logger.info(f"Visualizing {len(all_grasps)} grasps in 3D")
            visualize_grasps_3d(pcd, all_grasps)
    else:
        logger.info("Grasp generation disabled - skipping 3D grasp visualization")

    # Visualize full point cloud if available
    if "full_pointcloud" in results and results["full_pointcloud"] is not None:
        full_pcd = results["full_pointcloud"]
        print(f"Visualizing full point cloud with {len(np.asarray(full_pcd.points))} points")

        try:
            visualize_pcd(
                full_pcd,
                window_name="Full Scene Point Cloud",
                point_size=2.0,
                show_coordinate_frame=True,
            )
        except (KeyboardInterrupt, EOFError):
            print("\nSkipping full point cloud visualization")
    else:
        print("No full point cloud available for visualization")

    # Visualize misc/background clusters if available
    if "misc_clusters" in results and results["misc_clusters"]:
        misc_clusters = results["misc_clusters"]
        cluster_count = len(misc_clusters)
        total_misc_points = sum(len(np.asarray(cluster.points)) for cluster in misc_clusters)
        print(
            f"Visualizing {cluster_count} misc/background clusters with {total_misc_points} total points"
        )

        try:
            from dimos.perception.pointcloud.utils import visualize_clustered_point_clouds

            visualize_clustered_point_clouds(
                misc_clusters,
                window_name="Misc/Background Clusters (DBSCAN)",
                point_size=3.0,
                show_coordinate_frame=True,
            )
        except (KeyboardInterrupt, EOFError):
            print("\nSkipping misc clusters visualization")
    else:
        print("No misc clusters available for visualization")


if __name__ == "__main__":
    main()
