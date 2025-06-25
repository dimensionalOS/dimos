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

from dimos.perception.pointcloud.utils import visualize_clustered_point_clouds, visualize_voxel_grid
from dimos.perception.manip_aio_processer import ManipulationProcessor
from dimos.perception.pointcloud.utils import (
    load_camera_matrix_from_yaml,
    visualize_pcd,
    combine_object_pointclouds,
)
from dimos.utils.logging_config import setup_logger

# Import ContactGraspNet visualization
from dimos.models.manipulation.contact_graspnet_pytorch.contact_graspnet_pytorch.visualization_utils_o3d import (
    visualize_grasps,
)
from dimos.perception.grasp_generation.utils import parse_contactgraspnet_results

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


def run_processor(color_img, depth_img, intrinsics):
    """Run processor and collect results."""
    # Create processor with ContactGraspNet enabled
    processor = ManipulationProcessor(
        camera_intrinsics=intrinsics,
        enable_grasp_generation=True,
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

    # Print ContactGraspNet grasp summary
    if "grasps" in results and results["grasps"]:
        grasp_data = results["grasps"]
        if isinstance(grasp_data, dict):
            pred_grasps = grasp_data.get("pred_grasps_cam", {})
            scores = grasp_data.get("scores", {})

            total_grasps = 0
            best_score = 0
            for obj_id, obj_grasps in pred_grasps.items():
                num_grasps = len(obj_grasps) if hasattr(obj_grasps, "__len__") else 0
                total_grasps += num_grasps

                if obj_id in scores and len(scores[obj_id]) > 0:
                    obj_best_score = max(scores[obj_id])
                    if obj_best_score > best_score:
                        best_score = obj_best_score

            print(f"   ContactGraspNet grasps: {total_grasps} total (best score: {best_score:.3f})")
        else:
            print("   ContactGraspNet grasps: Invalid format")
    else:
        print("   ContactGraspNet grasps: None generated")

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

    # 3D ContactGraspNet visualization (if enabled)
    if "grasps" in results and results["grasps"] and "full_pointcloud" in results:
        grasp_data = results["grasps"]
        full_pcd = results["full_pointcloud"]

        if isinstance(grasp_data, dict) and full_pcd is not None:
            try:
                # Extract ContactGraspNet data
                pred_grasps_cam = grasp_data.get("pred_grasps_cam", {})
                scores = grasp_data.get("scores", {})
                contact_pts = grasp_data.get("contact_pts", {})
                gripper_openings = grasp_data.get("gripper_openings", {})

                # =====================================================================
                # =                    PARSE CONTACTGRASPNET RESULTS                  =
                # =                    ===========================                    =
                # =        Converting raw grasp data into clean dictionary format    =
                # =====================================================================
                parsed_grasps = parse_contactgraspnet_results(
                    pred_grasps_cam, scores, contact_pts, gripper_openings
                )

                # Check if we have valid grasp data
                total_grasps = (
                    sum(len(grasps) for grasps in pred_grasps_cam.values())
                    if pred_grasps_cam
                    else 0
                )

                if total_grasps > 0:
                    logger.info(f"Visualizing {total_grasps} ContactGraspNet grasps in 3D")

                    # Use ContactGraspNet's native visualization - pass dictionaries directly
                    visualize_grasps(
                        full_pcd,
                        pred_grasps_cam,  # Pass dictionary directly
                        scores,  # Pass dictionary directly
                        gripper_openings=gripper_openings,
                    )
                else:
                    logger.info("No valid grasps to visualize")

            except Exception as e:
                logger.error(f"Error in ContactGraspNet visualization: {e}")
                logger.info("Skipping 3D grasp visualization")
    else:
        logger.info(
            "ContactGraspNet grasp generation disabled or no results - skipping 3D grasp visualization"
        )

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

    # Visualize all objects point clouds if available
    if "all_objects" in results and results["all_objects"]:
        all_objects = results["all_objects"]

        if all_objects:
            # Extract point clouds
            point_clouds = [obj["point_cloud"] for obj in all_objects]
            colors = [obj["color"] for obj in all_objects]
            # Use utility function to combine point clouds
            combined_pcd = combine_object_pointclouds(point_clouds, colors)

            if len(np.asarray(combined_pcd.points)) > 0:
                visualize_pcd(
                    combined_pcd,
                    window_name="All Objects Point Cloud",
                    point_size=3.0,
                    show_coordinate_frame=True,
                )

    # Visualize misc/background clusters if available
    if "misc_clusters" in results and results["misc_clusters"]:
        misc_clusters = results["misc_clusters"]
        cluster_count = len(misc_clusters)
        total_misc_points = sum(len(np.asarray(cluster.points)) for cluster in misc_clusters)
        print(
            f"Visualizing {cluster_count} misc/background clusters with {total_misc_points} total points"
        )

        try:
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

    # Visualize voxel grid separately
    if "misc_voxel_grid" in results and results["misc_voxel_grid"] is not None:
        misc_voxel_grid = results["misc_voxel_grid"]
        misc_clusters = results.get("misc_clusters", [])

        voxel_count = len(misc_voxel_grid.get_voxels())
        print(f"Visualizing voxel grid with {voxel_count} voxels")

        try:
            visualize_voxel_grid(
                misc_voxel_grid,
                window_name="Misc/Background Voxel Grid",
                show_coordinate_frame=True,
            )
        except (KeyboardInterrupt, EOFError):
            print("\nSkipping voxel grid visualization")
        except Exception as e:
            print(f"Error in voxel grid visualization: {e}")
    else:
        print("No voxel grid available for visualization")


if __name__ == "__main__":
    main()
