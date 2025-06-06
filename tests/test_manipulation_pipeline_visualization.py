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

"""Test manipulation pipeline with direct visualization and grasp data output."""

import os
import sys
import cv2
import numpy as np
import time
import argparse
import matplotlib.pyplot as plt
import open3d as o3d
from typing import Dict, List
import threading
from reactivex import Observable, operators as ops
from reactivex.subject import Subject

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.perception.manip_aio_pipeline import ManipulationPipeline
from dimos.perception.grasp_generation.utils import visualize_grasps_3d
from dimos.perception.pointcloud.utils import load_camera_matrix_from_yaml
from dimos.utils.logging_config import setup_logger

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


def run_pipeline(color_img, depth_img, intrinsics, wait_time=5.0):
    """Run pipeline and collect results."""
    # Create pipeline
    pipeline = ManipulationPipeline(
        camera_intrinsics=intrinsics,
        grasp_server_url="ws://10.0.0.125:8000/ws/grasp",
        enable_grasp_generation=True,
    )

    # Create single-frame stream
    subject = Subject()
    streams = pipeline.create_streams(subject)

    # Debug: print available streams
    print(f"Available streams: {list(streams.keys())}")

    # Collect results
    results = {}

    def collect(key):
        def on_next(value):
            results[key] = value
            logger.info(f"Received {key}")

        return on_next

    # Subscribe to streams
    for key, stream in streams.items():
        if stream:
            stream.pipe(ops.take(1)).subscribe(on_next=collect(key))

    # Send frame
    threading.Timer(
        0.5,
        lambda: subject.on_next({"rgb": color_img, "depth": depth_img, "timestamp": time.time()}),
    ).start()

    # Wait for results
    time.sleep(wait_time)

    # If grasp generation is enabled, also check for latest grasps
    if pipeline.latest_grasps:
        results["grasps"] = pipeline.latest_grasps
        logger.info(f"Retrieved latest grasps: {len(pipeline.latest_grasps)} grasps")

    pipeline.cleanup()

    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default="assets/rgbd_data")
    parser.add_argument("--wait-time", type=float, default=5.0)
    args = parser.parse_args()

    # Load data
    color_img, depth_img, intrinsics = load_first_frame(args.data_dir)
    logger.info(f"Loaded images: color {color_img.shape}, depth {depth_img.shape}")

    # Run pipeline
    results = run_pipeline(color_img, depth_img, intrinsics, args.wait_time)

    # Debug: Print what we received
    print(f"\n✅ Pipeline Results:")
    print(f"   Available streams: {list(results.keys())}")

    if "filtered_objects" in results and results["filtered_objects"]:
        print(f"   Objects detected: {len(results['filtered_objects'])}")

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

    # Visualize 2D results
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    if "detection_viz" in results and results["detection_viz"] is not None:
        axes[0].imshow(results["detection_viz"])
        axes[0].set_title("Object Detection")
    axes[0].axis("off")

    if "pointcloud_viz" in results and results["pointcloud_viz"] is not None:
        axes[1].imshow(results["pointcloud_viz"])
        axes[1].set_title("Point Cloud Overlay")
    axes[1].axis("off")

    plt.tight_layout()
    plt.show()

    # 3D visualization with grasps
    if "grasps" in results and results["grasps"]:
        pcd = create_point_cloud(color_img, depth_img, intrinsics)
        all_grasps = results["grasps"]

        if all_grasps:
            logger.info(f"Visualizing {len(all_grasps)} grasps in 3D")
            visualize_grasps_3d(pcd, all_grasps)


if __name__ == "__main__":
    main()
