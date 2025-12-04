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
Sequential manipulation processor for single-frame processing without reactive streams.
"""

import time
from typing import Any

import cv2
import numpy as np

from dimos.perception.common.utils import (
    colorize_depth,
    combine_object_data,
    detection_results_to_object_data,
)
from dimos.perception.detection2d.detic_2d_det import (  # type: ignore[import-untyped]
    Detic2DDetector,
)
from dimos.perception.grasp_generation.grasp_generation import HostedGraspGenerator
from dimos.perception.grasp_generation.utils import create_grasp_overlay
from dimos.perception.pointcloud.pointcloud_filtering import PointcloudFiltering
from dimos.perception.pointcloud.utils import (
    create_point_cloud_overlay_visualization,
    extract_and_cluster_misc_points,
    overlay_point_clouds_on_image,
)
from dimos.perception.segmentation.sam_2d_seg import Sam2DSegmenter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ManipulationProcessor:
    """
    Sequential manipulation processor for single-frame processing.

    Processes RGB-D frames through object detection, point cloud filtering,
    and grasp generation in a single thread without reactive streams.
    """

    def __init__(
        self,
        camera_intrinsics: list[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        max_objects: int = 20,
        vocabulary: str | None = None,
        enable_grasp_generation: bool = False,
        grasp_server_url: str | None = None,  # Required when enable_grasp_generation=True
        enable_segmentation: bool = True,
    ) -> None:
        """
        Initialize the manipulation processor.

        Args:
            camera_intrinsics: [fx, fy, cx, cy] camera parameters
            min_confidence: Minimum detection confidence threshold
            max_objects: Maximum number of objects to process
            vocabulary: Optional vocabulary for Detic detector
            enable_grasp_generation: Whether to enable grasp generation
            grasp_server_url: WebSocket URL for Dimensional Grasp server (required when enable_grasp_generation=True)
            enable_segmentation: Whether to enable semantic segmentation
            segmentation_model: Segmentation model to use (SAM 2 or FastSAM)
        """
        self.camera_intrinsics = camera_intrinsics
        self.min_confidence = min_confidence
        self.max_objects = max_objects
        self.enable_grasp_generation = enable_grasp_generation
        self.grasp_server_url = grasp_server_url
        self.enable_segmentation = enable_segmentation

        # Validate grasp generation requirements
        if enable_grasp_generation and not grasp_server_url:
            raise ValueError("grasp_server_url is required when enable_grasp_generation=True")

        # Initialize object detector
        self.detector = Detic2DDetector(vocabulary=vocabulary, threshold=min_confidence)

        # Initialize point cloud processor
        self.pointcloud_filter = PointcloudFiltering(
            color_intrinsics=camera_intrinsics,
            depth_intrinsics=camera_intrinsics,  # ZED uses same intrinsics
            max_num_objects=max_objects,
        )

        # Initialize semantic segmentation
        self.segmenter = None
        if self.enable_segmentation:
            self.segmenter = Sam2DSegmenter(
                use_tracker=False,  # Disable tracker for simple segmentation
                use_analyzer=False,  # Disable analyzer for simple segmentation
            )

        # Initialize grasp generator if enabled
        self.grasp_generator = None
        if self.enable_grasp_generation:
            try:
                self.grasp_generator = HostedGraspGenerator(server_url=grasp_server_url)  # type: ignore[arg-type]
                logger.info("Hosted grasp generator initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize hosted grasp generator: {e}")
                self.grasp_generator = None
                self.enable_grasp_generation = False

        logger.info(
            f"Initialized ManipulationProcessor with confidence={min_confidence}, "
            f"grasp_generation={enable_grasp_generation}"
        )

    def process_frame(
        self,
        rgb_image: np.ndarray,  # type: ignore[type-arg]
        depth_image: np.ndarray,  # type: ignore[type-arg]
        generate_grasps: bool | None = None,
    ) -> dict[str, Any]:
        """
        Process a single RGB-D frame through the complete pipeline.

        Args:
            rgb_image: RGB image (H, W, 3)
            depth_image: Depth image (H, W) in meters
            generate_grasps: Override grasp generation setting for this frame

        Returns:
            Dictionary containing:
                - detection_viz: Visualization of object detection
                - pointcloud_viz: Visualization of point cloud overlay
                - segmentation_viz: Visualization of semantic segmentation (if enabled)
                - detection2d_objects: Raw detection results as ObjectData
                - segmentation2d_objects: Raw segmentation results as ObjectData (if enabled)
                - detected_objects: Detection (Object Detection) objects with point clouds filtered
                - all_objects: Combined objects with intelligent duplicate removal
                - full_pointcloud: Complete scene point cloud (if point cloud processing enabled)
                - misc_clusters: List of clustered background/miscellaneous point clouds (DBSCAN)
                - misc_voxel_grid: Open3D voxel grid approximating all misc/background points
                - misc_pointcloud_viz: Visualization of misc/background cluster overlay
                - grasps: Grasp results (list of dictionaries, if enabled)
                - grasp_overlay: Grasp visualization overlay (if enabled)
                - processing_time: Total processing time
        """
        start_time = time.time()
        results = {}

        try:
            # Step 1: Object Detection
            step_start = time.time()
            detection_results = self.run_object_detection(rgb_image)
            results["detection2d_objects"] = detection_results.get("objects", [])
            results["detection_viz"] = detection_results.get("viz_frame")
            detection_time = time.time() - step_start

            # Step 2: Semantic Segmentation (if enabled)
            segmentation_time = 0
            if self.enable_segmentation:
                step_start = time.time()
                segmentation_results = self.run_segmentation(rgb_image)
                results["segmentation2d_objects"] = segmentation_results.get("objects", [])
                results["segmentation_viz"] = segmentation_results.get("viz_frame")
                segmentation_time = time.time() - step_start  # type: ignore[assignment]

            # Step 3: Point Cloud Processing
            pointcloud_time = 0
            detection2d_objects = results.get("detection2d_objects", [])
            segmentation2d_objects = results.get("segmentation2d_objects", [])

            # Process detection objects if available
            detected_objects = []
            if detection2d_objects:
                step_start = time.time()
                detected_objects = self.run_pointcloud_filtering(
                    rgb_image, depth_image, detection2d_objects
                )
                pointcloud_time += time.time() - step_start  # type: ignore[assignment]

            # Process segmentation objects if available
            segmentation_filtered_objects = []
            if segmentation2d_objects:
                step_start = time.time()
                segmentation_filtered_objects = self.run_pointcloud_filtering(
                    rgb_image, depth_image, segmentation2d_objects
                )
                pointcloud_time += time.time() - step_start  # type: ignore[assignment]

            # Combine all objects using intelligent duplicate removal
            all_objects = combine_object_data(
                detected_objects,  # type: ignore[arg-type]
                segmentation_filtered_objects,  # type: ignore[arg-type]
                overlap_threshold=0.8,
            )

            # Get full point cloud
            full_pcd = self.pointcloud_filter.get_full_point_cloud()

            # Extract misc/background points and create voxel grid
            misc_start = time.time()
            misc_clusters, misc_voxel_grid = extract_and_cluster_misc_points(
                full_pcd,
                all_objects,  # type: ignore[arg-type]
                eps=0.03,
                min_points=100,
                enable_filtering=True,
                voxel_size=0.02,
            )
            misc_time = time.time() - misc_start

            # Store results
            results.update(
                {
                    "detected_objects": detected_objects,
                    "all_objects": all_objects,
                    "full_pointcloud": full_pcd,
                    "misc_clusters": misc_clusters,
                    "misc_voxel_grid": misc_voxel_grid,
                }
            )

            # Create point cloud visualizations
            base_image = colorize_depth(depth_image, max_depth=10.0)

            # Create visualizations
            results["pointcloud_viz"] = (
                create_point_cloud_overlay_visualization(
                    base_image=base_image,  # type: ignore[arg-type]
                    objects=all_objects,  # type: ignore[arg-type]
                    intrinsics=self.camera_intrinsics,  # type: ignore[arg-type]
                )
                if all_objects
                else base_image
            )

            results["detected_pointcloud_viz"] = (
                create_point_cloud_overlay_visualization(
                    base_image=base_image,  # type: ignore[arg-type]
                    objects=detected_objects,
                    intrinsics=self.camera_intrinsics,  # type: ignore[arg-type]
                )
                if detected_objects
                else base_image
            )

            if misc_clusters:
                # Generate consistent colors for clusters
                cluster_colors = [
                    tuple((np.random.RandomState(i + 100).rand(3) * 255).astype(int))
                    for i in range(len(misc_clusters))
                ]
                results["misc_pointcloud_viz"] = overlay_point_clouds_on_image(
                    base_image=base_image,  # type: ignore[arg-type]
                    point_clouds=misc_clusters,
                    camera_intrinsics=self.camera_intrinsics,
                    colors=cluster_colors,
                    point_size=2,
                    alpha=0.6,
                )
            else:
                results["misc_pointcloud_viz"] = base_image

            # Step 4: Grasp Generation (if enabled)
            should_generate_grasps = (
                generate_grasps if generate_grasps is not None else self.enable_grasp_generation
            )

            if should_generate_grasps and all_objects and full_pcd:
                grasps = self.run_grasp_generation(all_objects, full_pcd)  # type: ignore[arg-type]
                results["grasps"] = grasps
                if grasps:
                    results["grasp_overlay"] = create_grasp_overlay(
                        rgb_image, grasps, self.camera_intrinsics
                    )

        except Exception as e:
            logger.error(f"Error processing frame: {e}")
            results["error"] = str(e)

        # Add timing information
        total_time = time.time() - start_time
        results.update(
            {
                "processing_time": total_time,
                "timing_breakdown": {
                    "detection": detection_time if "detection_time" in locals() else 0,
                    "segmentation": segmentation_time if "segmentation_time" in locals() else 0,
                    "pointcloud": pointcloud_time if "pointcloud_time" in locals() else 0,
                    "misc_extraction": misc_time if "misc_time" in locals() else 0,
                    "total": total_time,
                },
            }
        )

        return results

    def run_object_detection(self, rgb_image: np.ndarray) -> dict[str, Any]:  # type: ignore[type-arg]
        """Run object detection on RGB image."""
        try:
            # Convert RGB to BGR for Detic detector
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # Use process_image method from Detic detector
            bboxes, track_ids, class_ids, confidences, names, masks = self.detector.process_image(
                bgr_image
            )

            # Convert to ObjectData format using utility function
            objects = detection_results_to_object_data(
                bboxes=bboxes,
                track_ids=track_ids,
                class_ids=class_ids,
                confidences=confidences,
                names=names,
                masks=masks,
                source="detection",
            )

            # Create visualization using detector's built-in method
            viz_frame = self.detector.visualize_results(
                rgb_image, bboxes, track_ids, class_ids, confidences, names
            )

            return {"objects": objects, "viz_frame": viz_frame}

        except Exception as e:
            logger.error(f"Object detection failed: {e}")
            return {"objects": [], "viz_frame": rgb_image.copy()}

    def run_pointcloud_filtering(
        self,
        rgb_image: np.ndarray,  # type: ignore[type-arg]
        depth_image: np.ndarray,  # type: ignore[type-arg]
        objects: list[dict],  # type: ignore[type-arg]
    ) -> list[dict]:  # type: ignore[type-arg]
        """Run point cloud filtering on detected objects."""
        try:
            filtered_objects = self.pointcloud_filter.process_images(
                rgb_image,
                depth_image,
                objects,  # type: ignore[arg-type]
            )
            return filtered_objects if filtered_objects else []  # type: ignore[return-value]
        except Exception as e:
            logger.error(f"Point cloud filtering failed: {e}")
            return []

    def run_segmentation(self, rgb_image: np.ndarray) -> dict[str, Any]:  # type: ignore[type-arg]
        """Run semantic segmentation on RGB image."""
        if not self.segmenter:
            return {"objects": [], "viz_frame": rgb_image.copy()}

        try:
            # Convert RGB to BGR for segmenter
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # Get segmentation results
            masks, bboxes, track_ids, probs, names = self.segmenter.process_image(bgr_image)  # type: ignore[no-untyped-call]

            # Convert to ObjectData format using utility function
            objects = detection_results_to_object_data(
                bboxes=bboxes,
                track_ids=track_ids,
                class_ids=list(range(len(bboxes))),  # Use indices as class IDs for segmentation
                confidences=probs,
                names=names,
                masks=masks,
                source="segmentation",
            )

            # Create visualization
            if masks:
                viz_bgr = self.segmenter.visualize_results(
                    bgr_image, masks, bboxes, track_ids, probs, names
                )
                # Convert back to RGB
                viz_frame = cv2.cvtColor(viz_bgr, cv2.COLOR_BGR2RGB)
            else:
                viz_frame = rgb_image.copy()

            return {"objects": objects, "viz_frame": viz_frame}

        except Exception as e:
            logger.error(f"Segmentation failed: {e}")
            return {"objects": [], "viz_frame": rgb_image.copy()}

    def run_grasp_generation(self, filtered_objects: list[dict], full_pcd) -> list[dict] | None:  # type: ignore[no-untyped-def, type-arg]
        """Run grasp generation using the configured generator."""
        if not self.grasp_generator:
            logger.warning("Grasp generation requested but no generator available")
            return None

        try:
            # Generate grasps using the configured generator
            grasps = self.grasp_generator.generate_grasps_from_objects(filtered_objects, full_pcd)  # type: ignore[arg-type]

            # Return parsed results directly (list of grasp dictionaries)
            return grasps

        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

    def cleanup(self) -> None:
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        if hasattr(self.pointcloud_filter, "cleanup"):
            self.pointcloud_filter.cleanup()
        if self.segmenter and hasattr(self.segmenter, "cleanup"):
            self.segmenter.cleanup()
        if self.grasp_generator and hasattr(self.grasp_generator, "cleanup"):
            self.grasp_generator.cleanup()
        logger.info("ManipulationProcessor cleaned up")
