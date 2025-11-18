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

import copy
import time
from typing import Any

import cv2
import numpy as np
import open3d as o3d

from dimos.perception.common.utils import (
    colorize_depth,
    combine_object_data,
    detection_results_to_object_data,
)
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
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

logger = setup_logger("dimos.perception.manip_aio_processor")


class ManipulationProcessor:
    """
    Sequential manipulation processor for single-frame processing.

    Processes RGB-D frames through object detection, point cloud filtering,
    and grasp generation in a single thread without reactive streams.
    """

    def __init__(
        self,
        camera_configs: list[dict],  # Multiple Cameras
        min_confidence: float = 0.6,
        max_objects: int = 20,
        vocabulary: str | None = None,
        enable_grasp_generation: bool = False,
        grasp_server_url: str | None = None,  # Required when enable_grasp_generation=True
        enable_segmentation: bool = True,
    ) -> None:
        """
        Initialize the dual-camera manipulation.

        Args:
            camera_configs: List of camera configurations, each containing:
                - intrinsics: [fx, fy, cx, cy]
                - extrinsics: 4x4 trasnfromation matrix (camera to world)
                - camera_id: unqiue identifier

            min_confidence: Minimum detection confidence threshold
            max_objects: Maximum number of objects to process
            vocabulary: Optional vocabulary for Detic detector
            enable_grasp_generation: Whether to enable grasp generation
            grasp_server_url: WebSocket URL for Dimensional Grasp server (required when enable_grasp_generation=True)
            enable_segmentation: Whether to enable semantic segmentation
            segmentation_model: Segmentation model to use (SAM 2 or FastSAM)
        """
        self.camera_configs = camera_configs
        self.num_cameras = len(camera_configs)
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

        # Initialize point cloud processor for EACH camera
        self.pointcloud_filters = []
        for config in camera_configs:
            self.pointcloud_filters.append(
                PointcloudFiltering(
                    color_intrinsics=config["intrinsics"],
                    depth_intrinsics=config["intrinsics"],
                    max_num_objects=max_objects,
                )
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
                self.grasp_generator = HostedGraspGenerator(server_url=grasp_server_url)
                logger.info("Hosted grasp generator initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize hosted grasp generator: {e}")
                self.grasp_generator = None
                self.enable_grasp_generation = False

        logger.info(
            f"Initialized ManipulationProcessor with {self.num_cameras} cameras, "
            f"confidence={min_confidence}, grasp_generation={enable_grasp_generation}"
        )

    def process_frame(
        self,
        rgb_images: list[np.ndarray],
        depth_images: list[np.ndarray],
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
            detection_time = 0
            segmentation_time = 0

            per_camera_detection_results = []
            per_camera_segmentation_results = []

            for cam_idx in range(self.num_cameras):
                rgb = rgb_images[cam_idx]

                # Step 1: Object Detection

                step_start = time.time()
                detection_results = self.run_object_detection(rgb)
                detection_time += time.time() - step_start
                per_camera_detection_results.append(detection_results)

                # Step 2: Semantic Segmentation (if enabled)
                if self.enable_segmentation:
                    step_start = time.time()
                    segmentation_results = self.run_segmentation(rgb)
                    segmentation_time += time.time() - step_start
                else:
                    segmentation_results = {"objects": [], "viz_frame": rgb.copy()}
                per_camera_segmentation_results.append(segmentation_results)

            # Step 3: Point Cloud Processing
            pointcloud_time = 0
            per_camera_all_objects = []
            per_camera_full_pcds = []
            per_camera_data = []

            for cam_idx in range(self.num_cameras):
                rgb = rgb_images[cam_idx]
                depth = depth_images[cam_idx]

                detection2d_objects = per_camera_detection_results[cam_idx].get("objects", [])
                segmentation2d_objects = per_camera_segmentation_results[cam_idx].get("objects", [])

                # Process detection objects if available
                detected_objects = []
                if detection2d_objects:
                    step_start = time.time()
                    detected_objects = self.run_pointcloud_filtering(
                        rgb, depth, detection2d_objects, cam_idx
                    )
                    pointcloud_time += time.time() - step_start

                # Process segmentation objects if available
                segmentation_filtered_objects = []
                if segmentation2d_objects:
                    step_start = time.time()
                    segmentation_filtered_objects = self.run_pointcloud_filtering(
                        rgb, depth, segmentation2d_objects, cam_idx
                    )
                    pointcloud_time += time.time() - step_start

                # Combine all objects using intelligent duplicate removal
                camera_all_objects = combine_object_data(
                    detected_objects, segmentation_filtered_objects, overlap_threshold=0.8
                )

                # Get full point cloud
                camera_full_pcd = self.pointcloud_filters[cam_idx].get_full_point_cloud()

                # Store per-camera data
                per_camera_data.append(
                    {
                        "camera_id": self.camera_configs[cam_idx]["camera_id"],
                        "camera_idx": cam_idx,
                        "rgb": rgb,
                        "depth": depth,
                        "detection2d_objects": detection2d_objects,
                        "segmentation2d_objects": segmentation2d_objects,
                        "detected_objects": detected_objects,
                        "segmentation_filtered_objects": segmentation_filtered_objects,
                        "all_objects": camera_all_objects,
                        "full_pointcloud": camera_full_pcd,
                        "detection_viz": per_camera_detection_results[cam_idx].get("viz_frame"),
                        "segmentation_viz": per_camera_segmentation_results[cam_idx].get(
                            "viz_frame"
                        ),
                    }
                )

                per_camera_all_objects.append(camera_all_objects)
                per_camera_full_pcds.append(camera_full_pcd)

            # Transform each camera's point cloud to world coordinates
            world_full_pcds = []
            world_all_objects = []

            for cam_idx, cam_data in enumerate(per_camera_data):
                extrinsics = self.camera_configs[cam_idx]["extrinsics"]

                # Transform full point cloud to world
                world_full_pcd = self.transform_pointcloud_to_world(
                    cam_data["full_pointcloud"], extrinsics
                )
                world_full_pcds.append(world_full_pcd)

                # Transform each object's point cloud to world
                world_objects = []
                for obj in cam_data["all_objects"]:
                    if obj.get("point_cloud") is not None:
                        obj_world = obj.copy()
                        obj_world["point_cloud"] = self.transform_pointcloud_to_world(
                            obj["point_cloud"], extrinsics
                        )
                        obj_world["camera_id"] = cam_data["camera_id"]
                        world_objects.append(obj_world)
                    else:
                        obj_world = obj.copy()
                        obj_world["camera_id"] = cam_data["camera_id"]
                        world_objects.append(obj_world)

                world_all_objects.extend(world_objects)

                # Update camera data with world coordinates
                cam_data["world_full_pointcloud"] = world_full_pcd
                cam_data["world_objects"] = world_objects

            # Stitch all world point clouds together
            full_pcd = self.stitch_pointclouds(world_full_pcds)

            # Merge objects from all cameras (remove duplicates in 3D)
            all_objects = self.merge_multi_view_detections(world_all_objects)

            world_detected_only = []
            for cam_data in per_camera_data:
                for obj in cam_data["detected_objects"]:
                    if obj.get("point_cloud") is not None:
                        obj_world = obj.copy()
                        obj_world["point_cloud"] = self.transform_pointcloud_to_world(
                            obj["point_cloud"],
                            self.camera_configs[cam_data["camera_idx"]]["extrinsics"],
                        )
                        world_detected_only.append(obj_world)
            detected_objects = self.merge_multi_view_detections(world_detected_only)

            # Extract misc/background points and create voxel grid
            misc_start = time.time()
            misc_clusters, misc_voxel_grid = extract_and_cluster_misc_points(
                full_pcd,
                all_objects,
                eps=0.03,
                min_points=100,
                enable_filtering=True,
                voxel_size=0.02,
            )
            misc_time = time.time() - misc_start

            # Store results
            results.update(
                {
                    "detected_objects": detected_objects,  # Merged detected objects only
                    "all_objects": all_objects,  # All merged objects
                    "full_pointcloud": full_pcd,  # Stitched world point cloud
                    "misc_clusters": misc_clusters,
                    "misc_voxel_grid": misc_voxel_grid,
                    "per_camera_data": per_camera_data,  # Keep per-camera data
                }
            )

            # Create point cloud visualizations
            base_image = colorize_depth(depth_images[0], max_depth=10.0)

            # Create visualizations (project world objects back to first camera)
            results["pointcloud_viz"] = (
                create_point_cloud_overlay_visualization(
                    base_image=base_image,
                    objects=all_objects,
                    intrinsics=self.camera_configs[0]["intrinsics"],
                )
                if all_objects
                else base_image
            )

            results["detected_pointcloud_viz"] = (
                create_point_cloud_overlay_visualization(
                    base_image=base_image,
                    objects=detected_objects,
                    intrinsics=self.camera_configs[0]["intrinsics"],
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
                    base_image=base_image,
                    point_clouds=misc_clusters,
                    camera_intrinsics=self.camera_configs[0]["intrinsics"],
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
                grasps = self.run_grasp_generation(all_objects, full_pcd)
                results["grasps"] = grasps
                if grasps:
                    results["grasp_overlay"] = create_grasp_overlay(
                        rgb_images[0], grasps, self.camera_configs[0]["intrinsics"]
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

    def run_object_detection(self, rgb_image: np.ndarray) -> dict[str, Any]:
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
        self, rgb_image: np.ndarray, depth_image: np.ndarray, objects: list[dict], cam_idx: int = 0
    ) -> list[dict]:
        """Run point cloud filtering on detected objects."""
        try:
            filtered_objects = self.pointcloud_filters[cam_idx].process_images(
                rgb_image, depth_image, objects
            )
            return filtered_objects if filtered_objects else []
        except Exception as e:
            logger.error(f"Point cloud filtering failed: {e}")
            return []

    def run_segmentation(self, rgb_image: np.ndarray) -> dict[str, Any]:
        """Run semantic segmentation on RGB image."""
        if not self.segmenter:
            return {"objects": [], "viz_frame": rgb_image.copy()}

        try:
            # Convert RGB to BGR for segmenter
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # Get segmentation results
            masks, bboxes, track_ids, probs, names = self.segmenter.process_image(bgr_image)

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

    def run_grasp_generation(self, filtered_objects: list[dict], full_pcd) -> list[dict] | None:
        """Run grasp generation using the configured generator."""
        if not self.grasp_generator:
            logger.warning("Grasp generation requested but no generator available")
            return None

        try:
            # Generate grasps using the configured generator
            grasps = self.grasp_generator.generate_grasps_from_objects(filtered_objects, full_pcd)

            # Return parsed results directly (list of grasp dictionaries)
            return grasps

        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

    def transform_pointcloud_to_world(
        self, pointcloud: o3d.geometry.PointCloud, extrinsics: np.ndarray
    ) -> o3d.geometry.PointCloud:
        """Transform point cloud from camera frame to world frame."""
        if pointcloud is None or len(pointcloud.points) == 0:
            return o3d.geometry.PointCloud()

        transformed_pcd = copy.deepcopy(pointcloud)
        transformed_pcd.transform(extrinsics)
        return transformed_pcd

    def stitch_pointclouds(
        self, pointclouds: list[o3d.geometry.PointCloud]
    ) -> o3d.geometry.PointCloud:
        """
        Stitch multiple point clouds together with overlap removal.
        """
        if len(pointclouds) == 0:
            return o3d.geometry.PointCloud()

        if len(pointclouds) == 1:
            return pointclouds[0]

        # Combine all point clouds
        combined_pcd = o3d.geometry.PointCloud()
        for pcd in pointclouds:
            if pcd is not None and len(pcd.points) > 0:
                combined_pcd += pcd

        if len(combined_pcd.points) == 0:
            return combined_pcd

        # Remove duplicate/overlapping points using voxel downsampling
        voxel_size = 0.005  # 5mm voxel size
        stitched_pcd = combined_pcd.voxel_down_sample(voxel_size)

        # Optional: Statistical outlier removal
        if len(stitched_pcd.points) > 20:
            stitched_pcd, _ = stitched_pcd.remove_statistical_outlier(
                nb_neighbors=20, std_ratio=2.0
            )

        return stitched_pcd

    def merge_multi_view_detections(self, all_objects: list[dict]) -> list[dict]:
        """
        Merge object detections from multiple cameras based on 3D proximity.
        Remove duplicates when objects are too close in 3D space.
        """
        if len(all_objects) <= 1:
            return all_objects

        merged = []
        used_indices = set()

        for i, obj1 in enumerate(all_objects):
            if i in used_indices:
                continue

            # Start with this object
            merged_obj = obj1.copy()
            used_indices.add(i)

            # Check for duplicates
            for j, obj2 in enumerate(all_objects[i + 1 :], start=i + 1):
                if j in used_indices:
                    continue

                # Check 3D proximity using point cloud centroids
                if self.objects_are_duplicate_3d(obj1, obj2, threshold=0.05):  # 5cm threshold
                    # Merge the objects (keep the one with higher confidence)
                    if obj2.get("confidence", 0) > merged_obj.get("confidence", 0):
                        merged_obj = obj2.copy()
                    used_indices.add(j)

            merged.append(merged_obj)

        return merged

    def objects_are_duplicate_3d(self, obj1: dict, obj2: dict, threshold: float = 0.05) -> bool:
        """Check if two objects are duplicates based on 3D proximity."""
        pcd1 = obj1.get("point_cloud")
        pcd2 = obj2.get("point_cloud")

        if pcd1 is None or pcd2 is None:
            return False

        if len(pcd1.points) == 0 or len(pcd2.points) == 0:
            return False

        # Compute centroids
        centroid1 = np.asarray(pcd1.points).mean(axis=0)
        centroid2 = np.asarray(pcd2.points).mean(axis=0)

        # Check distance
        distance = np.linalg.norm(centroid1 - centroid2)
        return distance < threshold

    def cleanup(self) -> None:
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        for pcf in self.pointcloud_filters:
            if hasattr(pcf, "cleanup"):
                pcf.cleanup()

        if self.segmenter and hasattr(self.segmenter, "cleanup"):
            self.segmenter.cleanup()
        if self.grasp_generator and hasattr(self.grasp_generator, "cleanup"):
            self.grasp_generator.cleanup()
        logger.info("ManipulationProcessor cleaned up")
