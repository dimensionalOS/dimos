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

import json
import logging
import time
import asyncio
import websockets
from typing import Dict, List, Optional, Any, Tuple
import numpy as np
import cv2

from dimos.utils.logging_config import setup_logger
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.perception.pointcloud.pointcloud_filtering import PointcloudFiltering
from dimos.perception.segmentation.sam_2d_seg import Sam2DSegmenter
from dimos.perception.grasp_generation.utils import draw_grasps_on_image
from dimos.perception.pointcloud.utils import create_point_cloud_overlay_visualization
from dimos.perception.common.utils import colorize_depth, detection_results_to_object_data

logger = setup_logger("dimos.perception.manip_aio_processor")


class ManipulationProcessor:
    """
    Sequential manipulation processor for single-frame processing.

    Processes RGB-D frames through object detection, point cloud filtering,
    and optional grasp generation in a single thread without reactive streams.
    """

    def __init__(
        self,
        camera_intrinsics: List[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        max_objects: int = 20,
        vocabulary: Optional[str] = None,
        grasp_server_url: Optional[str] = None,
        enable_grasp_generation: bool = False,
        enable_segmentation: bool = True,
        segmentation_model: str = "sam2_b.pt",
    ):
        """
        Initialize the manipulation processor.

        Args:
            camera_intrinsics: [fx, fy, cx, cy] camera parameters
            min_confidence: Minimum detection confidence threshold
            max_objects: Maximum number of objects to process
            vocabulary: Optional vocabulary for Detic detector
            grasp_server_url: Optional WebSocket URL for AnyGrasp server
            enable_grasp_generation: Whether to enable grasp generation
            enable_segmentation: Whether to enable semantic segmentation
            segmentation_model: Segmentation model to use (SAM 2 or FastSAM)
        """
        self.camera_intrinsics = camera_intrinsics
        self.min_confidence = min_confidence
        self.max_objects = max_objects
        self.grasp_server_url = grasp_server_url
        self.enable_grasp_generation = enable_grasp_generation
        self.enable_segmentation = enable_segmentation

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
                model_path=segmentation_model,
                device="cuda",
                use_tracker=False,  # Disable tracker for simple segmentation
                use_analyzer=False,  # Disable analyzer for simple segmentation
                model_type="auto",  # Auto-detect model type
            )

        logger.info(f"Initialized ManipulationProcessor with confidence={min_confidence}")

    def process_frame(
        self, rgb_image: np.ndarray, depth_image: np.ndarray, generate_grasps: bool = None
    ) -> Dict[str, Any]:
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
                - all_objects: All objects (including misc objects) (SAM segmentation) with point clouds filtered
                - full_pointcloud: Complete scene point cloud (if point cloud processing enabled)
                - misc_clusters: List of clustered background/miscellaneous point clouds (DBSCAN)
                - misc_pointcloud_viz: Visualization of misc/background cluster overlay
                - grasps: Grasp results (if enabled)
                - grasp_overlay: Grasp visualization (if enabled)
                - processing_time: Total processing time
        """
        start_time = time.time()
        results = {}

        try:
            # Step 1: Object Detection
            step_start = time.time()
            logger.debug("Running object detection...")
            detection_results = self._run_object_detection(rgb_image)

            results["detection2d_objects"] = detection_results.get("objects", [])
            results["detection_viz"] = detection_results.get("viz_frame")
            detection_time = time.time() - step_start

            # Step 2: Semantic Segmentation (if enabled)
            segmentation_time = 0
            segmentation_results = None
            if self.enable_segmentation:
                step_start = time.time()
                logger.debug("Running semantic segmentation...")
                segmentation_results = self._run_segmentation(rgb_image)
                results["segmentation2d_objects"] = segmentation_results.get("objects", [])
                results["segmentation_viz"] = segmentation_results.get("viz_frame")
                segmentation_time = time.time() - step_start

            # Step 3: Point Cloud Processing
            pointcloud_time = 0
            detection2d_objects = results.get("detection2d_objects", [])
            segmentation2d_objects = results.get("segmentation2d_objects", [])

            # Process detection objects if available
            detected_objects = []
            if detection2d_objects:
                step_start = time.time()
                logger.debug(f"Processing {len(detection2d_objects)} detection2d_objects...")
                detected_objects = self._run_pointcloud_filtering(
                    rgb_image, depth_image, detection2d_objects
                )
                pointcloud_time += time.time() - step_start

            # Process segmentation objects if available
            segmentation_filtered_objects = []
            if segmentation2d_objects:
                step_start = time.time()
                logger.debug(f"Processing {len(segmentation2d_objects)} segmentation objects...")
                segmentation_filtered_objects = self._run_pointcloud_filtering(
                    rgb_image, depth_image, segmentation2d_objects
                )
                pointcloud_time += time.time() - step_start

            # Combine all objects
            all_objects = segmentation_filtered_objects

            # Get full point cloud
            full_pcd = self.pointcloud_filter.get_full_point_cloud()

            # Calculate misc_points clusters (full point cloud minus all object points)
            misc_start = time.time()
            from dimos.perception.pointcloud.utils import extract_and_cluster_misc_points

            misc_clusters = extract_and_cluster_misc_points(
                full_pcd,
                all_objects,
                eps=0.05,  # 5cm cluster distance
                min_points=50,  # Minimum 50 points per cluster
                enable_filtering=True,
            )
            misc_time = time.time() - misc_start

            results["detected_objects"] = detected_objects
            results["all_objects"] = all_objects
            results["full_pointcloud"] = full_pcd
            results["misc_clusters"] = misc_clusters

            # Create point cloud visualizations
            base_image = colorize_depth(depth_image, max_depth=10.0)

            # Main pointcloud visualization (all objects)
            if all_objects:
                results["pointcloud_viz"] = create_point_cloud_overlay_visualization(
                    base_image=base_image,
                    objects=all_objects,
                    intrinsics=self.camera_intrinsics,
                )
            else:
                results["pointcloud_viz"] = base_image

            # Detection objects pointcloud visualization
            if detected_objects:
                results["detected_pointcloud_viz"] = create_point_cloud_overlay_visualization(
                    base_image=base_image,
                    objects=detected_objects,
                    intrinsics=self.camera_intrinsics,
                )
            else:
                results["detected_pointcloud_viz"] = base_image

            # Misc clusters visualization overlay
            if misc_clusters:
                from dimos.perception.pointcloud.utils import overlay_point_clouds_on_image

                # Generate random colors for each cluster
                cluster_colors = []
                for i in range(len(misc_clusters)):
                    np.random.seed(i + 100)  # Consistent colors
                    color = tuple((np.random.rand(3) * 255).astype(int))
                    cluster_colors.append(color)

                results["misc_pointcloud_viz"] = overlay_point_clouds_on_image(
                    base_image=base_image,
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

            if should_generate_grasps and all_objects:
                logger.debug("Generating grasps...")
                grasps = self._run_grasp_generation(all_objects)
                results["grasps"] = grasps

                # Create grasp overlay
                if grasps:
                    results["grasp_overlay"] = self._create_grasp_overlay(rgb_image, grasps)

            # Ensure segmentation runs even if no objects detected
            if self.enable_segmentation and "segmentation_viz" not in results:
                logger.debug("Running semantic segmentation (no objects detected)...")
                segmentation_results = self._run_segmentation(rgb_image)
                results["segmentation2d_objects"] = segmentation_results.get("objects", [])
                results["segmentation_viz"] = segmentation_results.get("viz_frame")

        except Exception as e:
            logger.error(f"Error processing frame: {e}")
            results["error"] = str(e)

        # Add timing information
        total_time = time.time() - start_time
        results["processing_time"] = total_time
        results["timing_breakdown"] = {
            "detection": detection_time if "detection_time" in locals() else 0,
            "segmentation": segmentation_time if "segmentation_time" in locals() else 0,
            "pointcloud": pointcloud_time if "pointcloud_time" in locals() else 0,
            "misc_extraction": misc_time if "misc_time" in locals() else 0,
            "total": total_time,
        }
        logger.debug(f"Frame processing completed in {total_time:.3f}s")
        logger.debug(
            f"Timing breakdown: detection={detection_time:.3f}s, segmentation={segmentation_time:.3f}s, pointcloud={pointcloud_time:.3f}s"
        )

        return results

    def _run_object_detection(self, rgb_image: np.ndarray) -> Dict[str, Any]:
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

    def _run_pointcloud_filtering(
        self, rgb_image: np.ndarray, depth_image: np.ndarray, objects: List[Dict]
    ) -> List[Dict]:
        """Run point cloud filtering on detected objects."""
        try:
            filtered_objects = self.pointcloud_filter.process_images(
                rgb_image, depth_image, objects
            )
            return filtered_objects if filtered_objects else []
        except Exception as e:
            logger.error(f"Point cloud filtering failed: {e}")
            return []

    def _run_segmentation(self, rgb_image: np.ndarray) -> Dict[str, Any]:
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

    def _run_grasp_generation(self, filtered_objects: List[Dict]) -> Optional[List[Dict]]:
        """Run grasp generation on filtered objects."""
        if not self.grasp_server_url:
            logger.warning("Grasp generation requested but no server URL provided")
            return None

        try:
            # Combine all point clouds
            all_points = []
            all_colors = []
            valid_objects = 0

            for obj in filtered_objects:
                if "point_cloud_numpy" not in obj or obj["point_cloud_numpy"] is None:
                    continue

                points = obj["point_cloud_numpy"]
                if not isinstance(points, np.ndarray) or points.size == 0:
                    continue

                if len(points.shape) != 2 or points.shape[1] != 3:
                    continue

                colors = None
                if "colors_numpy" in obj and obj["colors_numpy"] is not None:
                    colors = obj["colors_numpy"]
                    if isinstance(colors, np.ndarray) and colors.size > 0:
                        if (
                            colors.shape[0] != points.shape[0]
                            or len(colors.shape) != 2
                            or colors.shape[1] != 3
                        ):
                            colors = None

                all_points.append(points)
                if colors is not None:
                    all_colors.append(colors)
                valid_objects += 1

            if not all_points:
                return None

            # Combine point clouds
            combined_points = np.vstack(all_points)
            combined_colors = None
            if len(all_colors) == valid_objects and len(all_colors) > 0:
                combined_colors = np.vstack(all_colors)

            # Send grasp request synchronously
            return self._send_grasp_request_sync(combined_points, combined_colors)

        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

    def _send_grasp_request_sync(
        self, points: np.ndarray, colors: Optional[np.ndarray]
    ) -> Optional[List[Dict]]:
        """Send synchronous grasp request to AnyGrasp server."""
        try:
            # Validation (same as async version)
            if points is None or not isinstance(points, np.ndarray) or points.size == 0:
                logger.error("Invalid points array")
                return None

            if len(points.shape) != 2 or points.shape[1] != 3:
                logger.error(f"Points has invalid shape {points.shape}, expected (N, 3)")
                return None

            if points.shape[0] < 100:
                logger.error(f"Insufficient points for grasp detection: {points.shape[0]} < 100")
                return None

            # Prepare colors
            if colors is not None:
                if not isinstance(colors, np.ndarray) or colors.size == 0:
                    colors = None
                elif len(colors.shape) != 2 or colors.shape[1] != 3:
                    colors = None
                elif colors.shape[0] != points.shape[0]:
                    colors = None

            if colors is None:
                colors = np.ones((points.shape[0], 3), dtype=np.float32) * 0.5

            # Ensure correct data types
            points = points.astype(np.float32)
            colors = colors.astype(np.float32)

            # Validate ranges
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                logger.error("Points contain NaN or Inf values")
                return None
            if np.any(np.isnan(colors)) or np.any(np.isinf(colors)):
                logger.error("Colors contain NaN or Inf values")
                return None

            colors = np.clip(colors, 0.0, 1.0)

            # Run async request in sync context
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                result = loop.run_until_complete(self._async_grasp_request(points, colors))
                return result
            finally:
                loop.close()

        except Exception as e:
            logger.error(f"Error in synchronous grasp request: {e}")
            return None

    async def _async_grasp_request(
        self, points: np.ndarray, colors: np.ndarray
    ) -> Optional[List[Dict]]:
        """Async grasp request helper."""
        try:
            async with websockets.connect(self.grasp_server_url) as websocket:
                request = {
                    "points": points.tolist(),
                    "colors": colors.tolist(),
                    "lims": [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0],
                }

                await websocket.send(json.dumps(request))
                response = await websocket.recv()
                grasps = json.loads(response)

                if isinstance(grasps, dict) and "error" in grasps:
                    logger.error(f"Server returned error: {grasps['error']}")
                    return None
                elif isinstance(grasps, (int, float)) and grasps == 0:
                    return None
                elif not isinstance(grasps, list):
                    logger.error(f"Server returned unexpected response type: {type(grasps)}")
                    return None
                elif len(grasps) == 0:
                    return None

                return self._convert_grasp_format(grasps)

        except Exception as e:
            logger.error(f"Async grasp request failed: {e}")
            return None

    def _create_grasp_overlay(self, rgb_image: np.ndarray, grasps: List[Dict]) -> np.ndarray:
        """Create grasp visualization overlay on RGB image."""
        try:
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            result_bgr = draw_grasps_on_image(
                bgr_image,
                grasps,
                self.camera_intrinsics,
                max_grasps=-1,  # Show all grasps
            )
            return cv2.cvtColor(result_bgr, cv2.COLOR_BGR2RGB)
        except Exception as e:
            logger.error(f"Error creating grasp overlay: {e}")
            return rgb_image.copy()

    def _convert_grasp_format(self, anygrasp_grasps: List[dict]) -> List[dict]:
        """Convert AnyGrasp format to visualization format."""
        converted = []

        for i, grasp in enumerate(anygrasp_grasps):
            rotation_matrix = np.array(grasp.get("rotation_matrix", np.eye(3)))
            euler_angles = self._rotation_matrix_to_euler(rotation_matrix)

            converted_grasp = {
                "id": f"grasp_{i}",
                "score": grasp.get("score", 0.0),
                "width": grasp.get("width", 0.0),
                "height": grasp.get("height", 0.0),
                "depth": grasp.get("depth", 0.0),
                "translation": grasp.get("translation", [0, 0, 0]),
                "rotation_matrix": rotation_matrix.tolist(),
                "euler_angles": euler_angles,
            }
            converted.append(converted_grasp)

        converted.sort(key=lambda x: x["score"], reverse=True)
        return converted

    def _rotation_matrix_to_euler(self, rotation_matrix: np.ndarray) -> Dict[str, float]:
        """Convert rotation matrix to Euler angles (in radians)."""
        sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)

        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = 0

        return {"roll": x, "pitch": y, "yaw": z}

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        if hasattr(self.pointcloud_filter, "cleanup"):
            self.pointcloud_filter.cleanup()
        if self.segmenter and hasattr(self.segmenter, "cleanup"):
            self.segmenter.cleanup()
        logger.info("ManipulationProcessor cleaned up")
