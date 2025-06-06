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
Asynchronous, reactive manipulation pipeline for realtime detection, filtering, and grasp generation.
"""

import asyncio
import json
import threading
import time
from typing import Dict, List, Optional
import numpy as np
import reactivex as rx
import reactivex.operators as ops
import websockets
from dimos.utils.logging_config import setup_logger
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.perception.pointcloud.pointcloud_filtering import PointcloudFiltering
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.perception.pointcloud.utils import create_point_cloud_overlay_visualization
from dimos.perception.common.utils import colorize_depth
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.manip_aio_pipeline")


class ManipulationPipeline:
    """
    Clean separated stream pipeline with frame buffering.

    - Object detection runs independently on RGB stream
    - Point cloud processing subscribes to both detection and ZED streams separately
    - Simple frame buffering to match RGB+depth+objects
    """

    def __init__(
        self,
        camera_intrinsics: List[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        max_objects: int = 10,
        vocabulary: Optional[str] = None,
        grasp_server_url: Optional[str] = None,
        enable_grasp_generation: bool = False,
    ):
        """
        Initialize the manipulation pipeline.

        Args:
            camera_intrinsics: [fx, fy, cx, cy] camera parameters
            min_confidence: Minimum detection confidence threshold
            max_objects: Maximum number of objects to process
            vocabulary: Optional vocabulary for Detic detector
            grasp_server_url: Optional WebSocket URL for AnyGrasp server
            enable_grasp_generation: Whether to enable async grasp generation
        """
        self.camera_intrinsics = camera_intrinsics
        self.min_confidence = min_confidence

        # Grasp generation settings
        self.grasp_server_url = grasp_server_url
        self.enable_grasp_generation = enable_grasp_generation

        # Asyncio event loop for WebSocket communication
        self.grasp_loop = None
        self.grasp_loop_thread = None

        # Storage for grasp results and filtered objects
        self.latest_grasps: List[dict] = []  # Simplified: just a list of grasps
        self.latest_filtered_objects = []
        self.grasp_lock = threading.Lock()

        # Track pending requests - simplified to single task
        self.grasp_task: Optional[asyncio.Task] = None

        # Reactive subjects for streaming filtered objects and grasps
        self.filtered_objects_subject = rx.subject.Subject()
        self.grasps_subject = rx.subject.Subject()

        # Initialize grasp client if enabled
        if self.enable_grasp_generation and self.grasp_server_url:
            self._start_grasp_loop()

        # Initialize object detector
        self.detector = Detic2DDetector(vocabulary=vocabulary, threshold=min_confidence)

        # Initialize point cloud processor
        self.pointcloud_filter = PointcloudFiltering(
            color_intrinsics=camera_intrinsics,
            depth_intrinsics=camera_intrinsics,  # ZED uses same intrinsics
            max_num_objects=max_objects,
        )

        logger.info(f"Initialized ManipulationPipeline with confidence={min_confidence}")

    def create_streams(self, zed_stream: rx.Observable) -> Dict[str, rx.Observable]:
        """
        Create streams using exact old main logic.
        """
        # Create ZED streams (from old main)
        zed_frame_stream = zed_stream.pipe(ops.share())

        # RGB stream for object detection (from old main)
        video_stream = zed_frame_stream.pipe(
            ops.map(lambda x: x.get("rgb") if x is not None else None),
            ops.filter(lambda x: x is not None),
            ops.share(),
        )
        object_detector = ObjectDetectionStream(
            camera_intrinsics=self.camera_intrinsics,
            min_confidence=self.min_confidence,
            class_filter=None,
            detector=self.detector,
            video_stream=video_stream,
            disable_depth=True,
        )

        # Store latest frames for point cloud processing (from old main)
        latest_rgb = None
        latest_depth = None
        latest_point_cloud_overlay = None
        frame_lock = threading.Lock()

        # Subscribe to combined ZED frames (from old main)
        def on_zed_frame(zed_data):
            nonlocal latest_rgb, latest_depth
            if zed_data is not None:
                with frame_lock:
                    latest_rgb = zed_data.get("rgb")
                    latest_depth = zed_data.get("depth")

        # Depth stream for point cloud filtering (from old main)
        def get_depth_or_overlay(zed_data):
            if zed_data is None:
                return None

            # Check if we have a point cloud overlay available
            with frame_lock:
                overlay = latest_point_cloud_overlay

            if overlay is not None:
                return overlay
            else:
                # Return regular colorized depth
                return colorize_depth(zed_data.get("depth"), max_depth=10.0)

        depth_stream = zed_frame_stream.pipe(
            ops.map(get_depth_or_overlay), ops.filter(lambda x: x is not None), ops.share()
        )

        # Process object detection results with point cloud filtering (from old main)
        def on_detection_next(result):
            nonlocal latest_point_cloud_overlay
            if "objects" in result and result["objects"]:
                # Get latest RGB and depth frames
                with frame_lock:
                    rgb = latest_rgb
                    depth = latest_depth

                if rgb is not None and depth is not None:
                    try:
                        filtered_objects = self.pointcloud_filter.process_images(
                            rgb, depth, result["objects"]
                        )

                        if filtered_objects:
                            # Store filtered objects
                            with self.grasp_lock:
                                self.latest_filtered_objects = filtered_objects
                            self.filtered_objects_subject.on_next(filtered_objects)

                            # Request grasps if enabled
                            if self.enable_grasp_generation and filtered_objects:
                                logger.debug(
                                    f"Requesting grasps for {len(filtered_objects)} filtered objects"
                                )
                                task = self.request_scene_grasps(filtered_objects)
                                if task:
                                    logger.debug(
                                        "Grasp request task created, waiting for results..."
                                    )

                                    # Check for results after a delay
                                    def check_grasps_later():
                                        logger.debug("Starting delayed grasp check...")
                                        time.sleep(2.0)  # Wait for grasp processing
                                        grasps = self.get_latest_grasps()
                                        if grasps:
                                            logger.debug(
                                                f"Found {len(grasps)} grasps in delayed check"
                                            )
                                            self.grasps_subject.on_next(grasps)
                                            logger.info(f"Received {len(grasps)} grasps for scene")
                                            logger.debug(f"Grasps for scene: {grasps}")
                                        else:
                                            logger.debug("No grasps found in delayed check")

                                    threading.Thread(target=check_grasps_later, daemon=True).start()
                                else:
                                    logger.debug("Failed to create grasp request task")

                            # Create base image (colorized depth)
                            base_image = colorize_depth(depth, max_depth=10.0)

                            # Create point cloud overlay visualization
                            overlay_viz = create_point_cloud_overlay_visualization(
                                base_image=base_image,
                                objects=filtered_objects,
                                intrinsics=self.camera_intrinsics,
                            )

                            # Store the overlay for the stream
                            with frame_lock:
                                latest_point_cloud_overlay = overlay_viz
                        else:
                            # No filtered objects, clear overlay
                            with frame_lock:
                                latest_point_cloud_overlay = None

                    except Exception as e:
                        logger.error(f"Error in point cloud filtering: {e}")
                        with frame_lock:
                            latest_point_cloud_overlay = None

        def on_error(error):
            logger.error(f"Error in stream: {error}")

        def on_completed():
            logger.info("Stream completed")

        def start_subscriptions():
            """Start subscriptions in background thread (from old main)"""
            # Subscribe to combined ZED frames
            zed_frame_stream.subscribe(on_next=on_zed_frame)

        # Start subscriptions in background thread (from old main)
        subscription_thread = threading.Thread(target=start_subscriptions, daemon=True)
        subscription_thread.start()
        time.sleep(2)  # Give subscriptions time to start

        # Subscribe to object detection stream (from old main)
        object_detector.get_stream().subscribe(
            on_next=on_detection_next, on_error=on_error, on_completed=on_completed
        )

        # Create visualization stream for web interface (from old main)
        viz_stream = object_detector.get_stream().pipe(
            ops.map(lambda x: x["viz_frame"] if x is not None else None),
            ops.filter(lambda x: x is not None),
        )

        # Create filtered objects stream
        filtered_objects_stream = self.filtered_objects_subject

        # Create grasps stream
        grasps_stream = self.grasps_subject

        return {
            "detection_viz": viz_stream,
            "pointcloud_viz": depth_stream,
            "objects": object_detector.get_stream().pipe(ops.map(lambda x: x.get("objects", []))),
            "filtered_objects": filtered_objects_stream,
            "grasps": grasps_stream,
        }

    def _start_grasp_loop(self):
        """Start asyncio event loop in a background thread for WebSocket communication."""

        def run_loop():
            self.grasp_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.grasp_loop)
            self.grasp_loop.run_forever()

        self.grasp_loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.grasp_loop_thread.start()

        # Wait for loop to start
        while self.grasp_loop is None:
            time.sleep(0.01)

    async def _send_grasp_request(
        self, points: np.ndarray, colors: Optional[np.ndarray]
    ) -> Optional[List[dict]]:
        """Send grasp request to AnyGrasp server."""
        logger.debug(f"_send_grasp_request called with {len(points)} points")

        try:
            logger.debug(f"Connecting to WebSocket: {self.grasp_server_url}")
            async with websockets.connect(self.grasp_server_url) as websocket:
                logger.debug("WebSocket connected successfully")

                # Use the correct format expected by AnyGrasp server
                request = {
                    "points": points.tolist(),
                    "colors": colors.tolist() if colors is not None else None,
                    "lims": [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0],  # Default workspace limits
                }

                logger.debug(f"Sending grasp request with {len(points)} points")
                await websocket.send(json.dumps(request))

                logger.debug("Waiting for response...")
                response = await websocket.recv()
                logger.debug(f"Received response: {len(response)} characters")

                # Parse response - server returns list of grasps directly
                grasps = json.loads(response)
                logger.debug(f"Received {len(grasps) if grasps else 0} grasps from server")

                if grasps and len(grasps) > 0:
                    # Convert to our format and store
                    converted_grasps = self._convert_grasp_format(grasps)
                    logger.debug(f"Converted to {len(converted_grasps)} grasps")

                    with self.grasp_lock:
                        self.latest_grasps = converted_grasps
                    logger.debug(f"Stored {len(converted_grasps)} grasps")
                    return converted_grasps
                else:
                    logger.warning("No grasps returned from server")

        except Exception as e:
            logger.error(f"Error requesting grasps: {e}")
            logger.debug(f"Error details: {e}")

        return None

    def request_scene_grasps(self, objects: List[dict]) -> Optional[asyncio.Task]:
        """Request grasps for entire scene by combining all object point clouds."""
        logger.debug(f"request_scene_grasps called with {len(objects)} objects")

        if not self.grasp_loop or not objects:
            logger.debug(
                f"Cannot request grasps: grasp_loop={self.grasp_loop is not None}, objects={len(objects) if objects else 0}"
            )
            return None

        # Combine all object point clouds
        all_points = []
        all_colors = []

        for obj in objects:
            if "point_cloud_numpy" in obj and len(obj["point_cloud_numpy"]) > 0:
                all_points.append(obj["point_cloud_numpy"])
                if "colors_numpy" in obj and obj["colors_numpy"] is not None:
                    all_colors.append(obj["colors_numpy"])
                logger.debug(f"Added object with {len(obj['point_cloud_numpy'])} points")

        if not all_points:
            logger.debug("No points found in objects, cannot request grasps")
            return None

        # Concatenate all points and colors
        combined_points = np.vstack(all_points)
        combined_colors = np.vstack(all_colors) if all_colors else None

        logger.debug(
            f"Requesting scene grasps for combined point cloud with {len(combined_points)} points"
        )
        logger.debug(f"Grasp server URL: {self.grasp_server_url}")

        # Create and schedule the task
        try:
            task = asyncio.run_coroutine_threadsafe(
                self._send_grasp_request(combined_points, combined_colors), self.grasp_loop
            )

            self.grasp_task = task
            logger.debug("Successfully created grasp request task")
            return task
        except Exception as e:
            logger.error(f"Failed to create grasp request task: {e}")
            return None

    def get_latest_grasps(self) -> Optional[List[dict]]:
        """Get latest grasp results."""
        with self.grasp_lock:
            return self.latest_grasps

    def clear_grasps(self) -> None:
        """Clear all stored grasp results."""
        with self.grasp_lock:
            self.latest_grasps = []

    def _prepare_colors(self, colors: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """Prepare colors array, converting from various formats if needed."""
        if colors is None:
            return None

        # Convert from 0-255 to 0-1 range if needed
        if colors.max() > 1.0:
            colors = colors / 255.0

        return colors

    def _convert_grasp_format(self, anygrasp_grasps: List[dict]) -> List[dict]:
        """Convert AnyGrasp format to our visualization format."""
        converted = []

        for i, grasp in enumerate(anygrasp_grasps):
            # Extract rotation matrix and convert to Euler angles
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

        # Sort by score descending
        converted.sort(key=lambda x: x["score"], reverse=True)

        return converted

    def _rotation_matrix_to_euler(self, rotation_matrix: np.ndarray) -> Dict[str, float]:
        """Convert rotation matrix to Euler angles (in radians)."""
        # Check for gimbal lock
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

        # Stop the grasp event loop
        if self.grasp_loop and self.grasp_loop_thread:
            self.grasp_loop.call_soon_threadsafe(self.grasp_loop.stop)
            self.grasp_loop_thread.join(timeout=1.0)

        if hasattr(self.pointcloud_filter, "cleanup"):
            self.pointcloud_filter.cleanup()
        logger.info("ManipulationPipeline cleaned up")
