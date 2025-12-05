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

import cv2
import numpy as np
import reactivex as rx
import reactivex.operators as ops
import websockets

from dimos.perception.common.utils import colorize_depth
from dimos.perception.detection2d.detic_2d_det import (  # type: ignore[import-untyped]
    Detic2DDetector,
)
from dimos.perception.grasp_generation.utils import draw_grasps_on_image
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.perception.pointcloud.pointcloud_filtering import PointcloudFiltering
from dimos.perception.pointcloud.utils import create_point_cloud_overlay_visualization
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ManipulationPipeline:
    """
    Clean separated stream pipeline with frame buffering.

    - Object detection runs independently on RGB stream
    - Point cloud processing subscribes to both detection and ZED streams separately
    - Simple frame buffering to match RGB+depth+objects
    """

    def __init__(
        self,
        camera_intrinsics: list[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        max_objects: int = 10,
        vocabulary: str | None = None,
        grasp_server_url: str | None = None,
        enable_grasp_generation: bool = False,
    ) -> None:
        """
        Initialize the manipulation pipeline.

        Args:
            camera_intrinsics: [fx, fy, cx, cy] camera parameters
            min_confidence: Minimum detection confidence threshold
            max_objects: Maximum number of objects to process
            vocabulary: Optional vocabulary for Detic detector
            grasp_server_url: Optional WebSocket URL for Dimensional Grasp server
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
        self.latest_grasps: list[dict] = []  # type: ignore[type-arg]  # Simplified: just a list of grasps
        self.grasps_consumed = False
        self.latest_filtered_objects = []  # type: ignore[var-annotated]
        self.latest_rgb_for_grasps = None  # Store RGB image for grasp overlay
        self.grasp_lock = threading.Lock()

        # Track pending requests - simplified to single task
        self.grasp_task: asyncio.Task | None = None  # type: ignore[type-arg]

        # Reactive subjects for streaming filtered objects and grasps
        self.filtered_objects_subject = rx.subject.Subject()  # type: ignore[var-annotated]
        self.grasps_subject = rx.subject.Subject()  # type: ignore[var-annotated]
        self.grasp_overlay_subject = rx.subject.Subject()  # type: ignore[var-annotated]  # Add grasp overlay subject

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

    def create_streams(self, zed_stream: rx.Observable) -> dict[str, rx.Observable]:  # type: ignore[type-arg]
        """
        Create streams using exact old main logic.
        """
        # Create ZED streams (from old main)
        zed_frame_stream = zed_stream.pipe(ops.share())

        # RGB stream for object detection (from old main)
        video_stream = zed_frame_stream.pipe(
            ops.map(lambda x: x.get("rgb") if x is not None else None),  # type: ignore[attr-defined]
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
        def on_zed_frame(zed_data) -> None:  # type: ignore[no-untyped-def]
            nonlocal latest_rgb, latest_depth
            if zed_data is not None:
                with frame_lock:
                    latest_rgb = zed_data.get("rgb")
                    latest_depth = zed_data.get("depth")

        # Depth stream for point cloud filtering (from old main)
        def get_depth_or_overlay(zed_data):  # type: ignore[no-untyped-def]
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
        def on_detection_next(result) -> None:  # type: ignore[no-untyped-def]
            nonlocal latest_point_cloud_overlay
            if result.get("objects"):
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

                            # Create base image (colorized depth)
                            base_image = colorize_depth(depth, max_depth=10.0)

                            # Create point cloud overlay visualization
                            overlay_viz = create_point_cloud_overlay_visualization(
                                base_image=base_image,  # type: ignore[arg-type]
                                objects=filtered_objects,  # type: ignore[arg-type]
                                intrinsics=self.camera_intrinsics,  # type: ignore[arg-type]
                            )

                            # Store the overlay for the stream
                            with frame_lock:
                                latest_point_cloud_overlay = overlay_viz

                            # Request grasps if enabled
                            if self.enable_grasp_generation and len(filtered_objects) > 0:
                                # Save RGB image for later grasp overlay
                                with frame_lock:
                                    self.latest_rgb_for_grasps = rgb.copy()

                                task = self.request_scene_grasps(filtered_objects)  # type: ignore[arg-type]
                                if task:
                                    # Check for results after a delay
                                    def check_grasps_later() -> None:
                                        time.sleep(2.0)  # Wait for grasp processing
                                        # Wait for task to complete
                                        if hasattr(self, "grasp_task") and self.grasp_task:
                                            try:
                                                self.grasp_task.result(  # type: ignore[call-arg]
                                                    timeout=3.0
                                                )  # Get result with timeout
                                            except Exception as e:
                                                logger.warning(f"Grasp task failed or timeout: {e}")

                                        # Try to get latest grasps and create overlay
                                        with self.grasp_lock:
                                            grasps = self.latest_grasps

                                        if grasps and hasattr(self, "latest_rgb_for_grasps"):
                                            # Create grasp overlay on the saved RGB image
                                            try:
                                                bgr_image = cv2.cvtColor(  # type: ignore[call-overload]
                                                    self.latest_rgb_for_grasps, cv2.COLOR_RGB2BGR
                                                )
                                                result_bgr = draw_grasps_on_image(
                                                    bgr_image,
                                                    grasps,
                                                    self.camera_intrinsics,
                                                    max_grasps=-1,  # Show all grasps
                                                )
                                                result_rgb = cv2.cvtColor(
                                                    result_bgr, cv2.COLOR_BGR2RGB
                                                )

                                                # Emit grasp overlay immediately
                                                self.grasp_overlay_subject.on_next(result_rgb)

                                            except Exception as e:
                                                logger.error(f"Error creating grasp overlay: {e}")

                                            # Emit grasps to stream
                                            self.grasps_subject.on_next(grasps)

                                    threading.Thread(target=check_grasps_later, daemon=True).start()
                                else:
                                    logger.warning("Failed to create grasp task")
                    except Exception as e:
                        logger.error(f"Error in point cloud filtering: {e}")
                        with frame_lock:
                            latest_point_cloud_overlay = None

        def on_error(error) -> None:  # type: ignore[no-untyped-def]
            logger.error(f"Error in stream: {error}")

        def on_completed() -> None:
            logger.info("Stream completed")

        def start_subscriptions() -> None:
            """Start subscriptions in background thread (from old main)"""
            # Subscribe to combined ZED frames
            zed_frame_stream.subscribe(on_next=on_zed_frame)

        # Start subscriptions in background thread (from old main)
        subscription_thread = threading.Thread(target=start_subscriptions, daemon=True)
        subscription_thread.start()
        time.sleep(2)  # Give subscriptions time to start

        # Subscribe to object detection stream (from old main)
        object_detector.get_stream().subscribe(  # type: ignore[no-untyped-call]
            on_next=on_detection_next, on_error=on_error, on_completed=on_completed
        )

        # Create visualization stream for web interface (from old main)
        viz_stream = object_detector.get_stream().pipe(  # type: ignore[no-untyped-call]
            ops.map(lambda x: x["viz_frame"] if x is not None else None),  # type: ignore[index]
            ops.filter(lambda x: x is not None),
        )

        # Create filtered objects stream
        filtered_objects_stream = self.filtered_objects_subject

        # Create grasps stream
        grasps_stream = self.grasps_subject

        # Create grasp overlay subject for immediate emission
        grasp_overlay_stream = self.grasp_overlay_subject

        return {
            "detection_viz": viz_stream,
            "pointcloud_viz": depth_stream,
            "objects": object_detector.get_stream().pipe(ops.map(lambda x: x.get("objects", []))),  # type: ignore[attr-defined, no-untyped-call]
            "filtered_objects": filtered_objects_stream,
            "grasps": grasps_stream,
            "grasp_overlay": grasp_overlay_stream,
        }

    def _start_grasp_loop(self) -> None:
        """Start asyncio event loop in a background thread for WebSocket communication."""

        def run_loop() -> None:
            self.grasp_loop = asyncio.new_event_loop()  # type: ignore[assignment]
            asyncio.set_event_loop(self.grasp_loop)
            self.grasp_loop.run_forever()  # type: ignore[attr-defined]

        self.grasp_loop_thread = threading.Thread(target=run_loop, daemon=True)  # type: ignore[assignment]
        self.grasp_loop_thread.start()  # type: ignore[attr-defined]

        # Wait for loop to start
        while self.grasp_loop is None:
            time.sleep(0.01)

    async def _send_grasp_request(
        self,
        points: np.ndarray,  # type: ignore[type-arg]
        colors: np.ndarray | None,  # type: ignore[type-arg]
    ) -> list[dict] | None:  # type: ignore[type-arg]
        """Send grasp request to Dimensional Grasp server."""
        try:
            # Comprehensive client-side validation to prevent server errors

            # Validate points array
            if points is None:
                logger.error("Points array is None")
                return None
            if not isinstance(points, np.ndarray):
                logger.error(f"Points is not numpy array: {type(points)}")
                return None
            if points.size == 0:
                logger.error("Points array is empty")
                return None
            if len(points.shape) != 2 or points.shape[1] != 3:
                logger.error(f"Points has invalid shape {points.shape}, expected (N, 3)")
                return None
            if points.shape[0] < 100:  # Minimum points for stable grasp detection
                logger.error(f"Insufficient points for grasp detection: {points.shape[0]} < 100")
                return None

            # Validate and prepare colors
            if colors is not None:
                if not isinstance(colors, np.ndarray):
                    colors = None
                elif colors.size == 0:
                    colors = None
                elif len(colors.shape) != 2 or colors.shape[1] != 3:
                    colors = None
                elif colors.shape[0] != points.shape[0]:
                    colors = None

            # If no valid colors, create default colors (required by server)
            if colors is None:
                # Create default white colors for all points
                colors = np.ones((points.shape[0], 3), dtype=np.float32) * 0.5

            # Ensure data types are correct (server expects float32)
            points = points.astype(np.float32)
            colors = colors.astype(np.float32)

            # Validate ranges (basic sanity checks)
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                logger.error("Points contain NaN or Inf values")
                return None
            if np.any(np.isnan(colors)) or np.any(np.isinf(colors)):
                logger.error("Colors contain NaN or Inf values")
                return None

            # Clamp color values to valid range [0, 1]
            colors = np.clip(colors, 0.0, 1.0)

            async with websockets.connect(self.grasp_server_url) as websocket:  # type: ignore[arg-type]
                request = {
                    "points": points.tolist(),
                    "colors": colors.tolist(),  # Always send colors array
                    "lims": [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0],  # Default workspace limits
                }

                await websocket.send(json.dumps(request))

                response = await websocket.recv()
                grasps = json.loads(response)

                # Handle server response validation
                if isinstance(grasps, dict) and "error" in grasps:
                    logger.error(f"Server returned error: {grasps['error']}")
                    return None
                elif isinstance(grasps, int | float) and grasps == 0:
                    return None
                elif not isinstance(grasps, list):
                    logger.error(
                        f"Server returned unexpected response type: {type(grasps)}, value: {grasps}"
                    )
                    return None
                elif len(grasps) == 0:
                    return None

                converted_grasps = self._convert_grasp_format(grasps)
                with self.grasp_lock:
                    self.latest_grasps = converted_grasps
                    self.grasps_consumed = False  # Reset consumed flag

                # Emit to reactive stream
                self.grasps_subject.on_next(self.latest_grasps)

                return converted_grasps
        except websockets.exceptions.ConnectionClosed as e:
            logger.error(f"WebSocket connection closed: {e}")
        except websockets.exceptions.WebSocketException as e:
            logger.error(f"WebSocket error: {e}")
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse server response as JSON: {e}")
        except Exception as e:
            logger.error(f"Error requesting grasps: {e}")

        return None

    def request_scene_grasps(self, objects: list[dict]) -> asyncio.Task | None:  # type: ignore[type-arg]
        """Request grasps for entire scene by combining all object point clouds."""
        if not self.grasp_loop or not objects:
            return None

        all_points = []
        all_colors = []
        valid_objects = 0

        for _i, obj in enumerate(objects):
            # Validate point cloud data
            if "point_cloud_numpy" not in obj or obj["point_cloud_numpy"] is None:
                continue

            points = obj["point_cloud_numpy"]
            if not isinstance(points, np.ndarray) or points.size == 0:
                continue

            # Ensure points have correct shape (N, 3)
            if len(points.shape) != 2 or points.shape[1] != 3:
                continue

            # Validate colors if present
            colors = None
            if "colors_numpy" in obj and obj["colors_numpy"] is not None:
                colors = obj["colors_numpy"]
                if isinstance(colors, np.ndarray) and colors.size > 0:
                    # Ensure colors match points count and have correct shape
                    if colors.shape[0] != points.shape[0]:
                        colors = None  # Ignore colors for this object
                    elif len(colors.shape) != 2 or colors.shape[1] != 3:
                        colors = None  # Ignore colors for this object

            all_points.append(points)
            if colors is not None:
                all_colors.append(colors)
            valid_objects += 1

        if not all_points:
            return None

        try:
            combined_points = np.vstack(all_points)

            # Only combine colors if ALL objects have valid colors
            combined_colors = None
            if len(all_colors) == valid_objects and len(all_colors) > 0:
                combined_colors = np.vstack(all_colors)

            # Validate final combined data
            if combined_points.size == 0:
                logger.warning("Combined point cloud is empty")
                return None

            if combined_colors is not None and combined_colors.shape[0] != combined_points.shape[0]:
                logger.warning(
                    f"Color/point count mismatch: {combined_colors.shape[0]} colors vs {combined_points.shape[0]} points, dropping colors"
                )
                combined_colors = None

        except Exception as e:
            logger.error(f"Failed to combine point clouds: {e}")
            return None

        try:
            # Check if there's already a grasp task running
            if hasattr(self, "grasp_task") and self.grasp_task and not self.grasp_task.done():
                return self.grasp_task

            task = asyncio.run_coroutine_threadsafe(
                self._send_grasp_request(combined_points, combined_colors), self.grasp_loop
            )

            self.grasp_task = task
            return task
        except Exception:
            logger.warning("Failed to create grasp task")
            return None

    def get_latest_grasps(self, timeout: float = 5.0) -> list[dict] | None:  # type: ignore[type-arg]
        """Get latest grasp results, waiting for new ones if current ones have been consumed."""
        # Mark current grasps as consumed and get a reference
        with self.grasp_lock:
            current_grasps = self.latest_grasps
            self.grasps_consumed = True

        # If we already have grasps and they haven't been consumed, return them
        if current_grasps is not None and not getattr(self, "grasps_consumed", False):
            return current_grasps

        # Wait for new grasps
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.grasp_lock:
                # Check if we have new grasps (different from what we marked as consumed)
                if self.latest_grasps is not None and not getattr(self, "grasps_consumed", False):
                    return self.latest_grasps
            time.sleep(0.1)  # Check every 100ms

        return None  # Timeout reached

    def clear_grasps(self) -> None:
        """Clear all stored grasp results."""
        with self.grasp_lock:
            self.latest_grasps = []

    def _prepare_colors(self, colors: np.ndarray | None) -> np.ndarray | None:  # type: ignore[type-arg]
        """Prepare colors array, converting from various formats if needed."""
        if colors is None:
            return None

        if colors.max() > 1.0:
            colors = colors / 255.0

        return colors

    def _convert_grasp_format(self, grasps: list[dict]) -> list[dict]:  # type: ignore[type-arg]
        """Convert Grasp format to our visualization format."""
        converted = []

        for i, grasp in enumerate(grasps):
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

    def _rotation_matrix_to_euler(self, rotation_matrix: np.ndarray) -> dict[str, float]:  # type: ignore[type-arg]
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

    def cleanup(self) -> None:
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()

        if self.grasp_loop and self.grasp_loop_thread:
            self.grasp_loop.call_soon_threadsafe(self.grasp_loop.stop)
            self.grasp_loop_thread.join(timeout=1.0)

        if hasattr(self.pointcloud_filter, "cleanup"):
            self.pointcloud_filter.cleanup()
        logger.info("ManipulationPipeline cleaned up")
