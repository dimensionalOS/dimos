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
ManipulationPipeline: Streaming wrapper around ManipulationProcessor
Works with DepthAI camera queues
"""

from collections.abc import Callable
import threading
import time
from typing import Any

from manip_aio_processer_new_depthai import ManipulationProcessor
import numpy as np
import reactivex as rx
from reactivex import operators as ops
from reactivex.subject import Subject

from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.manip_pipeline")


class ManipulationPipeline:
    """
    Streaming manipulation pipeline that wraps ManipulationProcessor.

    Handles continuous processing of multi-camera RGB-D streams using RxPy.
    Designed to work with DepthAI camera queues.
    """

    def __init__(
        self,
        camera_configs: list[dict],
        min_confidence: float = 0.6,
        max_objects: int = 20,
        vocabulary: str | None = None,
        enable_grasp_generation: bool = False,
        grasp_server_url: str | None = None,
        enable_segmentation: bool = True,
        buffer_size: int = 1,  # Number of frames to buffer
    ) -> None:
        """
        Initialize the manipulation pipeline.

        Args:
            camera_configs: List of camera configurations (same as ManipulationProcessor)
            buffer_size: Number of frames to buffer before processing
            Other args: Same as ManipulationProcessor
        """
        self.camera_configs = camera_configs
        self.num_cameras = len(camera_configs)
        self.buffer_size = buffer_size

        # Initialize the core processor
        self.processor = ManipulationProcessor(
            camera_configs=camera_configs,
            min_confidence=min_confidence,
            max_objects=max_objects,
            vocabulary=vocabulary,
            enable_grasp_generation=enable_grasp_generation,
            grasp_server_url=grasp_server_url,
            enable_segmentation=enable_segmentation,
        )

        # State management
        self.lock = threading.Lock()
        self.latest_rgb_images = [None] * self.num_cameras
        self.latest_depth_images = [None] * self.num_cameras
        self.processing = False
        self.running = False

        # Output subjects for reactive streams
        self.subjects = {
            "detection_viz": Subject(),
            "pointcloud_viz": Subject(),
            "detected_pointcloud_viz": Subject(),
            "misc_pointcloud_viz": Subject(),
            "segmentation_viz": Subject(),
            "detected_objects": Subject(),  # Objects with grasps attached
            "all_objects": Subject(),  # All objects with grasps attached
            "full_pointcloud": Subject(),
            "grasp_overlay": Subject(),  # Visualization of all grasps
            "all_grasps_list": Subject(),  # Flat list of all grasps (convenience)
            "per_camera_data": Subject(),
            "processing_time": Subject(),
        }

        logger.info(f"Initialized ManipulationPipeline with {self.num_cameras} cameras")

    def create_depthai_stream(
        self,
        color_queue: Any,  # dai.DataOutputQueue
        depth_queue: Any,  # dai.DataOutputQueue
        camera_idx: int,
    ) -> rx.Observable:
        """
        Create an RxPy observable from DepthAI queues.

        Args:
            color_queue: DepthAI color output queue
            depth_queue: DepthAI depth output queue
            camera_idx: Index of this camera

        Returns:
            Observable that emits {"rgb": np.ndarray, "depth": np.ndarray, "camera_idx": int}
        """

        def subscribe(observer, scheduler=None):
            def emit_frames():
                logger.info(f"Camera {camera_idx} stream started")

                while self.running:
                    try:
                        # Try to get frames from DepthAI queues
                        color_data = color_queue.tryGet()
                        depth_data = depth_queue.tryGet()

                        if color_data and depth_data:
                            # Get numpy arrays from DepthAI
                            rgb = color_data.getCvFrame()
                            depth = depth_data.getFrame()

                            # Convert BGR to RGB if needed
                            if rgb.shape[-1] == 3:
                                rgb = rgb[:, :, ::-1]  # BGR to RGB

                            # Emit frame
                            observer.on_next(
                                {
                                    "rgb": rgb,
                                    "depth": depth.astype(np.float32)
                                    / 1000.0,  # Convert mm to meters
                                    "camera_idx": camera_idx,
                                }
                            )
                        else:
                            # No frames available, wait a bit
                            time.sleep(0.001)

                    except Exception as e:
                        logger.error(f"Camera {camera_idx} stream error: {e}")
                        observer.on_error(e)
                        break

                observer.on_completed()

            # Start thread
            thread = threading.Thread(target=emit_frames, daemon=True)
            thread.start()

        return rx.create(subscribe)

    def process_frame_async(self, generate_grasps: bool | None = None):
        """
        Process current buffered frames asynchronously.
        Called by the streaming pipeline when frames are ready.
        """
        if self.processing:
            return  # Skip if already processing

        with self.lock:
            # Check if we have frames from all cameras
            if any(img is None for img in self.latest_rgb_images):
                return
            if any(img is None for img in self.latest_depth_images):
                return

            # Copy current frames
            rgb_images = [img.copy() for img in self.latest_rgb_images]
            depth_images = [img.copy() for img in self.latest_depth_images]

        self.processing = True

        def process():
            try:
                # Process frame using the core processor
                results = self.processor.process_frame(
                    rgb_images=rgb_images,
                    depth_images=depth_images,
                    generate_grasps=generate_grasps,
                )

                # Emit results to all output streams
                self._emit_results(results)

            except Exception as e:
                logger.error(f"Frame processing error: {e}")
                import traceback

                traceback.print_exc()
            finally:
                self.processing = False

        # Process in separate thread to avoid blocking
        threading.Thread(target=process, daemon=True).start()

    def _emit_results(self, results: dict):
        """Emit processing results to output subjects."""
        # Emit visualizations
        if "detection_viz" in results:
            per_camera_data = results.get("per_camera_data", [])
            if per_camera_data and len(per_camera_data) > 0:
                self.subjects["detection_viz"].on_next(per_camera_data[0]["detection_viz"])

        if "pointcloud_viz" in results:
            self.subjects["pointcloud_viz"].on_next(results["pointcloud_viz"])

        if "detected_pointcloud_viz" in results:
            self.subjects["detected_pointcloud_viz"].on_next(results["detected_pointcloud_viz"])

        if "misc_pointcloud_viz" in results:
            self.subjects["misc_pointcloud_viz"].on_next(results["misc_pointcloud_viz"])

        # Emit segmentation viz if available
        per_camera_data = results.get("per_camera_data", [])
        if per_camera_data and len(per_camera_data) > 0:
            if "segmentation_viz" in per_camera_data[0]:
                self.subjects["segmentation_viz"].on_next(per_camera_data[0]["segmentation_viz"])

        # Emit object data (now with grasps attached)
        if "detected_objects" in results:
            self.subjects["detected_objects"].on_next(results["detected_objects"])

        if "all_objects" in results:
            self.subjects["all_objects"].on_next(results["all_objects"])

        if "full_pointcloud" in results:
            self.subjects["full_pointcloud"].on_next(results["full_pointcloud"])

        # Emit grasp visualization and flat list
        if "grasp_overlay" in results:
            self.subjects["grasp_overlay"].on_next(results["grasp_overlay"])

        if "all_grasps_list" in results:
            self.subjects["all_grasps_list"].on_next(results["all_grasps_list"])

        # Emit timing
        if "processing_time" in results:
            self.subjects["processing_time"].on_next(results["processing_time"])

        # Emit per-camera data
        if "per_camera_data" in results:
            self.subjects["per_camera_data"].on_next(results["per_camera_data"])

    def create_pipeline(
        self,
        camera_queues: list[tuple[Any, Any]],  # List of (color_queue, depth_queue) tuples
    ) -> dict[str, rx.Observable]:
        """
        Create the complete streaming pipeline from DepthAI queues.

        Args:
            camera_queues: List of (color_queue, depth_queue) tuples, one per camera

        Returns:
            Dictionary of output observables
        """
        if len(camera_queues) != self.num_cameras:
            raise ValueError(f"Expected {self.num_cameras} camera queues, got {len(camera_queues)}")

        self.running = True

        # Create observables from DepthAI queues
        camera_streams = []
        for idx, (color_queue, depth_queue) in enumerate(camera_queues):
            stream = self.create_depthai_stream(color_queue, depth_queue, idx)
            camera_streams.append(stream)

        # Combine camera streams and update buffers
        def update_buffers(frame_data):
            cam_idx = frame_data["camera_idx"]
            with self.lock:
                self.latest_rgb_images[cam_idx] = frame_data["rgb"]
                self.latest_depth_images[cam_idx] = frame_data["depth"]

            # Trigger processing when we have all frames
            self.process_frame_async()

        # Subscribe to all camera streams
        for stream in camera_streams:
            stream.subscribe(
                on_next=update_buffers, on_error=lambda e: logger.error(f"Stream error: {e}")
            )

        # Return output observables
        return {
            "detection_viz": self.subjects["detection_viz"],
            "pointcloud_viz": self.subjects["pointcloud_viz"],
            "detected_pointcloud_viz": self.subjects["detected_pointcloud_viz"],
            "misc_pointcloud_viz": self.subjects["misc_pointcloud_viz"],
            "segmentation_viz": self.subjects["segmentation_viz"],
            "detected_objects": self.subjects["detected_objects"],  # With grasps
            "all_objects": self.subjects["all_objects"],  # With grasps
            "full_pointcloud": self.subjects["full_pointcloud"],
            "grasp_overlay": self.subjects["grasp_overlay"],
            "all_grasps_list": self.subjects["all_grasps_list"],  # Flat list
            "per_camera_data": self.subjects["per_camera_data"],
            "processing_time": self.subjects["processing_time"],
        }

    def process_single_frame(
        self,
        rgb_images: list[np.ndarray],
        depth_images: list[np.ndarray],
        generate_grasps: bool | None = None,
    ) -> dict:
        """
        Process a single frame directly (non-streaming mode).

        Useful for testing or when you want synchronous processing.

        Args:
            rgb_images: List of RGB images
            depth_images: List of depth images
            generate_grasps: Whether to generate grasps

        Returns:
            Processing results dictionary
        """
        return self.processor.process_frame(
            rgb_images=rgb_images, depth_images=depth_images, generate_grasps=generate_grasps
        )

    def get_objects_with_grasps(self) -> list[dict]:
        """
        Get the latest detected objects with grasps attached (SYNCHRONOUS).

        This is a pull-based API for on-demand queries (VLM task planning).
        Processes the current buffered frames immediately and returns results.

        Returns:
            List of objects with 'grasps' field attached to each object
        """
        with self.lock:
            # Check if we have frames from all cameras
            if any(img is None for img in self.latest_rgb_images):
                logger.warning("get_objects_with_grasps: Missing RGB frames")
                return []
            if any(img is None for img in self.latest_depth_images):
                logger.warning("get_objects_with_grasps: Missing depth frames")
                return []

            # Copy current frames
            rgb_images = [img.copy() for img in self.latest_rgb_images]
            depth_images = [img.copy() for img in self.latest_depth_images]

        # Process single frame synchronously
        results = self.processor.process_frame(
            rgb_images=rgb_images,
            depth_images=depth_images,
            generate_grasps=True,  # Always generate grasps for VLM queries
        )

        return results.get("all_objects", [])

    def find_object_at_pixel(self, pixel: tuple[int, int], camera_id: int = 0) -> dict | None:
        """
        Find the object at a given pixel location (SYNCHRONOUS).

        Maps VLM's 2D click coordinates to an actual detected object.

        Args:
            pixel: (x, y) pixel coordinates from VLM
            camera_id: Which camera's view to use (default: 0)

        Returns:
            Object dictionary with grasps if found, None otherwise
        """
        objects = self.get_objects_with_grasps()

        if not objects:
            logger.warning("find_object_at_pixel: No objects detected")
            return None

        x, y = pixel

        # First pass: Check for exact bbox containment
        for obj in objects:
            bbox = obj.get("bbox")
            if bbox:
                x1, y1, x2, y2 = bbox
                if x1 <= x <= x2 and y1 <= y <= y2:
                    logger.info(
                        f"Found object '{obj.get('class_name', 'unknown')}' at pixel ({x}, {y})"
                    )
                    return obj

        # Second pass: Find closest object center (if no exact match)
        min_distance = float("inf")
        closest_obj = None

        for obj in objects:
            bbox = obj.get("bbox")
            if bbox:
                x1, y1, x2, y2 = bbox
                # Calculate bbox center
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Distance from clicked pixel to object center
                distance = np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)

                if distance < min_distance:
                    min_distance = distance
                    closest_obj = obj

        # Return closest if within reasonable distance (100 pixels threshold)
        if closest_obj and min_distance < 100:
            logger.info(
                f"Found closest object '{closest_obj.get('class_name', 'unknown')}' "
                f"at distance {min_distance:.1f}px from pixel ({x}, {y})"
            )
            return closest_obj

        logger.warning(f"No object found near pixel ({x}, {y})")
        return None

    def stop(self):
        """Stop the pipeline and clean up resources."""
        self.running = False
        time.sleep(0.1)  # Give threads time to stop

        # Complete all subjects
        for subject in self.subjects.values():
            subject.on_completed()

        # Cleanup processor
        self.processor.cleanup()
        logger.info("ManipulationPipeline stopped")

    def cleanup(self):
        """Alias for stop() for consistency."""
        self.stop()
