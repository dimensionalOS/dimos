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
import cv2
from manip_aio_processer_new_depthai import ManipulationProcessor
import numpy as np
import reactivex as rx
from reactivex import operators as ops
from reactivex.subject import Subject
import pyrealsense2 as rs
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

        self.last_grasp_time = 0
        self.grasp_interval = 5.0  # Generate grasps every 5 seconds

        logger.info(f"Initialized ManipulationPipeline with {self.num_cameras} cameras")


    def create_realsense_stream(self, pipeline: rs.pipeline, align: rs.align, camera_idx: int) -> rx.Observable:
        """Create an RxPy observable from RealSense pipeline."""
    
        def subscribe(observer, scheduler=None):
            def emit_frames():
                logger.info(f"Camera {camera_idx} RealSense stream started")
                
                while self.running:
                    try:
                        # Wait for frames with timeout
                        frames = pipeline.wait_for_frames(timeout_ms=100)
                        if not frames:
                            continue
                        
                        # Align depth to color
                        aligned_frames = align.process(frames)
                        
                        color_frame = aligned_frames.get_color_frame()
                        depth_frame = aligned_frames.get_depth_frame()
                        
                        if not color_frame or not depth_frame:
                            time.sleep(0.001)
                            continue
                        
                        # Get numpy arrays
                        rgb = np.asanyarray(color_frame.get_data())
                        depth = np.asanyarray(depth_frame.get_data())
                        
                        # Convert depth to meters
                        depth = depth.astype(np.float32) / 1000.0
                        
                        # Apply depth filtering for RealSense 435i
                        depth_filtered = depth.copy()
                        depth_filtered[depth_filtered < 0.3] = 0  # Min 30cm
                        depth_filtered[depth_filtered > 3.0] = 0  # Max 3m
                        depth = depth_filtered
                        
                        # No need for median blur - RealSense has better native quality
                        # No aggressive BGR conversion needed - RealSense gives RGB directly
                        
                        # Emit frame
                        observer.on_next({
                            "rgb": rgb,
                            "depth": depth, 
                            "camera_idx": camera_idx,
                        })
                        
                    except Exception as e:
                        if "timeout" not in str(e).lower():
                            #logger.error(f"Camera {camera_idx} stream error: {e}")
                            time.sleep(0.001)
                        
                observer.on_completed()
            
            thread = threading.Thread(target=emit_frames, daemon=True)
            thread.start()
        
        return rx.create(subscribe)

    def create_depthai_stream(self, color_queue: Any, depth_queue: Any, camera_idx: int) -> rx.Observable:
        """
        Create an RxPy observable from DepthAI queues.
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
                            
                            # ============ ADD DEBUG HERE ============
                            # DEBUG: Check raw depth from camera (before any processing)
                            print(f"Camera {camera_idx} RAW depth: min={depth.min()}, max={depth.max()} (in mm)")
                            # ========================================
                            
                            # Convert BGR to RGB if needed
                            if rgb.shape[-1] == 3:
                                rgb = rgb[:, :, ::-1]  # BGR to RGB
                            
                            # ADD MEDIAN FILTER HERE for OAK-D SR noise reduction
                            depth = cv2.medianBlur(depth.astype(np.uint16), 5)
                            
                            # Convert depth to meters
                            depth = depth.astype(np.float32) / 1000.0

                                                        # ============ AGGRESSIVE FILTERING FOR OAK-D SR ============
                            # Apply BEFORE any other processing
                            depth_filtered = depth.copy()
                            depth_filtered[depth_filtered < 0.2] = 0  # Min 20cm
                            depth_filtered[depth_filtered > 1.0] = 0  # Max 1m STRICT!
                            depth = depth_filtered  # Replace depth with filtered version
                            # ========================================================
                            
                            # ============ ADD DEBUG HERE ============
                            # DEBUG: Check after conversion to meters
                            valid_depth = depth[depth > 0]
                            if len(valid_depth) > 0:
                                print(f"Camera {camera_idx} depth in meters: min={valid_depth.min():.3f}m, max={valid_depth.max():.3f}m, mean={valid_depth.mean():.3f}m")
                                # Additional range analysis
                                near = np.sum((depth > 0.2) & (depth <= 0.5))
                                mid = np.sum((depth > 0.5) & (depth <= 1.0))
                                far = np.sum(depth > 1.0)
                                print(f"  Pixels by range: 20-50cm: {near}, 50-100cm: {mid}, >100cm: {far}")
                            # ========================================
                            
                            # FIX: Ensure matching dimensions
                            if rgb.shape[:2] != depth.shape[:2]:
                                h, w = rgb.shape[:2]
                                depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
                                logger.debug(f"Camera {camera_idx}: Resized depth to match RGB {w}x{h}")
                            
                            # Emit frame
                            observer.on_next({
                                "rgb": rgb,
                                "depth": depth,
                                "camera_idx": camera_idx,
                            })
                        else:
                            # No frames available, wait a bit
                            time.sleep(0.001)
                            
                    except Exception as e:
                        #logger.error(f"Camera {camera_idx} stream error: {e}")
                        observer.on_error(e)
                        break
                        
                observer.on_completed()

            # Start thread
            thread = threading.Thread(target=emit_frames, daemon=True)
            thread.start()

        return rx.create(subscribe)

    def process_frame_async(self, generate_grasps: bool = False):
        """Process current buffered frames asynchronously."""
        if self.processing:
            return
        
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
        
        # REMOVED: Automatic periodic grasp generation
        # Now only generate grasps when explicitly requested
        should_generate_grasps = generate_grasps  # Direct control only
        
        def process():
            try:
                results = self.processor.process_frame(
                    rgb_images=rgb_images,
                    depth_images=depth_images,
                    generate_grasps=should_generate_grasps  # Only when explicitly requested
                )
                self._emit_results(results)
            except Exception as e:
                logger.error(f"Frame processing error: {e}")
                import traceback
                traceback.print_exc()
            finally:
                self.processing = False
        
        threading.Thread(target=process, daemon=True).start()

    def _emit_results(self, results: dict):
        """Emit processing results to output subjects."""
        # Emit detection visualization directly from results
        if "detection_viz" in results and results["detection_viz"] is not None:
            self.subjects["detection_viz"].on_next(results["detection_viz"])

        # Emit other visualizations
        if "pointcloud_viz" in results:
            self.subjects["pointcloud_viz"].on_next(results["pointcloud_viz"])

        if "detected_pointcloud_viz" in results:
            self.subjects["detected_pointcloud_viz"].on_next(results["detected_pointcloud_viz"])

        if "misc_pointcloud_viz" in results:
            self.subjects["misc_pointcloud_viz"].on_next(results["misc_pointcloud_viz"])

        # Emit segmentation viz if available from first camera
        if "segmentation_viz" in results and results["segmentation_viz"] is not None:
            self.subjects["segmentation_viz"].on_next(results["segmentation_viz"])

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
        camera_inputs: list[tuple[Any, Any]],  # Can be DepthAI queues OR RealSense (pipeline, align)
        input_type: str = "depthai"  # "depthai" or "realsense"
    ) -> dict[str, rx.Observable]:
        """Create streaming pipeline from camera inputs."""
        
        self.running = True
        camera_streams = []
        
        for idx, inputs in enumerate(camera_inputs):
            if input_type == "depthai":
                color_queue, depth_queue = inputs
                stream = self.create_depthai_stream(color_queue, depth_queue, idx)
            elif input_type == "realsense":
                pipeline, align = inputs
                stream = self.create_realsense_stream(pipeline, align, idx)
            else:
                raise ValueError(f"Unknown input type: {input_type}")
            
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

    def generate_grasp_for_object_at_pixel(self, pixel: tuple[int, int], camera_id: int = 0) -> dict | None:
        """Generate grasps ONLY for the object at specified pixel."""
        
        print(f"🔍 DEBUG: generate_grasp_for_object_at_pixel called with pixel {pixel}")
        
        with self.lock:
            print(f"🔍 DEBUG: Lock acquired in generate_grasp")
            if self.latest_rgb_images[camera_id] is None or self.latest_depth_images[camera_id] is None:
                print(f"🔍 DEBUG: No images available")
                return None
            
            rgb_images = [img.copy() for img in self.latest_rgb_images]
            depth_images = [img.copy() for img in self.latest_depth_images]
            print(f"🔍 DEBUG: Copied images")
        
        print(f"🔍 DEBUG: Calling processor.process_frame WITHOUT grasps...")
        # Process frame WITHOUT grasps first
        results = self.processor.process_frame(
            rgb_images=rgb_images,
            depth_images=depth_images,
            generate_grasps=False  # No automatic grasps
        )
        print(f"🔍 DEBUG: process_frame returned")
        
        # Find object at pixel
        all_objects = results.get("all_objects", [])
        print(f"🔍 DEBUG: Found {len(all_objects)} total objects")
        
        x, y = pixel
        target_object = None
        
        for i, obj in enumerate(all_objects):
            bbox = obj.get("bbox")
            if bbox is not None and len(bbox) >= 4:
                x1, y1, x2, y2 = bbox[:4]
                print(f"🔍 DEBUG: Object {i} bbox: ({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f})")
                if x1 <= x <= x2 and y1 <= y <= y2:
                    target_object = obj
                    print(f"🔍 DEBUG: Found target object at index {i}")
                    break
        
        if not target_object:
            logger.warning(f"No object found at pixel ({x}, {y})")
            print(f"🔍 DEBUG: No object found at pixel ({x}, {y})")
            return None
        
        logger.info(f"Found {target_object.get('class_name', 'unknown')} at pixel ({x}, {y})")
        print(f"🔍 DEBUG: Found {target_object.get('class_name', 'unknown')} - checking for point cloud")
        
        # Generate grasps for THIS object
        if self.processor.grasp_generator and target_object.get('point_cloud'):
            print(f"🔍 DEBUG: Starting grasp generation...")
            logger.info(f"Generating grasps for single object...")
            grasps = self.processor.grasp_generator.generate_grasps_from_objects(
                [target_object],
                results.get("full_pointcloud")
            )
            print(f"🔍 DEBUG: Generated {len(grasps) if grasps else 0} grasps")
            target_object['grasps'] = grasps
            
            # EMIT GRASP VISUALIZATION - THIS IS THE KEY PART!
            if grasps:
                print(f"🔍 DEBUG: Creating and emitting grasp visualization...")
                # Create grasp overlay visualization
                from dimos.perception.grasp_generation.utils import create_grasp_overlay
                intrinsics_1d = self.camera_configs[camera_id]['intrinsics']
                intrinsics_3x3 = np.array([
                    [intrinsics_1d[0], 0, intrinsics_1d[2]],
                    [0, intrinsics_1d[1], intrinsics_1d[3]],
                    [0, 0, 1]
                ])
                
                grasp_viz = create_grasp_overlay(
                    rgb_images[camera_id], 
                    grasps, 
                    intrinsics_3x3
                )
                
                # Emit to visualization stream
                logger.info(f"Emitting grasp visualization with {len(grasps)} grasps")
                self.subjects["grasp_overlay"].on_next(grasp_viz)
                print(f"🔍 DEBUG: ✓ Grasp visualization emitted!")
            else:
                print(f"🔍 DEBUG: No grasps to visualize")
            
            return target_object
        else:
            print(f"🔍 DEBUG: Missing grasp_generator or point_cloud")
            return None

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
