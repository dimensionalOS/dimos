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
from typing import Any

import cv2
import numpy as np
import reactivex as rx
import reactivex.operators as ops
import websockets

from dimos.perception.common.utils import colorize_depth
from dimos.perception.grasp_generation.utils import draw_grasps_on_image
from dimos.manipulation.manip_aio_pipeline_new import ManipulationProcessor
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.manipulation_pipeline")


class ManipulationPipeline:
    """
    Pipeline with per-object grasp generation for VLM integration.
    Each detected object gets its own set of grasps.
    """

    def __init__(
        self,
        camera_configs: list[dict],
        min_confidence: float = 0.6,
        max_objects: int = 20,
        vocabulary: str | None = None,
        grasp_server_url: str | None = None,
        enable_grasp_generation: bool = False,
        enable_segmentation: bool = True,
        max_grasps_per_object: int = 5,
        min_points_for_grasp: int = 100,
    ) -> None:
        """
        Initialize the pipeline with per-object grasp support.
        
        Args:
            camera_configs: List of camera configurations
            min_confidence: Minimum detection confidence threshold
            max_objects: Maximum number of objects to process
            vocabulary: Optional vocabulary for Detic detector
            grasp_server_url: WebSocket URL for grasp server
            enable_grasp_generation: Whether to enable grasp generation
            enable_segmentation: Whether to enable semantic segmentation
            max_grasps_per_object: Maximum number of grasps to generate per object
            min_points_for_grasp: Minimum points in object for grasp generation
        """
        self.camera_configs = camera_configs
        self.num_cameras = len(camera_configs)
        self.enable_grasp_generation = enable_grasp_generation
        self.grasp_server_url = grasp_server_url
        self.max_grasps_per_object = max_grasps_per_object
        self.min_points_for_grasp = min_points_for_grasp


        # Initialize ManipulationProcessor
        self.processor = ManipulationProcessor(
            camera_configs=camera_configs,
            min_confidence=min_confidence,
            max_objects=max_objects,
            vocabulary=vocabulary,
            enable_grasp_generation=False,  # Pipeline handles grasps
            grasp_server_url=None,
            enable_segmentation=enable_segmentation,
        )

        # Streaming subjects
        self.filtered_objects_subject = rx.subject.Subject()
        self.objects_with_grasps_subject = rx.subject.Subject()
        self.grasp_overlay_subject = rx.subject.Subject()

        # Multi-camera frame buffering
        self.latest_frames = {
            cam_id: {"rgb": None, "depth": None}
            for cam_id in range(self.num_cameras)
        }
        self.frame_locks = {
            cam_id: threading.Lock()
            for cam_id in range(self.num_cameras)
        }
        self.sync_lock = threading.Lock()

        # Storage for objects and grasps
        self.latest_objects_with_grasps = []
        self.latest_rgb_for_overlay = None
        self.grasp_lock = threading.Lock()

        # Track pending grasp tasks
        self.pending_grasp_tasks = {}
        self.task_lock = threading.Lock()

        # Asyncio event loop for WebSocket
        self.grasp_loop = None
        self.grasp_loop_thread = None

        if self.enable_grasp_generation and self.grasp_server_url:
            self._start_grasp_loop()

        logger.info(
            f"Initialized ManipulationPipeline with {self.num_cameras} cameras, "
            f"per-object grasp generation={enable_grasp_generation}"
        )


    def create_streams(self, camera_streams: list[rx.Observable]) -> dict[str, rx.Observable]:
        """
        Create processing streams from multiple camera streams.
        """
        if len(camera_streams) != self.num_cameras:
            raise ValueError(f"Expected {self.num_cameras} camera streams, got {len(camera_streams)}")

        # Subscribe to each camera stream
        for cam_id, stream in enumerate(camera_streams):
            def create_camera_callback(camera_id):
                def on_camera_frame(frame_data):
                    if frame_data is not None:
                        with self.frame_locks[camera_id]:
                            self.latest_frames[camera_id]["rgb"] = frame_data.get("rgb")
                            self.latest_frames[camera_id]["depth"] = frame_data.get("depth")
                return on_camera_frame

            stream.subscribe(on_next=create_camera_callback(cam_id))

        # Create combined processing stream
        trigger_stream = camera_streams[0].pipe(ops.share())

        def process_synchronized_frames(trigger_frame) -> dict | None:
            """Process frames from all cameras with per-object grasps."""
            if trigger_frame is None:
                return None

            # Collect frames from all cameras
            rgb_images = []
            depth_images = []
            all_ready = True

            with self.sync_lock:
                for cam_id in range(self.num_cameras):
                    with self.frame_locks[cam_id]:
                        rgb = self.latest_frames[cam_id]["rgb"]
                        depth = self.latest_frames[cam_id]["depth"]
                    
                    if rgb is None or depth is None:
                        all_ready = False
                        break
                    
                    rgb_images.append(rgb)
                    depth_images.append(depth)

            if not all_ready:
                logger.debug("Not all camera frames ready, skipping")
                return None

            try:
                # Process with ManipulationProcessor
                processed = self.processor.process_frame(
                    rgb_images=rgb_images,
                    depth_images=depth_images,
                    generate_grasps=False
                )

                # Get merged objects in world coordinates
                if processed.get("all_objects"):
                    all_objects = processed["all_objects"]
                    
                    # Store for grasp overlay
                    with self.frame_locks[0]:
                        self.latest_rgb_for_overlay = rgb_images[0].copy()
                    
                    # Emit filtered objects immediately
                    self.filtered_objects_subject.on_next(all_objects)
                    
                    # Request per-object grasps if enabled
                    if self.enable_grasp_generation:
                        self._request_per_object_grasps_async(all_objects)

                return processed

            except Exception as e:
                logger.error(f"Error in multi-camera processing: {e}")
                return None

        # Create main processing stream
        processing_stream = trigger_stream.pipe(
            ops.map(process_synchronized_frames),
            ops.filter(lambda x: x is not None),
            ops.share()
        )

        # Visualization streams
        detection_viz_stream = processing_stream.pipe(
            ops.map(lambda x: x.get("per_camera_data", [{}])[0].get("detection_viz") 
                    if x.get("per_camera_data") else None),
            ops.filter(lambda x: x is not None)
        )

        pointcloud_viz_stream = processing_stream.pipe(
            ops.map(lambda x: x.get("pointcloud_viz")),
            ops.filter(lambda x: x is not None)
        )

        detected_viz_stream = processing_stream.pipe(
            ops.map(lambda x: x.get("detected_pointcloud_viz")),
            ops.filter(lambda x: x is not None)
        )

        misc_viz_stream = processing_stream.pipe(
            ops.map(lambda x: x.get("misc_pointcloud_viz")),
            ops.filter(lambda x: x is not None)
        )

        # Raw objects stream
        objects_stream = processing_stream.pipe(
            ops.map(lambda x: [obj for cam_data in x.get("per_camera_data", []) 
                             for obj in cam_data.get("detection2d_objects", [])]),
            ops.share()
        )

        return {
            "detection_viz": detection_viz_stream,
            "pointcloud_viz": pointcloud_viz_stream,
            "detected_viz": detected_viz_stream,
            "misc_viz": misc_viz_stream,
            "objects": objects_stream,
            "filtered_objects": self.filtered_objects_subject,
            "objects_with_grasps": self.objects_with_grasps_subject,
            "grasp_overlay": self.grasp_overlay_subject,
            "full_pointcloud": processing_stream.pipe(
                ops.map(lambda x: x.get("full_pointcloud")),
                ops.filter(lambda x: x is not None)
            ),
        }


    def _request_per_object_grasps_async(self, objects: list[dict]) -> None:
        """
        Request grasps for each individual object asynchronously.
        """
        if not self.enable_grasp_generation or not objects:
            return

        objects_with_pending_grasps = []
        
        for obj_idx, obj in enumerate(objects):
            # Check if object has sufficient points
            if obj.get('point_cloud') is None:
                continue
                
            points = np.asarray(obj['point_cloud'].points)
            if len(points) < self.min_points_for_grasp:
                logger.debug(f"Object {obj_idx} has insufficient points ({len(points)})")
                continue
            
            # Create unique ID for this object
            obj_id = f"obj_{obj_idx}_{time.time()}"
            obj['temp_id'] = obj_id
            
            # Prepare colors
            colors = None
            if hasattr(obj['point_cloud'], 'colors'):
                colors = np.asarray(obj['point_cloud'].colors)
            
            # Create grasp task for this object
            task = self._create_object_grasp_task(obj_id, points, colors)
            if task:
                with self.task_lock:
                    self.pending_grasp_tasks[obj_id] = {
                        'task': task,
                        'object': obj,
                        'index': obj_idx
                    }
                objects_with_pending_grasps.append(obj)

        if objects_with_pending_grasps:
            # Check for results after delay
            def check_object_grasps_later():
                time.sleep(2.0)  # Wait for grasp processing
                
                completed_objects = []
                with self.task_lock:
                    for obj_id, task_info in list(self.pending_grasp_tasks.items()):
                        try:
                            # Check if task is done
                            if task_info['task'].done():
                                grasps = task_info['task'].result(timeout=0.1)
                                if grasps:
                                    # Add grasps to object
                                    task_info['object']['grasps'] = grasps
                                    task_info['object']['has_grasps'] = True
                                else:
                                    task_info['object']['grasps'] = []
                                    task_info['object']['has_grasps'] = False
                                
                                completed_objects.append(task_info['object'])
                                del self.pending_grasp_tasks[obj_id]
                        except Exception as e:
                            logger.warning(f"Failed to get grasps for object {obj_id}: {e}")
                            task_info['object']['grasps'] = []
                            task_info['object']['has_grasps'] = False
                
                if completed_objects:
                    # Store objects with grasps
                    with self.grasp_lock:
                        self.latest_objects_with_grasps = completed_objects
                    
                    # Emit objects with their grasps
                    self.objects_with_grasps_subject.on_next(completed_objects)
                    
                    # Create visualization overlay if we have RGB
                    if self.latest_rgb_for_overlay is not None:
                        self._create_grasp_overlay(completed_objects)

            threading.Thread(target=check_object_grasps_later, daemon=True).start()

    def _create_object_grasp_task(
        self, obj_id: str, points: np.ndarray, colors: np.ndarray | None
    ) -> asyncio.Task | None:
        """Create an async task to request grasps for a single object."""
        if not self.grasp_loop:
            return None

        try:
            task = asyncio.run_coroutine_threadsafe(
                self._send_object_grasp_request(obj_id, points, colors),
                self.grasp_loop
            )
            return task
        except Exception as e:
            logger.error(f"Failed to create grasp task for {obj_id}: {e}")
            return None

    async def _send_object_grasp_request(
        self, obj_id: str, points: np.ndarray, colors: np.ndarray | None
    ) -> list[dict] | None:
        """Send grasp request for a single object."""
        try:
            # Validation
            if points is None or not isinstance(points, np.ndarray):
                return None
            if points.size == 0 or len(points.shape) != 2 or points.shape[1] != 3:
                return None
            if points.shape[0] < self.min_points_for_grasp:
                logger.debug(f"Object {obj_id} has too few points: {points.shape[0]}")
                return None

            # Prepare colors
            if colors is not None:
                if not isinstance(colors, np.ndarray) or colors.shape[0] != points.shape[0]:
                    colors = None
            
            if colors is None:
                colors = np.ones((points.shape[0], 3), dtype=np.float32) * 0.5

            # Ensure correct types
            points = points.astype(np.float32)
            colors = colors.astype(np.float32)

            # Validate data
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                return None
            if np.any(np.isnan(colors)) or np.any(np.isinf(colors)):
                return None

            colors = np.clip(colors, 0.0, 1.0)

            # Send request
            async with websockets.connect(self.grasp_server_url) as websocket:
                request = {
                    "points": points.tolist(),
                    "colors": colors.tolist(),
                    "lims": [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0],
                    "object_id": obj_id,  # Track which object
                    "max_grasps": self.max_grasps_per_object
                }

                await websocket.send(json.dumps(request))
                response = await websocket.recv()
                grasps = json.loads(response)

                # Validate response
                if isinstance(grasps, dict) and "error" in grasps:
                    logger.error(f"Grasp server error for {obj_id}: {grasps['error']}")
                    return None
                elif not isinstance(grasps, list):
                    return None
                elif len(grasps) == 0:
                    logger.debug(f"No grasps found for object {obj_id}")
                    return []

                # Convert and sort grasps
                converted_grasps = self._convert_grasp_format(grasps, obj_id)
                return converted_grasps

        except Exception as e:
            logger.error(f"Error requesting grasps for {obj_id}: {e}")
            return None

    def _create_grasp_overlay(self, objects_with_grasps: list[dict]) -> None:
        """Create visualization overlay showing grasps for all objects."""
        try:
            if self.latest_rgb_for_overlay is None:
                return

            bgr_image = cv2.cvtColor(self.latest_rgb_for_overlay, cv2.COLOR_RGB2BGR)
            
            # Draw grasps for each object with different colors
            for obj_idx, obj in enumerate(objects_with_grasps):
                if not obj.get('grasps'):
                    continue
                
                # Use different color for each object's grasps
                color = self._get_object_color(obj_idx)
                
                # Draw this object's grasps
                bgr_image = self._draw_object_grasps(
                    bgr_image, 
                    obj['grasps'], 
                    self.camera_configs[0]['intrinsics'],
                    color,
                    obj.get('class_name', f'Object {obj_idx}')
                )
            
            result_rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            self.grasp_overlay_subject.on_next(result_rgb)
            
        except Exception as e:
            logger.error(f"Error creating grasp overlay: {e}")

    def _draw_object_grasps(
        self, 
        image: np.ndarray, 
        grasps: list[dict], 
        intrinsics: list[float],
        color: tuple[int, int, int],
        label: str
    ) -> np.ndarray:
        """Draw grasps for a specific object with color and label."""
        for grasp in grasps:
            # Project grasp to image
            translation = grasp['translation']
            
            # Convert to camera frame if needed
            fx, fy, cx, cy = intrinsics
            if translation[2] > 0:  # Valid depth
                x = int(fx * translation[0] / translation[2] + cx)
                y = int(fy * translation[1] / translation[2] + cy)
                
                # Draw grasp marker
                cv2.circle(image, (x, y), 5, color, -1)
                cv2.circle(image, (x, y), 7, (255, 255, 255), 2)
                
                # Draw score
                score_text = f"{grasp['score']:.2f}"
                cv2.putText(image, score_text, (x + 10, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return image

    def _get_object_color(self, obj_idx: int) -> tuple[int, int, int]:
        """Get a unique color for each object."""
        colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Cyan
            (255, 128, 0),  # Orange
            (128, 0, 255),  # Purple
        ]
        return colors[obj_idx % len(colors)]

    def _start_grasp_loop(self) -> None:
        """Start asyncio event loop for WebSocket communication."""
        def run_loop() -> None:
            self.grasp_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.grasp_loop)
            self.grasp_loop.run_forever()

        self.grasp_loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.grasp_loop_thread.start()

        while self.grasp_loop is None:
            time.sleep(0.01)

    def _convert_grasp_format(self, grasps: list[dict], obj_id: str) -> list[dict]:
        """Convert grasp format and add object association."""
        converted = []

        for i, grasp in enumerate(grasps):
            rotation_matrix = np.array(grasp.get("rotation_matrix", np.eye(3)))
            euler_angles = self._rotation_matrix_to_euler(rotation_matrix)

            converted_grasp = {
                "id": f"{obj_id}_grasp_{i}",
                "object_id": obj_id,
                "score": grasp.get("score", 0.0),
                "width": grasp.get("width", 0.0),
                "height": grasp.get("height", 0.0),
                "depth": grasp.get("depth", 0.0),
                "translation": grasp.get("translation", [0, 0, 0]),
                "rotation_matrix": rotation_matrix.tolist(),
                "euler_angles": euler_angles,
            }
            converted.append(converted_grasp)

        # Sort by score
        converted.sort(key=lambda x: x["score"], reverse=True)
        
        # Limit to max grasps per object
        return converted[:self.max_grasps_per_object]

    def _rotation_matrix_to_euler(self, rotation_matrix: np.ndarray) -> dict[str, float]:
        """Convert rotation matrix to Euler angles."""
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

    def get_objects_with_grasps(self) -> list[dict]:
        """Get the latest objects with their computed grasps."""
        with self.grasp_lock:
            return self.latest_objects_with_grasps.copy()

    def find_object_at_pixel(
        self, pixel: tuple[int, int], camera_id: int = 0
    ) -> dict | None:
        """
        Find which object contains a given pixel (for VLM integration).
        
        Args:
            pixel: (x, y) pixel coordinates
            camera_id: Which camera view
            
        Returns:
            Object dict with grasps if found
        """
        with self.grasp_lock:
            for obj in self.latest_objects_with_grasps:
                # Check if pixel is in object's 2D bounding box
                if 'bbox' in obj:
                    x1, y1, x2, y2 = obj['bbox']
                    if x1 <= pixel[0] <= x2 and y1 <= pixel[1] <= y2:
                        return obj
        return None

    def select_best_grasp_for_object(self, object_id: str) -> dict | None:
        """Get the best grasp for a specific object."""
        with self.grasp_lock:
            for obj in self.latest_objects_with_grasps:
                if obj.get('temp_id') == object_id or obj.get('id') == object_id:
                    if obj.get('grasps'):
                        return obj['grasps'][0]  # Best grasp (highest score)
        return None

    def create_single_stream_adapter(self, single_stream: rx.Observable) -> list[rx.Observable]:
        """Adapter for single camera compatibility."""
        return [single_stream]

    def cleanup(self) -> None:
        """Clean up resources."""
        if hasattr(self.processor, "cleanup"):
            self.processor.cleanup()

        if self.grasp_loop and self.grasp_loop_thread:
            self.grasp_loop.call_soon_threadsafe(self.grasp_loop.stop)
            self.grasp_loop_thread.join(timeout=1.0)

        logger.info("ManipulationPipeline cleaned up")
