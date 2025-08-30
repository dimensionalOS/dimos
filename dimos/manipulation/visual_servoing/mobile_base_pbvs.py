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
Mobile base Position-Based Visual Servoing module for object tracking and following.
Uses PBVS controller to compute velocity commands for mobile base control.
"""

import time
import threading
from typing import Dict, Any
import numpy as np
import cv2

from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.geometry_msgs import Twist, Vector3, Pose, Quaternion, Transform
from dimos_lcm.vision_msgs import Detection3DArray, Detection2DArray
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.std_msgs import String

from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.manipulation.visual_servoing.pbvs import PBVSController
from dimos.manipulation.visual_servoing.utils import match_detection_by_id
from dimos.perception.common.utils import find_clicked_detection
from dimos.protocol.tf import TF
from dimos.utils.transform_utils import get_distance
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.manipulation.visual_servoing.mobile_base_pbvs")


class MobileBasePBVS(Module):
    """
    Mobile base visual servoing module for tracking and following objects.

    Subscribes to:
        - RGB images (for detection and visualization)
        - Depth images (for 3D detection)
        - Camera info (for intrinsics)

    Publishes:
        - Detection3DArray (3D object detections)
        - Detection2DArray (2D object detections)
        - Twist commands (velocity commands for mobile base)
        - Visualization images
        - Tracking state

    RPC methods:
        - start: Start tracking at given pixel coordinates
        - stop: Stop tracking and servoing
    """

    # LCM inputs
    rgb_image: In[Image] = None
    depth_image: In[Image] = None
    camera_info: In[CameraInfo] = None

    # LCM outputs
    viz_image: Out[Image] = None
    cmd_vel: Out[Twist] = None
    tracking_state: Out[String] = None
    detection3d_array: Out[Detection3DArray] = None
    detection2d_array: Out[Detection2DArray] = None

    def __init__(
        self,
        position_gain: float = 0.4,
        rotation_gain: float = 0.5,
        max_linear_velocity: float = 0.6,  # m/s
        max_angular_velocity: float = 0.8,  # rad/s
        target_distance: float = 1.2,  # Target distance to maintain from object
        target_tolerance: float = 0.2,  # 20cm tolerance
        min_confidence: float = 0.5,
        camera_frame_id: str = "camera_link",
        track_frame_id: str = "world",
        tracking_loss_timeout: float = 2.0,
        **kwargs,
    ):
        """Initialize mobile base PBVS module."""
        super().__init__(**kwargs)

        # Control parameters
        self.position_gain = position_gain
        self.rotation_gain = rotation_gain
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.target_distance = target_distance
        self.target_tolerance = target_tolerance
        self.tracking_loss_timeout = tracking_loss_timeout

        # Frame IDs
        self.camera_frame_id = camera_frame_id
        self.track_frame_id = track_frame_id
        self.target_frame_id = "target"

        # Initialize components
        self.tf = TF()
        self.detector = None
        self.controller = PBVSController(
            position_gain=position_gain,
            rotation_gain=rotation_gain,
            max_velocity=max_linear_velocity,
            max_angular_velocity=max_angular_velocity,
            target_tolerance=target_tolerance,
        )

        # Tracking state
        self.is_tracking = False
        self.target_object = None
        self.last_detection_time = None
        self.tracking_thread = None
        self.stop_event = threading.Event()

        # Sensor data
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_intrinsics = None
        self.last_detections_3d = None
        self.last_detections_2d = None

        # Detection parameters
        self.min_confidence = min_confidence
        self.max_detection_distance = 5.0  # Maximum detection distance in meters

        logger.info(f"Initialized MobileBasePBVS")

    @rpc
    def start(self):
        """Start the module and subscribe to input streams."""
        # Subscribe to input streams
        self.rgb_image.subscribe(self._on_rgb_image)
        self.depth_image.subscribe(self._on_depth_image)
        self.camera_info.subscribe(self._on_camera_info)
        logger.info("Mobile base PBVS module started")

    @rpc
    def track(self, target_x: int = None, target_y: int = None) -> Dict[str, Any]:
        """
        Start tracking and following an object at given coordinates.

        Args:
            target_x: X coordinate of target object in image
            target_y: Y coordinate of target object in image

        Returns:
            Dict with status and message
        """
        if self.is_tracking:
            return {"status": "error", "message": "Already tracking"}

        if target_x is None or target_y is None:
            return {"status": "error", "message": "Target coordinates required"}

        # Find and select target object
        if not self._select_target(target_x, target_y):
            return {"status": "error", "message": "No object found at coordinates"}

        # Start tracking thread
        self.stop_event.clear()
        self.is_tracking = True
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()

        # Publish state
        if self.tracking_state:
            self.tracking_state.publish(String(data="tracking"))

        logger.info(f"Started tracking object at ({target_x}, {target_y})")
        return {"status": "success", "message": "Tracking started"}

    @rpc
    def stop_track(self) -> Dict[str, Any]:
        """Stop tracking and servoing."""
        if not self.is_tracking:
            return {"status": "warning", "message": "Not currently tracking"}

        # Stop tracking
        self.stop_event.set()
        self.is_tracking = False

        # Wait for thread to finish
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=2.0)

        # Stop robot
        self._send_zero_velocity()

        # Clear state
        self.target_object = None
        self.last_detection_time = None
        self.controller.clear_state()

        # Publish state
        if self.tracking_state:
            self.tracking_state.publish(String(data="stopped"))

        logger.info("Stopped tracking")
        return {"status": "success", "message": "Tracking stopped"}

    def _on_rgb_image(self, msg: Image):
        """Handle RGB image messages."""
        self.latest_rgb = msg

    def _on_depth_image(self, msg: Image):
        """Handle depth image messages."""
        self.latest_depth = msg

    def _on_camera_info(self, msg: CameraInfo):
        """Handle camera info messages."""
        intrinsics = [msg.K[0], msg.K[4], msg.K[2], msg.K[5]]
        if self.detector is None or self.camera_intrinsics != intrinsics:
            self.camera_intrinsics = intrinsics
            self.detector = Detection3DProcessor(
                camera_intrinsics=self.camera_intrinsics,
                min_confidence=self.min_confidence,
                max_depth=self.max_detection_distance,
            )
            logger.info("Initialized detector with new camera intrinsics")

    def _select_target(self, x: int, y: int) -> bool:
        """Select target object at given coordinates."""
        if not self.last_detections_2d or not self.last_detections_3d:
            # Process one frame to get detections
            self._process_frame()

        if not self.last_detections_2d or not self.last_detections_3d:
            logger.warning("No detections available")
            return False

        # Find clicked detection
        clicked_3d = find_clicked_detection(
            (x, y), self.last_detections_2d.detections, self.last_detections_3d.detections
        )

        if clicked_3d and clicked_3d.bbox and clicked_3d.bbox.center:
            self.target_object = clicked_3d
            self.last_detection_time = time.time()
            pos = clicked_3d.bbox.center.position
            logger.info(f"Selected target at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            return True

        return False

    def _tracking_loop(self):
        """Main tracking loop running in separate thread."""
        logger.info("Tracking loop started")

        while not self.stop_event.is_set():
            try:
                # Process current frame
                self._process_frame()

                # Update tracking
                if not self._update_tracking():
                    # Check for tracking loss timeout
                    if self.last_detection_time:
                        time_since_detection = time.time() - self.last_detection_time
                        if time_since_detection > self.tracking_loss_timeout:
                            logger.warning("Lost tracking - timeout exceeded")
                            # Don't call stop_track() from within the thread - just reset state and exit
                            self._reset_tracking_state()
                            break

                # Compute and send velocity commands
                self._compute_and_send_commands()

                # Publish target TF
                self._publish_target_tf()

                # Publish visualization
                self._publish_visualization()

                time.sleep(0.05)  # 20Hz control loop

            except Exception as e:
                logger.error(f"Error in tracking loop: {e}")
                break

        logger.info("Tracking loop ended")

    def _reset_tracking_state(self):
        """Reset tracking state without thread operations."""
        # Stop robot
        self._send_zero_velocity()

        # Clear state
        self.is_tracking = False
        self.target_object = None
        self.last_detection_time = None
        self.controller.clear_state()

        # Publish state
        if self.tracking_state:
            self.tracking_state.publish(String(data="stopped"))

    def _process_frame(self):
        """Process current frame to get detections."""
        if not all([self.latest_rgb is not None, self.latest_depth is not None, self.detector]):
            return

        try:
            # Get camera to base transform
            transform = self.tf.get(
                parent_frame=self.track_frame_id,
                child_frame=self.camera_frame_id,
                time_point=self.latest_rgb.ts,
                time_tolerance=0.2,
            )

            # Process frame with numpy arrays
            # Detection3DProcessor will convert from optical to robot frame internally
            if self.latest_depth.format == ImageFormat.DEPTH16:
                depth_data = self.latest_depth.data.astype(np.float32) / 1000.0
            else:
                depth_data = self.latest_depth.data

            detections_3d, detections_2d = self.detector.process_frame(
                self.latest_rgb.data, depth_data, transform
            )

            self.last_detections_3d = detections_3d
            self.last_detections_2d = detections_2d

            # Publish detections
            if self.detection3d_array:
                self.detection3d_array.publish(detections_3d)
            if self.detection2d_array:
                self.detection2d_array.publish(detections_2d)

        except Exception as e:
            logger.error(f"Error processing frame: {e}")

    def _update_tracking(self) -> bool:
        """Update tracking of target object."""
        if not self.target_object or not self.last_detections_3d:
            return False

        # Find best matching detection
        best_match = None
        min_distance = float("inf")

        for detection in self.last_detections_3d.detections:
            if not detection.bbox or not detection.bbox.center:
                continue

            # Calculate distance between current and target positions
            distance = get_distance(self.target_object.bbox.center, detection.bbox.center)

            # Update best match if within tracking threshold
            if distance < min_distance and distance < 0.2:  # 30cm tracking threshold
                min_distance = distance
                best_match = detection

        if best_match:
            self.target_object = best_match
            self.last_detection_time = time.time()
            return True

        return False

    def _compute_and_send_commands(self):
        """Compute and send velocity commands to mobile base."""
        if not self.target_object or not self.is_tracking:
            self._send_zero_velocity()
            return

        # Create virtual end-effector pose at robot base
        ee_pose = Pose(position=Vector3(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))

        # Create target pose at desired distance
        target_pos = self.target_object.bbox.center.position
        target_direction = np.array([target_pos.x, target_pos.y, 0])  # Project to XY plane
        target_norm = np.linalg.norm(target_direction)

        if target_norm > 0:
            target_direction = target_direction / target_norm * self.target_distance

        target_pose = Pose(
            position=Vector3(target_direction[0], target_direction[1], 0),
            orientation=Quaternion(0, 0, 0, 1),
        )

        # Compute control commands
        linear_vel, angular_vel, _ = self.controller.compute_control(ee_pose, target_pose)

        if linear_vel and angular_vel:
            # Convert to mobile base commands (only use x linear and z angular)
            twist = Twist(linear=Vector3(linear_vel.x, 0, 0), angular=Vector3(0, 0, angular_vel.z))

            if self.cmd_vel:
                self.cmd_vel.publish(twist)

    def _send_zero_velocity(self):
        """Send zero velocity command to stop robot."""
        if self.cmd_vel:
            twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            self.cmd_vel.publish(twist)

    def _publish_visualization(self):
        """Publish visualization image with tracking overlay."""
        if self.latest_rgb is None:
            return

        try:
            viz = self.latest_rgb.data.copy()

            # Draw target if tracking
            if self.target_object and self.last_detections_2d and self.last_detections_3d:
                # Use match_detection_by_id to find corresponding 2D detection
                det_2d = match_detection_by_id(
                    self.target_object,
                    self.last_detections_3d.detections,
                    self.last_detections_2d.detections,
                )

                if det_2d and det_2d.bbox:
                    # Draw bounding box
                    bbox = det_2d.bbox
                    x = int(bbox.center.position.x - bbox.size_x / 2)
                    y = int(bbox.center.position.y - bbox.size_y / 2)
                    w = int(bbox.size_x)
                    h = int(bbox.size_y)

                    cv2.rectangle(viz, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Add tracking info
                    pos = self.target_object.bbox.center.position
                    text = f"Tracking: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                    cv2.putText(
                        viz, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
                    )

            # Publish visualization
            if self.viz_image and viz is not None:
                self.viz_image.publish(Image.from_numpy(viz))

        except Exception as e:
            logger.error(f"Error publishing visualization: {e}")

    def _publish_target_tf(self):
        """Publish TF transform for the tracked target."""
        if not self.target_object or not self.is_tracking:
            return

        try:
            # Get target position in base frame
            target_pos = self.target_object.bbox.center.position

            # Create transform from base to target (following object_tracker.py pattern)
            target_tf = Transform(
                translation=Vector3(target_pos.x, target_pos.y, target_pos.z),
                rotation=Quaternion(0, 0, 0, 1),  # Identity rotation
                frame_id=self.track_frame_id,  # Parent frame
                child_frame_id=self.target_frame_id,  # Child frame
                ts=time.time(),
            )

            # Publish transform
            self.tf.publish(target_tf)

        except Exception as e:
            logger.error(f"Error publishing target TF: {e}")

    @rpc
    def cleanup(self):
        """Clean up resources and stop tracking."""
        # Stop any active tracking
        if self.is_tracking:
            self.stop_track()

        # Stop tracking thread if running
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.stop_event.set()
            self.tracking_thread.join(timeout=2.0)

        logger.info("Mobile base PBVS module cleaned up")
