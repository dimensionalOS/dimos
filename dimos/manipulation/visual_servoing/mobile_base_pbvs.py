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
from collections import deque
import numpy as np
import cv2

from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.geometry_msgs import Twist, Vector3, Pose, Quaternion, Transform, PoseStamped
from dimos_lcm.vision_msgs import Detection3DArray, Detection2DArray
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.std_msgs import String

from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.manipulation.visual_servoing.pbvs import PBVSController
from dimos.manipulation.visual_servoing.utils import (
    match_detection_by_id,
    is_target_reached,
    find_best_object_match,
)
from dimos.perception.common.utils import find_clicked_detection, avg_detection3d
from dimos.protocol.tf import TF
from dimos.utils.transform_utils import (
    get_distance,
    yaw_towards_point,
    euler_to_quaternion,
    offset_distance,
)
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
    odom: In[PoseStamped] = None

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
        deadband_velocity: float = 0.2,  # Minimum linear velocity to overcome deadband
        deadband_angular_velocity: float = 0.2,  # Minimum angular velocity to overcome deadband
        target_distance: float = 1.2,  # Target distance to maintain from object
        target_tolerance: float = 0.2,  # 20cm tolerance
        min_confidence: float = 0.5,
        camera_frame_id: str = "camera_link",
        base_frame_id: str = "base_link",
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
        self.deadband_velocity = deadband_velocity
        self.deadband_angular_velocity = deadband_angular_velocity
        self.target_distance = target_distance
        self.target_tolerance = target_tolerance
        self.tracking_loss_timeout = tracking_loss_timeout

        # Frame IDs
        self.camera_frame_id = camera_frame_id
        self.track_frame_id = track_frame_id
        self.base_frame_id = base_frame_id
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
        self.target_object_history = deque(maxlen=4)  # Keep last 4 target positions
        self.last_detection_time = None
        self.tracking_thread = None
        self.stop_event = threading.Event()
        self.last_error_magnitude = None

        # Detection thread
        self.detection_thread = None
        self.stop_detection = threading.Event()
        self.detection_lock = threading.Lock()

        # Sensor data
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_odom = None
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
        self.odom.subscribe(self._on_odom)
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

        # Start detection thread
        self.stop_detection.clear()
        self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.detection_thread.start()

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

        # Wait for thread to finish
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=2.0)

        self.stop()

        logger.info("Stopped tracking")
        return {"status": "success", "message": "Tracking stopped"}

    def stop(self):
        self.is_tracking = False
        # Stop robot
        self._send_zero_velocity()

        # Stop detection thread
        if self.detection_thread and self.detection_thread.is_alive():
            self.stop_detection.set()

        # Clear state
        self.target_object = None
        self.target_object_history.clear()
        self.last_detection_time = None
        self.last_detections_2d = None
        self.last_detections_3d = None
        self.controller.clear_state()

        # Publish state
        if self.tracking_state:
            self.tracking_state.publish(String(data="stopped"))

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
                max_object_size=5.0,
            )
            logger.info("Initialized detector with new camera intrinsics")

    def _on_odom(self, msg: PoseStamped):
        """Handle odometry messages."""
        self.latest_odom = msg

    def _select_target(self, x: int, y: int) -> bool:
        """Select target object at given coordinates."""
        # Always process a fresh frame for selection
        self._process_frame()

        # Give it a second try if first attempt failed
        if not self.last_detections_2d or not self.last_detections_3d:
            time.sleep(0.1)  # Wait briefly for new sensor data
            self._process_frame()

        if not self.last_detections_2d or not self.last_detections_3d:
            logger.warning("No detections available after processing frame")
            return False

        # Find clicked detection
        clicked_3d = find_clicked_detection(
            (x, y), self.last_detections_2d.detections, self.last_detections_3d.detections
        )

        if clicked_3d and clicked_3d.bbox and clicked_3d.bbox.center:
            self.target_object = clicked_3d
            # Initialize history with the first detection
            self.target_object_history.clear()
            self.target_object_history.append(clicked_3d)
            self.last_detection_time = time.time()
            pos = clicked_3d.bbox.center.position
            logger.info(f"Selected target at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            return True

        return False

    def _detection_loop(self):
        """Detection loop running in separate thread."""
        logger.info("Detection loop started")

        while not self.stop_detection.is_set():
            try:
                self._process_frame()
                time.sleep(0.05)  # 20Hz detection rate
            except Exception as e:
                logger.error(f"Error in detection loop: {e}")

        logger.info("Detection loop ended")

    def _tracking_loop(self):
        """Main tracking loop running in separate thread."""
        logger.info("Tracking loop started")

        while not self.stop_event.is_set():
            try:
                # Update tracking (detection is done in separate thread)
                if not self._update_tracking():
                    # Check for tracking loss timeout
                    if self.last_detection_time:
                        time_since_detection = time.time() - self.last_detection_time
                        if time_since_detection > self.tracking_loss_timeout:
                            logger.warning("Lost tracking - timeout exceeded")
                            self.stop_event.set()
                            self.stop()
                            break

                self._publish_target_tf()
                self._compute_and_send_commands()
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
        self.target_object_history.clear()
        self.last_detection_time = None
        self.last_detections_2d = None
        self.last_detections_3d = None
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

            if not transform:
                logger.warning("No transform available")
                return

            # Process frame with numpy arrays
            # Detection3DProcessor will convert from optical to robot frame internally
            if self.latest_depth.format == ImageFormat.DEPTH16:
                depth_data = self.latest_depth.data.astype(np.float32) / 1000.0
            else:
                depth_data = self.latest_depth.data

            detections_3d, detections_2d = self.detector.process_frame(
                self.latest_rgb.data, depth_data, transform
            )

            # Thread-safe update of detections
            with self.detection_lock:
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
        if not self.target_object:
            return False

        # Thread-safe read of detections
        with self.detection_lock:
            if not self.last_detections_3d:
                return False
            detections_3d_copy = self.last_detections_3d

        # Use averaged target position for more stable matching
        averaged_target = (
            avg_detection3d(list(self.target_object_history))
            if self.target_object_history
            else self.target_object
        )

        # Use find_best_object_match for robust tracking
        match_result = find_best_object_match(
            target_obj=averaged_target,
            candidates=detections_3d_copy.detections,
            max_distance=0.2,  # 20cm tracking threshold
            min_size_similarity=0.75,  # Lower threshold for tracking
        )

        if match_result.is_valid_match:
            self.target_object = match_result.matched_object
            # Add to history for averaging
            self.target_object_history.append(match_result.matched_object)
            self.last_detection_time = time.time()
            logger.debug(
                f"Tracking updated: distance={match_result.distance:.3f}m, "
                f"confidence={match_result.confidence:.3f}"
            )
            return True

        logger.debug(
            f"Tracking lost: distance={match_result.distance:.3f}m, "
            f"confidence={match_result.confidence:.3f}"
        )
        return False

    def _compute_and_send_commands(self):
        """Compute and send velocity commands to mobile base."""
        # Use robot origin as end-effector pose
        ee_pose = Pose(position=Vector3(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))

        try:
            target_transform_in_base_frame = self.tf.get(
                parent_frame=self.base_frame_id,
                child_frame=self.target_frame_id,
                time_point=self.latest_odom.ts,
                time_tolerance=3.0,
            )
            if not target_transform_in_base_frame:
                logger.warning("No transform available for target in base frame")
                return
        except Exception as e:
            logger.error(f"Error getting target pose: {e}")
            return

        # Use target object position as target pose
        target_pose = target_transform_in_base_frame.to_pose()
        retracted_pose = offset_distance(
            target_pose, self.target_distance, approach_vector=Vector3(-1, 0, 0)
        )

        # Check if target reached first (position tolerance only for mobile base)
        error_magnitude, target_reached = is_target_reached(
            retracted_pose,
            ee_pose,
            position_tolerance=self.target_tolerance,
            orientation_tolerance=0.15,
        )
        self.last_error_magnitude = error_magnitude

        if target_reached:
            logger.info(f"Target reached! Error magnitude: {error_magnitude:.3f}m")
            # Signal to stop tracking instead of calling stop_track from within thread
            self.stop_event.set()
            self.stop()
            return

        # Compute control commands only if not reached
        twist_cmd = self.controller.compute_control(ee_pose, retracted_pose)

        # Apply deadband compensation
        linear_x = twist_cmd.linear.x
        linear_y = twist_cmd.linear.y
        angular_z = twist_cmd.angular.z

        # Apply deadband to linear velocities
        if abs(linear_x) > 0 and abs(linear_x) < self.deadband_velocity:
            linear_x = self.deadband_velocity if linear_x > 0 else -self.deadband_velocity
        if abs(linear_y) > 0 and abs(linear_y) < self.deadband_velocity:
            linear_y = self.deadband_velocity if linear_y > 0 else -self.deadband_velocity

        # Apply deadband to angular velocity
        if abs(angular_z) > 0 and abs(angular_z) < self.deadband_angular_velocity:
            angular_z = (
                self.deadband_angular_velocity if angular_z > 0 else -self.deadband_angular_velocity
            )

        mobile_twist = Twist(
            linear=Vector3(linear_x, linear_y, 0),
            angular=Vector3(0, 0, angular_z),
        )
        self.cmd_vel.publish(mobile_twist)

    def _send_zero_velocity(self):
        """Send zero velocity command to stop robot."""
        twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
        self.cmd_vel.publish(twist)

    def _publish_visualization(self):
        """Publish visualization image with tracking overlay."""
        if self.latest_rgb is None:
            return

        try:
            viz = self.latest_rgb.data.copy()

            # Draw target if tracking
            if self.target_object:
                # Thread-safe read of detections
                with self.detection_lock:
                    detections_2d_copy = self.last_detections_2d
                    detections_3d_copy = self.last_detections_3d

                # Check if detections match the current image timestamp
                image_ts = self.latest_rgb.ts if self.latest_rgb.ts else 0

                # Convert Time objects to float timestamps
                detections_ts_2d = (
                    detections_2d_copy.header.stamp.sec
                    + detections_2d_copy.header.stamp.nsec * 1e-9
                )
                detections_ts_3d = (
                    detections_3d_copy.header.stamp.sec
                    + detections_3d_copy.header.stamp.nsec * 1e-9
                )

                # Only draw if timestamps are reasonably close (within 100ms)
                timestamp_tolerance = 0.5  # seconds
                timestamps_match = (
                    abs(image_ts - detections_ts_2d) < timestamp_tolerance
                    and abs(image_ts - detections_ts_3d) < timestamp_tolerance
                )

                if detections_2d_copy and detections_3d_copy and timestamps_match:
                    # Use match_detection_by_id to find corresponding 2D detection
                    det_2d = match_detection_by_id(
                        self.target_object,
                        detections_3d_copy.detections,
                        detections_2d_copy.detections,
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

                        # Add error magnitude overlay
                        if self.last_error_magnitude is not None:
                            error_text = f"Error: {self.last_error_magnitude:.3f}m"
                            # Color based on error magnitude (green->yellow->red)
                            if self.last_error_magnitude < self.target_tolerance:
                                color = (0, 255, 0)  # Green - reached
                            elif self.last_error_magnitude < self.target_tolerance * 2:
                                color = (0, 255, 255)  # Yellow - close
                            else:
                                color = (0, 0, 255)  # Red - far

                            cv2.putText(
                                viz,
                                error_text,
                                (x, y + h + 20),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                color,
                                2,
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
            # Calculate target orientation: facing towards the object from robot position
            target_yaw = yaw_towards_point(
                self.target_object.bbox.center.position, self.latest_odom.position
            )
            target_euler = Vector3(0.0, 0.0, target_yaw)  # Only yaw, no roll/pitch
            target_orientation = euler_to_quaternion(target_euler)
            self.target_object.bbox.center.orientation = target_orientation
            target_tf = Transform(
                translation=self.target_object.bbox.center.position,
                rotation=self.target_object.bbox.center.orientation,
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

        # Stop detection thread if running
        if self.detection_thread and self.detection_thread.is_alive():
            self.stop_detection.set()
            self.detection_thread.join(timeout=2.0)

        logger.info("Mobile base PBVS module cleaned up")
